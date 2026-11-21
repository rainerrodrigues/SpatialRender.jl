# Neighbour search without per-point allocations.
#
# `KHeap` is a fixed-size max-heap for the k smallest distances. `GridNeighbors`
# sorts points into a uniform grid (cell size ≈ 2 × mean spacing) and answers
# k-NN / radius queries by visiting cell shells outward until the k-th
# distance is final. Building it costs ~24 bytes per point (sorted positions,
# original indices, cell table); queries allocate nothing.

"""
    threaded_chunks(kernel!, n, args...)

Run `kernel!(range, args...)` over chunks of `1:n` on all threads. Unlike a
`Threads.@threads` loop body, the kernel is a named function with explicit
arguments: on Julia 1.12, calls made from inside a `@threads` closure were
not specialised and boxed their arguments on every iteration (~130 bytes
per point in the filters).
"""
function threaded_chunks(kernel!::F, n::Integer, args::Vararg{Any, N}) where {F, N}
    # (::F and Vararg{Any, N} force specialisation: a function argument that is
    # only passed along, or plain varargs, would otherwise compile generically)
    n == 0 && return
    @sync for r in Iterators.partition(1:n, cld(n, 4 * Threads.nthreads()))
        Threads.@spawn kernel!(r, args...)
    end
end

"Bounded max-heap keeping the `k` smallest (distance², index) pairs."
mutable struct KHeap
    d::Vector{Float32}
    i::Vector{Int32}
    n::Int
end
KHeap(k::Integer) = KHeap(fill(Inf32, k), zeros(Int32, k), 0)
isfull(h::KHeap) = h.n == length(h.d)
top(h::KHeap) = h.n == 0 ? Inf32 : h.d[1]
reset!(h::KHeap) = (h.n = 0; h)
Base.length(h::KHeap) = h.n

@inline function Base.push!(h::KHeap, d::Float32, i::Int32)
    k = length(h.d)
    if h.n < k
        h.n += 1
        j = h.n
        @inbounds while j > 1                       # sift up
            p = j >> 1
            h.d[p] >= d && break
            h.d[j] = h.d[p]; h.i[j] = h.i[p]; j = p
        end
        @inbounds h.d[j] = d; h.i[j] = i
    elseif d < h.d[1]
        j = 1                                         # replace root, sift down
        @inbounds while true
            l = 2j; l > k && break
            r = l + 1
            c = (r <= k && h.d[r] > h.d[l]) ? r : l
            h.d[c] <= d && break
            h.d[j] = h.d[c]; h.i[j] = h.i[c]; j = c
        end
        @inbounds h.d[j] = d; h.i[j] = i
    end
    return h
end

"Contents sorted by distance: (indices, distances²)."
function sorted(h::KHeap)
    p = sortperm(view(h.d, 1:h.n))
    return h.i[p], h.d[p]
end

struct GridNeighbors
    h::Float32
    lo::Vec3f
    dims::NTuple{3, Int}
    pos::Vector{Vec3f}                  # positions sorted by cell
    orig::Vector{Int32}                 # original index of each sorted position
    starts::Vector{Int32}               # dense mode: cell c holds pos[starts[c+1]+1 : starts[c+2]]
    cells::Dict{Int64, UnitRange{Int32}} # sparse mode (grids with more than 4 cells per point)
    noccupied::Int
end

"""
    GridNeighbors(points, h=nothing)

Grid over `points` (an `AbstractVector` of 3-vectors) with cell size `h`.
By default `h` targets about 8 points per occupied cell, which suits both
volumes and the surface-like clouds surveys produce.
"""
function GridNeighbors(pts::AbstractVector, h=nothing)
    n = length(pts)
    b = bounds_of(pts)
    e = max.(extent(b), 1f-6)
    if h === nothing
        hmin = maximum(e) / 4096                    # bound the grid size
        h = max(2f0 * cbrt(prod(e) / max(n, 1)), hmin)
        # Survey data lies on surfaces, so the volume estimate makes cells too
        # big: measure the occupancy and shrink once (surface: count ∝ h²).
        m = n / max(_occupied_cells(pts, b, Float32(h)), 1)
        m > 16 && (h = max(h * sqrt(8 / m), hmin))
    end
    return _grid(pts, b, Float32(h))                # function barrier: `h` concretely typed
end

function _cellkeys(pts, b::AABB, h::Float32, dims)
    keys = Vector{Int64}(undef, length(pts))
    threaded_chunks(_cellkey_kernel!, length(pts), keys, pts, b.lo, h, dims)
    return keys
end
function _cellkey_kernel!(range, keys, pts, lo, h, dims)
    @inbounds for i in range
        keys[i] = _gridkey(pts[i], lo, h, dims)
    end
end
_griddims(b::AABB, h::Float32) = Tuple(max.(ceil.(Int, max.(extent(b), 1f-6) ./ h), 1))

function _occupied_cells(pts, b::AABB, h::Float32)
    keys = sort!(_cellkeys(pts, b, h, _griddims(b, h)))
    return isempty(keys) ? 0 : 1 + count(i -> keys[i] != keys[i-1], 2:length(keys))
end

function _grid(pts::AbstractVector, b::AABB, h::Float32)
    n = length(pts)
    dims = _griddims(b, h)
    keys = _cellkeys(pts, b, h, dims)
    pos = Vector{Vec3f}(undef, n); orig = Vector{Int32}(undef, n)
    ncell = prod(dims)
    if ncell <= 4 * max(n, 1)
        # dense: counting sort straight into place, O(1) cell lookups
        starts = zeros(Int32, ncell + 1)
        @inbounds for k in keys
            starts[k+2] += 1
        end
        nocc = count(>(0), view(starts, 2:ncell+1))
        cumsum!(starts, starts)
        next = starts[1:ncell]                      # write cursor per cell
        @inbounds for i in 1:n
            j = (next[keys[i]+1] += 1)
            pos[j] = Vec3f(pts[i]); orig[j] = i
        end
        return GridNeighbors(h, b.lo, dims, pos, orig, starts, Dict{Int64, UnitRange{Int32}}(), nocc)
    end
    perm = sortperm(keys)                           # sparse: sort by cell, hash the ranges
    cells = Dict{Int64, UnitRange{Int32}}()
    sizehint!(cells, n ÷ 4)
    start = 1
    @inbounds for j in 1:n
        pos[j] = Vec3f(pts[perm[j]]); orig[j] = perm[j]
        if j == n || keys[perm[j+1]] != keys[perm[j]]
            cells[keys[perm[j]]] = Int32(start):Int32(j)
            start = j + 1
        end
    end
    return GridNeighbors(h, b.lo, dims, pos, orig, Int32[], cells, length(cells))
end

@inline function _gridkey(p, lo::Vec3f, h::Float32, dims::NTuple{3, Int})
    q = clamp.(floor.(Int, (Vec3f(p) - lo) ./ h), 0, SVector(dims) .- 1)
    return q[1] + dims[1] * (q[2] + dims[2] * q[3])
end

@inline _cell(g::GridNeighbors, p) = clamp.(floor.(Int, (p - g.lo) ./ g.h), 0, SVector(g.dims) .- 1)

"Points of cell `key` as a range into `g.pos` (empty when unoccupied)."
@inline function _cellrange(g::GridNeighbors, key::Int)
    if !isempty(g.starts)
        @inbounds return (g.starts[key+1] + Int32(1)):g.starts[key+2]
    end
    return get(g.cells, key, Int32(1):Int32(0))
end

"""
    knn!(heap, g, p, exclude=0) -> heap

Fill `heap` (a `KHeap`) with the nearest points to `p`, skipping original
index `exclude`. Allocation-free.
"""
# @inline: called from threaded kernels, a non-inlined knn! had its arguments
# boxed on every call (Julia 1.12), costing ~130 bytes per query
@inline function knn!(heap::KHeap, g::GridNeighbors, p::Vec3f, exclude::Integer=0)
    reset!(heap)
    c = _cell(g, p)
    smax = maximum(g.dims)
    for s in 0:smax
        if (2s + 1)^3 > 4 * g.noccupied
            # shells now cost more than visiting every occupied cell once:
            # finish with one pass over the cells, pruned by cell distance
            _knn_all_cells!(heap, g, p, exclude)
            break
        end
        @inbounds for dz in -s:s, dy in -s:s, dx in -s:s
            max(abs(dx), abs(dy), abs(dz)) == s || continue
            q = c + SVector(dx, dy, dz)
            (all(q .>= 0) && all(q .< SVector(g.dims))) || continue
            for j in _cellrange(g, q[1] + g.dims[1] * (q[2] + g.dims[2] * q[3]))
                g.orig[j] == exclude && continue
                push!(heap, sum(abs2, g.pos[j] - p), g.orig[j])
            end
        end
        # every point in shell s+1 is at least s·h away
        isfull(heap) && top(heap) <= (s * g.h)^2 && break
    end
    return heap
end

function _knn_all_cells!(heap::KHeap, g::GridNeighbors, p::Vec3f, exclude)
    ks = isempty(g.starts) ? collect(keys(g.cells)) : (0:prod(g.dims)-1)
    for k in ks
        r = _cellrange(g, k)
        isempty(r) && continue
        q = SVector(k % g.dims[1], (k ÷ g.dims[1]) % g.dims[2], k ÷ (g.dims[1] * g.dims[2]))
        lo = g.lo + Vec3f(q) .* g.h
        isfull(heap) && box_dist2(AABB(lo, lo .+ g.h), p) > top(heap) && continue
        @inbounds for j in r
            g.orig[j] == exclude && continue
            push!(heap, sum(abs2, g.pos[j] - p), g.orig[j])
        end
    end
    return heap
end

"Number of points within `r` of `p` (excluding original index `exclude`), stopping at `limit`. Positional arguments: keyword calls from threaded loops were not specialised and boxed every argument."
function count_within(g::GridNeighbors, p::Vec3f, r::Float32, exclude::Integer=0, limit::Integer=typemax(Int))
    c = _cell(g, p)
    s = ceil(Int, r / g.h)
    r2 = r * r; n = 0
    @inbounds for dz in -s:s, dy in -s:s, dx in -s:s
        q = c + SVector(dx, dy, dz)
        (all(q .>= 0) && all(q .< SVector(g.dims))) || continue
        for j in _cellrange(g, q[1] + g.dims[1] * (q[2] + g.dims[2] * q[3]))
            g.orig[j] == exclude && continue
            if sum(abs2, g.pos[j] - p) <= r2
                n += 1
                n >= limit && return n
            end
        end
    end
    return n
end
