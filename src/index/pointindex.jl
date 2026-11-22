# Spatial index for point clouds.
#
# The cloud is reordered in place so that every k-d tree leaf ("chunk") is a
# contiguous range of the vertex buffer. Culling then reduces to choosing
# ranges, which go to `glMultiDrawArrays` with no index buffer and no copy.
# Points inside each chunk are shuffled, so any prefix of a chunk is a
# uniform subsample: drawing a shorter prefix for distant chunks is a
# continuous level of detail with no extra memory.
#
# The same chunks answer nearest-neighbour and radius queries (chunks are
# visited nearest box first), so the index costs ~40 bytes per *chunk*, not
# per point.

"Positions of a vertex vector, without copying them out."
struct Positions{V <: AbstractVector{PointVertex}} <: AbstractVector{Vec3f}
    v::V
end
Base.size(p::Positions) = size(p.v)
Base.@propagate_inbounds Base.getindex(p::Positions, i::Integer) = p.v[i].position

"Chunks of the reordered cloud: `count[i]` points from 0-based `first[i]`, inside `box[i]`."
struct ChunkTable
    first::Vector{Int32}
    count::Vector{Int32}
    box::Vector{AABB}
end
Base.length(c::ChunkTable) = length(c.first)
Base.eachindex(c::ChunkTable) = eachindex(c.first)

"""
    DrawRange(first, count)

One `glMultiDrawArrays` range (documentation type: [`VisibleRanges`](@ref)
stores ranges as two parallel `Int32` arrays, exactly what OpenGL reads).
"""
struct DrawRange
    first::Int32
    count::Int32
end

"Two parallel `Int32` arrays (`first`, `count`) passed to `glMultiDrawArrays` by pointer."
struct RangeArrays
    first::Vector{Int32}
    count::Vector{Int32}
end

"""
    VisibleRanges(capacity)

Pre-allocated output of [`cull!`](@ref), reused every frame (no allocation
in the render loop).
"""
mutable struct VisibleRanges
    ranges::RangeArrays
    n::Int
    npoints::Int
end
VisibleRanges(capacity::Integer) = VisibleRanges(RangeArrays(zeros(Int32, capacity), zeros(Int32, capacity)), 0, 0)
Base.length(v::VisibleRanges) = v.n

"""
    PointIndex

Chunked k-d tree over a [`PointCloud`](@ref), used for culling, level of
detail, picking and neighbour queries. Build with [`build_point_index`](@ref).
"""
struct PointIndex
    chunks::ChunkTable
    vertices::Vector{PointVertex}      # the cloud's own (reordered) array, not a copy
    visible::VisibleRanges
    bounds::AABB
end

"""
    build_point_index(pc; chunk_size=8192, seed=1) -> PointIndex

Reorders `pc.vertices` (and `pc.gps_time`) in place into spatially coherent
chunks of at most `chunk_size` points (median splits on the longest axis)
and shuffles points within each chunk. Extra memory while building is one
`Int32` permutation (4 bytes per point); what stays is one entry per chunk.
"""
function build_point_index(pc::PointCloud; chunk_size::Int=8192, seed::Int=1)
    chunk_size >= 1 || throw(ArgumentError("chunk_size must be at least 1"))
    n = length(pc)
    perm = collect(Int32, 1:n)
    pos = Positions(pc.vertices)
    firsts = Int32[]; counts = Int32[]; boxes = AABB[]
    work = [(1, n)]
    rng = Xoshiro(seed)
    while !isempty(work)
        lo, hi = pop!(work)
        hi < lo && continue
        box = AABB()
        @inbounds for i in lo:hi
            box = union(box, pos[perm[i]])
        end
        if hi - lo + 1 <= chunk_size
            shuffle!(rng, view(perm, lo:hi))
            push!(firsts, Int32(lo - 1)); push!(counts, Int32(hi - lo + 1)); push!(boxes, box)
            continue
        end
        axis = argmax(extent(box))
        mid = (lo + hi) >>> 1
        _select!(perm, lo, hi, mid, pos, axis)
        push!(work, (mid + 1, hi), (lo, mid))
    end
    order = sortperm(firsts)
    chunks = ChunkTable(firsts[order], counts[order], boxes[order])
    permute_inplace!(pc.vertices, perm)
    isempty(pc.gps_time) || permute_inplace!(pc.gps_time, perm)
    return PointIndex(chunks, pc.vertices, VisibleRanges(length(chunks)), reduce(union, chunks.box; init=AABB()))
end

nchunks(ix::PointIndex) = length(ix.chunks)

"""
    permute_inplace!(v, perm) -> v

`v .= v[perm]` without the copy: follows the permutation's cycles, using one
bit per element to mark finished positions. (`Base.permute!` allocates a
full copy of `v` in current Julia.)
"""
function permute_inplace!(v::AbstractVector, perm::AbstractVector{<:Integer})
    done = falses(length(v))
    @inbounds for start in eachindex(v)
        done[start] && continue
        tmp = v[start]
        i = start
        while true
            j = perm[i]
            done[i] = true
            if j == start
                v[i] = tmp
                break
            end
            v[i] = v[j]
            i = j
        end
    end
    return v
end

# Quickselect (Hoare partition): afterwards perm[lo:k-1] ≤ perm[k] ≤ perm[k+1:hi]
# by coordinate `axis`, which is what makes each half spatially coherent.
function _select!(perm, lo, hi, k, pos, axis)
    @inbounds while lo < hi
        a, b, c = pos[perm[lo]][axis], pos[perm[(lo + hi) >>> 1]][axis], pos[perm[hi]][axis]
        pivot = max(min(a, b), min(max(a, b), c))          # median of three
        i, j = lo, hi
        while i <= j
            while pos[perm[i]][axis] < pivot; i += 1; end
            while pos[perm[j]][axis] > pivot; j -= 1; end
            if i <= j
                perm[i], perm[j] = perm[j], perm[i]
                i += 1; j -= 1
            end
        end
        if k <= j
            hi = j
        elseif k >= i
            lo = i
        else
            break
        end
    end
    return perm
end

"Squared distance from `p` to the box (0 inside)."
@inline function box_dist2(b::AABB, p::Vec3f)
    d = max.(b.lo - p, 0f0, p - b.hi)
    return dot(d, d)
end

"""
    nearest_points(index, p, k) -> (indices, distances)

The `k` nearest points to `p`, nearest first (1-based indices into the
reordered cloud; Euclidean distances). Chunks are scanned in order of box
distance and the search stops once no remaining chunk can contain a closer
point.
"""
function nearest_points(ix::PointIndex, p, k::Integer)
    q = Vec3f(p)
    ch = ix.chunks
    d2 = [box_dist2(ch.box[c], q) for c in eachindex(ch)]
    heap = KHeap(k)
    for c in sortperm(d2)
        isfull(heap) && d2[c] > top(heap) && break
        @inbounds for i in ch.first[c]+1:ch.first[c]+ch.count[c]
            push!(heap, sum(abs2, ix.vertices[i].position - q), Int32(i))
        end
    end
    idx, dist2 = sorted(heap)
    return Int.(idx), sqrt.(dist2)
end

"""
    points_in_radius(index, p, r) -> Vector{Int}

Indices of all points within distance `r` of `p`.
"""
function points_in_radius(ix::PointIndex, p, r::Real)
    q = Vec3f(p); r2 = Float32(r)^2
    ch = ix.chunks
    out = Int[]
    for c in eachindex(ch)
        box_dist2(ch.box[c], q) > r2 && continue
        @inbounds for i in ch.first[c]+1:ch.first[c]+ch.count[c]
            sum(abs2, ix.vertices[i].position - q) <= r2 && push!(out, i)
        end
    end
    return out
end
