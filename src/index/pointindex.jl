# Spatial index for point clouds.
#
# The cloud is reordered in place so that every k-d tree leaf ("chunk") is a
# contiguous range of the vertex buffer. Culling then reduces to choosing
# ranges, which go to `glMultiDrawArrays` with no index buffer and no copy.
# Points inside each chunk are shuffled, so any prefix of a chunk is a
# uniform subsample: drawing a shorter prefix for distant chunks is a
# continuous level of detail with no extra memory.

"A chunk of the reordered cloud: `count` points starting at 0-based `first`."
struct PointChunk
    first::Int32
    count::Int32
    box::AABB
end

"""
    DrawRange(first, count)

One `glMultiDrawArrays` range. Stored in a `StructVector`, so the `first`
and `count` fields are two contiguous `Int32` arrays: exactly the two
arrays OpenGL wants, passed by pointer with no packing.
"""
struct DrawRange
    first::Int32
    count::Int32
end

"""
    VisibleRanges(capacity)

Pre-allocated output of [`cull!`](@ref), reused every frame (no allocation
in the render loop).
"""
mutable struct VisibleRanges
    ranges::StructVector{DrawRange, @NamedTuple{first::Vector{Int32}, count::Vector{Int32}}, Int}
    n::Int
    npoints::Int
end
VisibleRanges(capacity::Integer) =
    VisibleRanges(StructVector{DrawRange}((first=zeros(Int32, capacity), count=zeros(Int32, capacity))), 0, 0)
Base.length(v::VisibleRanges) = v.n

"""
    PointIndex

Chunked k-d tree over a [`PointCloud`](@ref) plus a `NearestNeighbors.KDTree`
for k-NN / radius queries. Build with [`build_point_index`](@ref).
"""
struct PointIndex{C <: StructVector{PointChunk}, T <: KDTree}
    chunks::C
    tree::T
    visible::VisibleRanges
    bounds::AABB
end

"""
    build_point_index(pc; chunk_size=8192, seed=1) -> PointIndex

Reorders `pc.vertices` (and `pc.gps_time`) in place into spatially coherent
chunks of at most `chunk_size` points (median splits on the longest axis),
shuffles points within each chunk, and builds a k-d tree for queries.
"""
function build_point_index(pc::PointCloud; chunk_size::Int=8192, seed::Int=1)
    n = length(pc)
    perm = collect(Int32, 1:n)
    pos = [v.position for v in pc.vertices]
    chunks = PointChunk[]
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
            push!(chunks, PointChunk(Int32(lo - 1), Int32(hi - lo + 1), box))
            continue
        end
        axis = argmax(extent(box))
        mid = (lo + hi) >>> 1
        _select!(perm, lo, hi, mid, pos, axis)
        push!(work, (mid + 1, hi), (lo, mid))
    end
    sort!(chunks; by = c -> c.first)
    pc.vertices .= pc.vertices[perm]
    isempty(pc.gps_time) || (pc.gps_time .= pc.gps_time[perm])
    tree = KDTree([v.position for v in pc.vertices]; leafsize=16, reorder=false)
    sc = StructVector(chunks)
    return PointIndex(sc, tree, VisibleRanges(length(chunks)), reduce(union, sc.box; init=AABB()))
end

nchunks(ix::PointIndex) = length(ix.chunks)

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

"""
    nearest_points(index, p, k) -> (indices, distances)

`k` nearest points to `p` (1-based indices into the reordered cloud).
"""
nearest_points(ix::PointIndex, p, k::Integer) = knn(ix.tree, Vec3f(p), k, true)

"""
    points_in_radius(index, p, r) -> Vector{Int}
"""
points_in_radius(ix::PointIndex, p, r::Real) = inrange(ix.tree, Vec3f(p), r)
