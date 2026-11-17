# Bounding volume hierarchy over mesh triangles (binned SAH build, flat
# array layout, stackless-allocation traversal).

struct BVHNode
    box::AABB
    # interior: `a` = index of the left child (right child is a + 1), n = 0
    # leaf:     `a` = first entry in `tris`, n = triangle count
    a::Int32
    n::Int32
end
isleaf(n::BVHNode) = n.n > 0

"""
    BVH

Bounding volume hierarchy over one [`TriMesh`](@ref), for ray casting
(picking, measurements, line-of-sight checks) and spatial queries.
Build with [`build_bvh`](@ref).
"""
struct BVH
    nodes::Vector{BVHNode}
    tris::Vector{Int32}           # triangle ids, leaf-contiguous
    v0::Vector{Vec3f}             # triangle vertices in `tris` order (cache friendly)
    e1::Vector{Vec3f}
    e2::Vector{Vec3f}
    stack::Vector{Int32}          # traversal scratch, reused (not thread-safe)
end

const BVH_BINS = 12

"""
    build_bvh(mesh; leaf_size=4) -> BVH

Top-down build with a 12-bin surface-area heuristic.
"""
function build_bvh(m::TriMesh; leaf_size::Int=4)
    nt = ntriangles(m)
    boxes = Vector{AABB}(undef, nt)
    cents = Vector{Vec3f}(undef, nt)
    for t in 1:nt
        a, b, c = triangle(m, t)
        boxes[t] = union(union(AABB(a, a), b), c)
        cents[t] = (a + b + c) / 3
    end
    ids = collect(Int32, 1:nt)
    nodes = BVHNode[]
    sizehint!(nodes, 2 * max(nt ÷ leaf_size, 1))
    push!(nodes, BVHNode(AABB(), 0, 0))
    work = [(1, 1, nt)]                         # (node index, lo, hi) into ids
    while !isempty(work)
        ni, lo, hi = pop!(work)
        box = AABB(); cb = AABB()
        for i in lo:hi
            box = union(box, boxes[ids[i]]); cb = union(cb, cents[ids[i]])
        end
        cnt = hi - lo + 1
        if cnt <= leaf_size
            nodes[ni] = BVHNode(box, Int32(lo), Int32(cnt)); continue
        end
        split = _sah_split!(ids, lo, hi, boxes, cents, cb, box)
        if split === nothing           # degenerate (all centroids equal)
            nodes[ni] = BVHNode(box, Int32(lo), Int32(cnt)); continue
        end
        left = length(nodes) + 1
        push!(nodes, BVHNode(AABB(), 0, 0), BVHNode(AABB(), 0, 0))
        nodes[ni] = BVHNode(box, Int32(left), Int32(0))
        push!(work, (left, lo, split), (left + 1, split + 1, hi))
    end
    v0 = Vector{Vec3f}(undef, nt); e1 = similar(v0); e2 = similar(v0)
    for (k, t) in enumerate(ids)
        a, b, c = triangle(m, t)
        v0[k] = a; e1[k] = b - a; e2[k] = c - a
    end
    return BVH(nodes, ids, v0, e1, e2, Vector{Int32}(undef, 128))
end

# Partition ids[lo:hi] at the best binned-SAH plane; returns the last index of
# the left half, or nothing when splitting does not help.
function _sah_split!(ids, lo, hi, boxes, cents, cb, box)
    e = extent(cb)
    axis = argmax(e)
    e[axis] <= 0 && return nothing
    scale = BVH_BINS / e[axis]
    binbox = fill(AABB(), BVH_BINS); bincnt = zeros(Int, BVH_BINS)
    bin(t) = clamp(floor(Int, (cents[t][axis] - cb.lo[axis]) * scale) + 1, 1, BVH_BINS)
    for i in lo:hi
        t = ids[i]; b = bin(t)
        bincnt[b] += 1; binbox[b] = union(binbox[b], boxes[t])
    end
    # sweep: cost of splitting after bin k
    lA = zeros(Float32, BVH_BINS - 1); lN = zeros(Int, BVH_BINS - 1)
    acc = AABB(); n = 0
    for k in 1:BVH_BINS-1
        acc = union(acc, binbox[k]); n += bincnt[k]
        lA[k] = surface_area(acc); lN[k] = n
    end
    best, bestk = Inf32, 0
    acc = AABB(); n = 0
    for k in BVH_BINS-1:-1:1
        acc = union(acc, binbox[k+1]); n += bincnt[k+1]
        (lN[k] == 0 || n == 0) && continue
        c = lA[k] * lN[k] + surface_area(acc) * n
        c < best && ((best, bestk) = (c, k))
    end
    cnt = hi - lo + 1
    leafcost = surface_area(box) * cnt
    bestk == 0 && return nothing
    best >= leafcost && cnt <= 16 && return nothing   # small node: a leaf is cheaper
    # partition
    i, j = lo, hi
    while i <= j
        if bin(ids[i]) <= bestk
            i += 1
        else
            ids[i], ids[j] = ids[j], ids[i]; j -= 1
        end
    end
    mid = i - 1
    (mid < lo || mid >= hi) && return nothing
    return mid
end

"""
    raycast(bvh, ray; tmax=Inf32) -> (t, triangle)

Closest hit of `ray` (Möller–Trumbore), as distance along the ray and the
1-based triangle id; `(Inf32, 0)` on a miss. Allocation-free.
"""
function raycast(bvh::BVH, r::Ray; tmax::Real=Inf32)
    best = Float32(tmax); hit = Int32(0)
    st = bvh.stack; sp = 1; st[1] = 1
    nodes = bvh.nodes
    isempty(bvh.tris) && return (Inf32, 0)
    @inbounds while sp > 0
        node = nodes[st[sp]]; sp -= 1
        ray_box(r, node.box, best) == Inf32 && continue
        if isleaf(node)
            for k in node.a:node.a+node.n-1
                t = _moller_trumbore(r, bvh.v0[k], bvh.e1[k], bvh.e2[k])
                t < best && ((best, hit) = (t, bvh.tris[k]))
            end
        else
            # visit the nearer child first
            l, rr = node.a, node.a + Int32(1)
            tl = ray_box(r, nodes[l].box, best); tr = ray_box(r, nodes[rr].box, best)
            if sp + 2 > length(st)
                resize!(st, 2length(st))
            end
            if tl <= tr
                tr < Inf32 && (st[sp += 1] = rr)
                tl < Inf32 && (st[sp += 1] = l)
            else
                tl < Inf32 && (st[sp += 1] = l)
                tr < Inf32 && (st[sp += 1] = rr)
            end
        end
    end
    return hit == 0 ? (Inf32, 0) : (best, Int(hit))
end

@inline function _moller_trumbore(r::Ray, v0, e1, e2)
    p = cross(r.dir, e2)
    det = dot(e1, p)
    abs(det) < 1f-12 && return Inf32
    inv = 1f0 / det
    s = r.origin - v0
    u = dot(s, p) * inv
    (u < 0f0) | (u > 1f0) && return Inf32
    q = cross(s, e1)
    v = dot(r.dir, q) * inv
    (v < 0f0) | (u + v > 1f0) && return Inf32
    t = dot(e2, q) * inv
    return t > 1f-6 ? t : Inf32
end

depth(bvh::BVH) = _depth(bvh.nodes, 1)
_depth(nodes, i) = isleaf(nodes[i]) ? 1 : 1 + max(_depth(nodes, nodes[i].a), _depth(nodes, nodes[i].a + 1))
