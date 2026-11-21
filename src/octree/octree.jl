# Runtime side of out-of-core point clouds: hierarchy + memory-mapped data,
# and the per-frame choice of which nodes to show.

struct OctreeNode
    name::String
    depth::Int
    box::AABB
    offset::Int            # first point (0-based) in nodes.bin
    count::Int
    children::Vector{Int32}
end

"""
    OctreeCloud(dir; point_budget=3_000_000, min_node_pixels=80, point_size=2, color_mode)

A point cloud converted with [`build_octree`](@ref) and streamed from disk.
With `adaptive_size` (default) each node's points are drawn at a world size
matching that node's point spacing (never below `point_size` pixels), so
coarse regions read as continuous surfaces rather than sparse dots.

Streaming details:
only the nodes that matter for the current view are loaded (on a
background thread) and kept on the GPU, within `point_budget` points.
Nodes are refined while their projected size exceeds `min_node_pixels`.

The layer's coordinates are relative to `frame(cloud)` (an `OffsetFrame` at
the data centre); load other data with that frame to overlay it.
"""
mutable struct OctreeCloud <: Layer
    name::String
    dir::String
    nodes::Vector{OctreeNode}
    data::Vector{PointVertex}          # memory-mapped nodes.bin
    frame::OffsetFrame
    bounds::AABB
    npoints::Int
    point_budget::Int
    min_node_pixels::Float32
    point_size::Float32
    color_mode::Int32
    visible::Bool
    model::Mat4f
    rev::Int
    spacing_div::Int
    adaptive_size::Bool
    octants::Vector{NTuple{8, Int32}}  # child node per octant digit (0 = none)
end

function OctreeCloud(dir::AbstractString; name=basename(normpath(dir)), point_budget::Integer=3_000_000,
                     min_node_pixels::Real=80, point_size::Real=2.0, color_mode=nothing,
                     adaptive_size::Bool=true)
    idx = JSON3.read(read(joinpath(dir, "octree.json"), String))
    idx.version == OCTREE_VERSION || error("unsupported octree version $(idx.version)")
    v3(x) = Vec3f(x[1], x[2], x[3])
    byname = Dict{String, Int}(String(nd.name) => i for (i, nd) in enumerate(idx.nodes))
    nodes = [OctreeNode(String(nd.name), nd.depth, AABB(v3(nd.lo), v3(nd.hi)), nd.offset, nd.count,
                        Int32[byname[String(c)] for c in nd.children]) for nd in idx.nodes]
    total = sum(nd.count for nd in nodes)
    data = open(io -> Mmap.mmap(io, Vector{PointVertex}, total), joinpath(dir, "nodes.bin"))
    mode = something(color_mode, idx.has_rgb ? COLOR_RGB : COLOR_HEIGHT)
    root = findfirst(nd -> nd.name == "r", nodes)
    root == 1 || error("octree index must list the root first")
    OctreeCloud(String(name), String(dir), nodes, data, OffsetFrame(Vec3d(idx.origin...)),
                AABB(v3(idx.bounds.lo), v3(idx.bounds.hi)), idx.points, point_budget,
                Float32(min_node_pixels), Float32(point_size), Int32(mode), true, IDENTITY4, 0,
                Int(get(idx, :spacing_div, 128)), adaptive_size, _octant_table(nodes))
end

bounds(o::OctreeCloud) = o.bounds
Base.length(o::OctreeCloud) = o.npoints
frame(o::OctreeCloud) = o.frame
function _octant_table(nodes)
    map(nodes) do nd
        t = zeros(Int32, 8)
        for c in nd.children
            t[parse(Int, nodes[c].name[end])+1] = c
        end
        Tuple(t)
    end
end

"Typical distance between neighbouring points stored in node `i`."
node_spacing(o::OctreeCloud, i::Integer) = Float32(maximum(extent(o.nodes[i].box)) / o.spacing_div)

node_points(o::OctreeCloud, i::Integer) = view(o.data, o.nodes[i].offset+1:o.nodes[i].offset+o.nodes[i].count)

# --- a small max-heap of (priority, node) ---------------------------------------

struct NodeHeap
    pri::Vector{Float32}
    idx::Vector{Int32}
end
NodeHeap() = NodeHeap(Float32[], Int32[])
Base.isempty(h::NodeHeap) = isempty(h.pri)
Base.empty!(h::NodeHeap) = (empty!(h.pri); empty!(h.idx); h)
function Base.push!(h::NodeHeap, p::Float32, i::Int32)
    push!(h.pri, p); push!(h.idx, i)
    k = length(h.pri)
    while k > 1
        par = k >> 1
        h.pri[par] >= h.pri[k] && break
        h.pri[par], h.pri[k] = h.pri[k], h.pri[par]; h.idx[par], h.idx[k] = h.idx[k], h.idx[par]
        k = par
    end
    return h
end
function Base.pop!(h::NodeHeap)
    p, i = h.pri[1], h.idx[1]
    lp, li = pop!(h.pri), pop!(h.idx)
    if !isempty(h.pri)
        h.pri[1], h.idx[1] = lp, li
        k = 1; n = length(h.pri)
        while true
            l = 2k; r = l + 1; m = k
            l <= n && h.pri[l] > h.pri[m] && (m = l)
            r <= n && h.pri[r] > h.pri[m] && (m = r)
            m == k && break
            h.pri[m], h.pri[k] = h.pri[k], h.pri[m]; h.idx[m], h.idx[k] = h.idx[k], h.idx[m]
            k = m
        end
    end
    return p, i
end

"""
    select_nodes!(out, cloud, frustum, eye, proj_scale, isloaded; heap=NodeHeap()) -> (out, wanted)

Choose the nodes to draw, largest on screen first, within the point budget.
Only loaded nodes are drawn and only their children are refined, so the view
always shows complete coarse levels while finer ones stream in. Returns the
drawable nodes in `out` and the unloaded nodes that should be fetched next
(highest priority first) in `wanted`.
"""
function select_nodes!(out::Vector{Int32}, wanted::Vector{Int32}, o::OctreeCloud, f::Frustum,
                       eye::Vec3f, proj_scale::Float32, isloaded; heap::NodeHeap=NodeHeap())
    empty!(out); empty!(wanted); empty!(heap)
    budget = o.point_budget
    priority(nd) = (r = radius(nd.box); d = norm(center(nd.box) - eye);
                    d <= r ? Inf32 : Float32(proj_scale * r / (d - r)))   # projected radius, px
    intersects(f, o.nodes[1].box) && push!(heap, priority(o.nodes[1]), Int32(1))
    while !isempty(heap)
        p, i = pop!(heap)
        nd = o.nodes[i]
        if !isloaded(i)
            push!(wanted, i)
            continue
        end
        nd.count > budget && continue
        budget -= nd.count
        push!(out, i)
        p < o.min_node_pixels && continue          # small enough on screen: stop refining
        for c in nd.children
            ch = o.nodes[c]
            intersects(f, ch.box) && push!(heap, priority(ch), c)
        end
    end
    return out, wanted
end
