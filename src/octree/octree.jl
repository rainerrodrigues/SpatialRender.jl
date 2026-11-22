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
    point_budget > 0 || throw(ArgumentError("point_budget must be positive"))
    min_node_pixels > 0 || throw(ArgumentError("min_node_pixels must be positive"))
    index = joinpath(dir, "octree.json")
    isfile(index) || throw(ArgumentError("$dir is not an octree directory (no octree.json); create one with build_octree"))
    idx = try
        JSON3.read(read(index, String))
    catch e
        throw(ArgumentError("$index is not valid JSON: " * first(split(sprint(showerror, e), '\n'))))
    end
    bad(msg) = throw(ArgumentError("$index: $msg (rebuild the directory with build_octree)"))
    idx isa JSON3.Object || bad("expected a JSON object")
    version = get(idx, :version, nothing)
    version == OCTREE_VERSION || bad("octree format $(repr(version)); this version of SpatialRender reads format $OCTREE_VERSION")
    num(o, k) = (x = get(o, k, nothing); x isa Real ? x : bad("field `$k` missing or not a number"))
    vec3(o, k) = (x = get(o, k, nothing);
                  x isa AbstractVector && length(x) == 3 && all(v -> v isa Real, x) ? Vec3f(x[1], x[2], x[3]) :
                  bad("field `$k` must be a list of 3 numbers"))
    obj(o, k) = (x = get(o, k, nothing); x isa JSON3.Object ? x : bad("field `$k` missing or not an object"))
    origin = let x = get(idx, :origin, nothing)          # Float64: survey coordinates need it
        x isa AbstractVector && length(x) == 3 && all(v -> v isa Real, x) ? Vec3d(x[1], x[2], x[3]) :
        bad("field `origin` must be a list of 3 numbers")
    end
    rawnodes = get(idx, :nodes, nothing)
    rawnodes isa AbstractVector && !isempty(rawnodes) || bad("no `nodes` list")
    byname = Dict{String, Int}()
    for (i, nd) in enumerate(rawnodes)
        nd isa JSON3.Object && get(nd, :name, nothing) isa AbstractString || bad("node $i has no name")
        byname[String(nd.name)] = i
    end
    nodes = map(enumerate(rawnodes)) do (i, nd)
        kids = get(nd, :children, Any[])
        kids isa AbstractVector || bad("node $(nd.name): `children` must be a list")
        OctreeNode(String(nd.name), Int(num(nd, :depth)), AABB(vec3(nd, :lo), vec3(nd, :hi)),
                   Int(num(nd, :offset)), Int(num(nd, :count)),
                   Int32[get(byname, String(c), 0) == 0 ? bad("node $(nd.name): unknown child $c") : byname[String(c)]
                         for c in kids])
    end
    nodes[1].name == "r" || bad("the root node must be listed first")
    total = sum(nd.count for nd in nodes)
    all(nd -> 0 <= nd.offset && nd.offset + nd.count <= total, nodes) || bad("a node's points lie outside nodes.bin")
    bin = joinpath(dir, "nodes.bin")
    isfile(bin) || bad("nodes.bin is missing")
    filesize(bin) == total * sizeof(PointVertex) ||
        bad("nodes.bin holds $(filesize(bin)) bytes, expected $(total * sizeof(PointVertex)) (truncated or from another version?)")
    data = open(io -> Mmap.mmap(io, Vector{PointVertex}, total), bin)
    has_rgb = get(idx, :has_rgb, false) === true
    spacing_div = let x = get(idx, :spacing_div, 128)     # older indexes lack it: 128 was the default
        x isa Integer && x > 0 ? Int(x) : bad("field `spacing_div` must be a positive integer")
    end
    mode = something(color_mode, has_rgb ? COLOR_RGB : COLOR_HEIGHT)
    bnds = obj(idx, :bounds)
    OctreeCloud(String(name), String(dir), nodes, data, OffsetFrame(origin),
                AABB(vec3(bnds, :lo), vec3(bnds, :hi)), Int(num(idx, :points)), Int(point_budget),
                Float32(min_node_pixels), Float32(point_size), Int32(mode), true, IDENTITY4, 0,
                spacing_div, adaptive_size, _octant_table(nodes))
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

layer_summary(o::OctreeCloud) = "$(o.npoints) points in $(length(o.nodes)) nodes, out-of-core"
