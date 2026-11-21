# GPU residency for OctreeCloud: background loading, per-node buffers, LRU.

mutable struct GPUOctree <: GPULayer
    vaos::Dict{Int32, GLuint}
    vbos::Dict{Int32, GLBuffer}
    last_used::Dict{Int32, Int}
    resident::Int                              # points on the GPU
    requested::Set{Int32}
    requests::Channel{Int32}
    loaded::Channel{Tuple{Int32, Vector{PointVertex}}}
    loader::Task
    frame::Int
    draw::Vector{Int32}
    wanted::Vector{Int32}
    heap::NodeHeap
    drawn::BitVector                           # membership of `draw`, for adaptive sizes
end

"Nodes uploaded per frame at most (keeps frame times smooth while streaming)."
const OCTREE_UPLOADS_PER_FRAME = 8

function upload(o::OctreeCloud)
    req = Channel{Int32}(Inf)
    done = Channel{Tuple{Int32, Vector{PointVertex}}}(Inf)
    # Page node data in from the memory map off the render thread; GL calls
    # stay on the render thread.
    loader = Threads.@spawn begin
        for i in req
            put!(done, (i, collect(node_points(o, i))))
        end
    end
    GPUOctree(Dict{Int32, GLuint}(), Dict{Int32, GLBuffer}(), Dict{Int32, Int}(), 0, Set{Int32}(),
              req, done, loader, 0, Int32[], Int32[], NodeHeap(), falses(length(o.nodes)))
end

function release!(g::GPUOctree)
    close(g.requests)
    for (i, vao) in g.vaos
        glDeleteVertexArrays(1, Ref(vao)); release!(g.vbos[i])
    end
    empty!(g.vaos); empty!(g.vbos)
end

function _upload_node!(g::GPUOctree, i::Int32, pts::Vector{PointVertex})
    vao = new_vao()
    g.vbos[i] = GLBuffer(GL_ARRAY_BUFFER, pts)
    point_layout!()
    glBindVertexArray(0)
    g.vaos[i] = vao
    g.resident += length(pts)
end

function _evict!(g::GPUOctree, o::OctreeCloud)
    cap = 2 * o.point_budget
    g.resident <= cap && return
    inuse = Set(g.draw)
    for (i, _) in sort(collect(g.last_used); by = last)          # least recently used first
        g.resident <= cap && break
        (i in inuse || i == 1) && continue
        glDeleteVertexArrays(1, Ref(g.vaos[i])); release!(g.vbos[i])
        g.resident -= o.nodes[i].count
        delete!(g.vaos, i); delete!(g.vbos, i); delete!(g.last_used, i); delete!(g.requested, i)
    end
end

function draw!(g::GPUOctree, o::OctreeCloud, p::GLProgram, fc::FrameContext, s::RenderSettings)
    g.frame += 1
    # 1. take what the loader finished (bounded per frame)
    k = 0
    while isready(g.loaded) && k < OCTREE_UPLOADS_PER_FRAME
        i, pts = take!(g.loaded)
        haskey(g.vaos, i) || _upload_node!(g, i, pts)
        k += 1
    end
    # 2. choose nodes in the layer's frame
    fr, eye = local_view(fc, o.model)
    select_nodes!(g.draw, g.wanted, o, fr, eye, fc.proj_scale, i -> haskey(g.vaos, i); heap=g.heap)
    # 3. request missing nodes, most important first
    for i in g.wanted
        i in g.requested && continue
        push!(g.requested, i); put!(g.requests, i)
    end
    # 4. draw
    npts = 0
    if !isempty(g.draw)
        _point_uniforms!(p, fc, s, o.model, o.point_size, o.color_mode, _zrange(o.model, o.bounds))
        setuniform(p, :u_seq, 0); setuniform(p, :u_highlight, -1)
        if o.adaptive_size
            fill!(g.drawn, false); g.drawn[g.draw] .= true
            setuniform(p, :u_size_mode, 1)
            setuniform(p, :u_min_px, o.point_size)
            setuniform(p, :u_octree, 1)
        end
        for i in g.draw
            o.adaptive_size && _octant_sizes!(p, o, g, i)
            glBindVertexArray(g.vaos[i])
            glDrawArrays(GL_POINTS, 0, o.nodes[i].count)
            g.last_used[i] = g.frame
            npts += o.nodes[i].count
        end
    end
    setuniform(p, :u_octree, 0)
    _evict!(g, o)
    return npts
end

# Per-octant world point sizes for node `i`: a child region with a drawn child
# is as dense as that child's finest drawn level; otherwise only this node's
# points cover it.
const ADAPTIVE_FACTOR = 0.9f0
function _octant_sizes!(p, o::OctreeCloud, g::GPUOctree, i::Integer)
    own = node_spacing(o, i)
    t = o.octants[i]
    sizes = ntuple(k -> ADAPTIVE_FACTOR * (t[k] != 0 && g.drawn[t[k]] ? _effective_spacing(o, g, t[k]) : own), 8)
    box = o.nodes[i].box
    sp = split_axes(extent(box))
    setuniform(p, :u_node_mid, center(box))
    setuniform(p, :u_node_split, Vec3f(sp[1], sp[2], sp[3]))
    setuniform(p, :u_octant_size, sizes)
end

"Spacing of the finest drawn level under node `i` (its own if no child is drawn)."
function _effective_spacing(o::OctreeCloud, g::GPUOctree, i::Integer)
    best = Inf32
    for c in o.nodes[i].children
        g.drawn[c] && (best = min(best, _effective_spacing(o, g, c)))
    end
    return isfinite(best) ? best : node_spacing(o, i)
end

"Block until every requested node has arrived and upload it (offscreen rendering)."
function drain!(g::GPUOctree)
    while any(i -> !haskey(g.vaos, i), g.requested)
        i, pts = take!(g.loaded)
        haskey(g.vaos, i) || _upload_node!(g, i, pts)
    end
end

"Is the octree still fetching nodes for the current view?"
streaming(g::GPUOctree) = !isempty(g.wanted) || isready(g.loaded)

local_box(g::GPUOctree, l) = l.bounds

# Picking: test the nodes drawn in the last frame (what the user sees), points
# read from the memory map. Index is global into nodes.bin (1-based).
function pick_layer(o::OctreeCloud, g::GPUOctree, r::Ray; tmax=Inf32, angle=0.003)
    tanα = Float32(tan(angle))
    best = Float32(tmax); besti = 0
    for i in g.draw
        nd = o.nodes[i]
        tfar = norm(center(nd.box) - r.origin) + radius(nd.box)
        ray_box(r, expand(nd.box, tanα * tfar), best) == Inf32 && continue
        t, k = pick_points_brute(node_points(o, i), r, tanα, best)
        k != 0 && ((best, besti) = (t, nd.offset + k))
    end
    return besti == 0 ? (Inf32, 0, r.origin) : (best, besti, o.data[besti].position)
end
