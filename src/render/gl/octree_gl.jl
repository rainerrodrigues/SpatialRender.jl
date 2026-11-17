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
              req, done, loader, 0, Int32[], Int32[], NodeHeap())
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
        for i in g.draw
            glBindVertexArray(g.vaos[i])
            glDrawArrays(GL_POINTS, 0, o.nodes[i].count)
            g.last_used[i] = g.frame
            npts += o.nodes[i].count
        end
    end
    _evict!(g, o)
    return npts
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
