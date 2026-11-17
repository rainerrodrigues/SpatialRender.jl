# GPU buffers and vertex layouts.
#
# Vertex arrays are uploaded straight from the Julia array's memory: the
# element types in `core/types.jl` are isbits structs laid out exactly as the
# attribute pointers below describe, so `glBufferData` reads the Vector
# in place (the only copy is the driver's transfer into GPU memory).

mutable struct GLBuffer
    id::GLuint
    target::GLenum
    bytes::Int
end

function GLBuffer(target::GLenum, data::Vector{T}; usage::GLenum=GL_STATIC_DRAW) where {T}
    isbitstype(T) || throw(ArgumentError("GPU buffers need isbits elements, got $T"))
    id = Ref{GLuint}(0)
    glGenBuffers(1, id)
    glBindBuffer(target, id[])
    GC.@preserve data glBufferData(target, sizeof(data), pointer(data), usage)
    return GLBuffer(id[], target, sizeof(data))
end

"Replace a buffer's contents, reallocating only when it must grow."
function update!(b::GLBuffer, data::Vector{T}) where {T}
    glBindBuffer(b.target, b.id)
    GC.@preserve data if sizeof(data) <= b.bytes
        glBufferSubData(b.target, 0, sizeof(data), pointer(data))
    else
        glBufferData(b.target, sizeof(data), pointer(data), GL_DYNAMIC_DRAW)
        b.bytes = sizeof(data)
    end
    return b
end

release!(b::GLBuffer) = (b.id != 0 && glDeleteBuffers(1, Ref(b.id)); b.id = 0; nothing)

function new_vao()
    id = Ref{GLuint}(0)
    glGenVertexArrays(1, id)
    glBindVertexArray(id[])
    return id[]
end

_off(T, f::Symbol) = Ptr{Cvoid}(fieldoffset(T, Base.fieldindex(T, f)))

function _float_attr(loc, n, T, field; type=GL_FLOAT, normalized=GL_FALSE, divisor=0)
    glEnableVertexAttribArray(loc)
    glVertexAttribPointer(loc, n, type, normalized, sizeof(T), _off(T, field))
    divisor > 0 && glVertexAttribDivisor(loc, divisor)
end

"Attribute layout of [`PointVertex`](@ref) (locations 0–3)."
function point_layout!()
    T = PointVertex
    _float_attr(0, 3, T, :position)
    _float_attr(1, 1, T, :intensity)
    _float_attr(2, 4, T, :rgba; type=GL_UNSIGNED_BYTE, normalized=GL_TRUE)
    glEnableVertexAttribArray(3)
    glVertexAttribIPointer(3, 1, GL_UNSIGNED_BYTE, sizeof(T), _off(T, :classification))
end

"Attribute layout of [`MeshVertex`](@ref) (locations 0–2)."
function mesh_layout!()
    T = MeshVertex
    _float_attr(0, 3, T, :position)
    _float_attr(1, 3, T, :normal)
    _float_attr(2, 2, T, :uv)
end

"Attribute layout of [`LineVertex`](@ref) as per-instance data (locations 0–2)."
function line_instance_layout!()
    T = LineVertex
    _float_attr(0, 3, T, :a; divisor=1)
    _float_attr(1, 3, T, :b; divisor=1)
    _float_attr(2, 4, T, :color; divisor=1)
end

# ---------------------------------------------------------------------------
# GPU-side objects per layer

abstract type GPULayer end

mutable struct GPUPoints <: GPULayer
    vao::GLuint
    vbo::GLBuffer
    index::PointIndex
    zrange::Vec2f
end

mutable struct GPUMesh <: GPULayer
    vao::GLuint
    vbo::GLBuffer
    ibo::GLBuffer
    count::Int
    bvh::BVH
    box::AABB
end

mutable struct GPULines <: GPULayer
    vao::GLuint
    instances::GLBuffer
    count::Int
    box::AABB
end

const QUAD_CORNERS = Vec2f[Vec2f(0, -1), Vec2f(0, 1), Vec2f(1, -1), Vec2f(1, 1)]

function upload(pc::PointCloud; chunk_size=8192)
    ix = build_point_index(pc; chunk_size)        # reorders pc in place
    vao = new_vao()
    vbo = GLBuffer(GL_ARRAY_BUFFER, pc.vertices)
    point_layout!()
    glBindVertexArray(0)
    b = ix.bounds
    return GPUPoints(vao, vbo, ix, isempty(b) ? Vec2f(0, 1) : Vec2f(b.lo[3], b.hi[3]))
end

function upload(m::TriMesh)
    vao = new_vao()
    vbo = GLBuffer(GL_ARRAY_BUFFER, m.vertices)
    mesh_layout!()
    ibo = GLBuffer(GL_ELEMENT_ARRAY_BUFFER, m.indices)   # recorded in the VAO
    glBindVertexArray(0)
    return GPUMesh(vao, vbo, ibo, length(m.indices), build_bvh(m), bounds(m))
end

function upload(l::LineLayer, corners::GLBuffer)
    segs = segments(l)
    vao = new_vao()
    inst = GLBuffer(GL_ARRAY_BUFFER, segs)
    line_instance_layout!()
    glBindBuffer(GL_ARRAY_BUFFER, corners.id)
    glEnableVertexAttribArray(3)
    glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2f), C_NULL)
    glBindVertexArray(0)
    return GPULines(vao, inst, length(segs), bounds(l))
end

function release!(g::GPULayer)
    glDeleteVertexArrays(1, Ref(g.vao))
    for f in fieldnames(typeof(g))
        x = getfield(g, f)
        x isa GLBuffer && release!(x)
    end
end
