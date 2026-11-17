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
    glEnableVertexAttribArray(4)
    glVertexAttribIPointer(4, 1, GL_UNSIGNED_BYTE, sizeof(T), _off(T, :flags))
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

mutable struct GPUStream <: GPULayer
    vao::GLuint
    vbo::GLBuffer
end

"One label stroke: a segment in pixels relative to a 3-D anchor; `width > 0` overrides the stroke width (backing plates)."
struct LabelSegment
    anchor::Vec3f
    a::Vec2f
    b::Vec2f
    color::Vec4f
    width::Float32
end
LabelSegment(anchor, a, b, color) = LabelSegment(anchor, a, b, color, 0f0)

const PLATE_COLOR = Vec4f(0.05, 0.06, 0.08, 0.72)

mutable struct GPUAnnotations <: GPULayer
    lines::Any                     # GPULines (declared below)
    vao::GLuint
    instances::GLBuffer
    count::Int
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

function upload(sp::StreamingPointCloud)
    vao = new_vao()
    id = Ref{GLuint}(0)
    glGenBuffers(1, id)
    glBindBuffer(GL_ARRAY_BUFFER, id[])
    bytes = sizeof(PointVertex) * capacity(sp)
    glBufferData(GL_ARRAY_BUFFER, bytes, C_NULL, GL_STREAM_DRAW)   # allocate once
    point_layout!()
    glBindVertexArray(0)
    g = GPUStream(vao, GLBuffer(id[], GL_ARRAY_BUFFER, bytes))
    sp.dirty = [1:length(sp)]                                       # first upload: everything live
    return g
end

"Copy the stream's dirty ranges into its GPU buffer (caller holds UPDATE_LOCK)."
function sync_stream!(g::GPUStream, sp::StreamingPointCloud)
    isempty(sp.dirty) && return
    glBindBuffer(GL_ARRAY_BUFFER, g.vbo.id)
    T = sizeof(PointVertex)
    buf = sp.buffer
    GC.@preserve buf for r in sp.dirty
        isempty(r) && continue
        glBufferSubData(GL_ARRAY_BUFFER, (first(r) - 1) * T, length(r) * T, pointer(buf, first(r)))
    end
    empty!(sp.dirty)
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

"Lay out every label's text and marker as pixel-space segments."
function label_segments(a::Annotations)
    out = LabelSegment[]
    for l in a.labels
        if l.marker
            push!(out, LabelSegment(l.position, Vec2f(-4, -4), Vec2f(4, 4), l.color),
                       LabelSegment(l.position, Vec2f(-4, 4), Vec2f(4, -4), l.color))
        end
        isempty(l.text) && continue
        sc = l.size / 6f0                                  # cap height → glyph units
        nlines = count(==('\n'), l.text) + 1
        origin = Vec2f(8, 8 + (nlines - 1) * GLYPH_LINE * sc)   # text block above-right
        # translucent plate behind the text: one thick horizontal segment
        ext = text_extent(l.text) * sc
        top = origin[2] + 6sc; bottom = origin[2] - (nlines - 1) * GLYPH_LINE * sc
        mid = (top + bottom) / 2
        push!(out, LabelSegment(l.position, Vec2f(origin[1] - 2, mid), Vec2f(origin[1] + ext[1] - 2sc, mid),
                                PLATE_COLOR, top - bottom + 8))
        for (p, q) in text_segments(l.text)
            push!(out, LabelSegment(l.position, origin + sc * p, origin + sc * q, l.color))
        end
    end
    return out
end

function upload(a::Annotations, corners::GLBuffer)
    lines = upload(LineLayer(a.lines; width=a.width), corners)
    segs = label_segments(a)
    vao = new_vao()
    inst = GLBuffer(GL_ARRAY_BUFFER, isempty(segs) ? [LabelSegment(Vec3f(0, 0, 0), Vec2f(0, 0), Vec2f(0, 0), Vec4f(0, 0, 0, 0))] : segs)
    T = LabelSegment
    _float_attr(0, 3, T, :anchor; divisor=1)
    _float_attr(1, 2, T, :a; divisor=1)
    _float_attr(2, 2, T, :b; divisor=1)
    _float_attr(3, 4, T, :color; divisor=1)
    _float_attr(5, 1, T, :width; divisor=1)
    glBindBuffer(GL_ARRAY_BUFFER, corners.id)
    glEnableVertexAttribArray(4)
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2f), C_NULL)
    glBindVertexArray(0)
    return GPUAnnotations(lines, vao, inst, length(segs))
end

function release!(g::GPUAnnotations)
    release!(g.lines)
    glDeleteVertexArrays(1, Ref(g.vao))
    release!(g.instances)
end

function release!(g::GPULayer)
    glDeleteVertexArrays(1, Ref(g.vao))
    for f in fieldnames(typeof(g))
        x = getfield(g, f)
        x isa GLBuffer && release!(x)
    end
end
