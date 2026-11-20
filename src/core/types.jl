# Geometry primitives and the GPU-ready vertex layouts.
#
# Every vertex type is an `isbits` struct whose memory layout is exactly the
# interleaved layout the shaders read, so a `Vector{PointVertex}` (etc.) is
# uploaded to the GPU straight from its own memory: no repacking, no copy.

const Vec2f = SVector{2, Float32}
const Vec3f = SVector{3, Float32}
const Vec4f = SVector{4, Float32}
const Vec3d = SVector{3, Float64}
const Mat3f = SMatrix{3, 3, Float32, 9}
const Mat4f = SMatrix{4, 4, Float32, 16}
const RGBA8 = NTuple{4, UInt8}
const IDENTITY4 = Mat4f(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)

"""
    AABB(lo, hi)

Axis-aligned bounding box. `AABB()` is the empty box (`lo = +Inf`, `hi = -Inf`),
the identity of `union`.
"""
struct AABB
    lo::Vec3f
    hi::Vec3f
end
AABB() = AABB(Vec3f(Inf32, Inf32, Inf32), Vec3f(-Inf32, -Inf32, -Inf32))
Base.union(a::AABB, b::AABB) = AABB(min.(a.lo, b.lo), max.(a.hi, b.hi))
Base.union(a::AABB, p::Vec3f) = AABB(min.(a.lo, p), max.(a.hi, p))
Base.isempty(a::AABB) = any(a.lo .> a.hi)
center(a::AABB) = (a.lo + a.hi) / 2
extent(a::AABB) = a.hi - a.lo
radius(a::AABB) = isempty(a) ? 0.0f0 : norm(extent(a)) / 2
contains_point(a::AABB, p) = all(a.lo .<= p) && all(p .<= a.hi)
surface_area(a::AABB) = (e = max.(extent(a), 0f0); 2f0 * (e[1]*e[2] + e[2]*e[3] + e[3]*e[1]))

function bounds_of(ps)
    b = AABB()
    for p in ps
        b = union(b, p)
    end
    return b
end

# ---------------------------------------------------------------------------
# Vertex layouts (std430-compatible, tightly interleaved)

"""
    PointVertex(position, intensity, rgba, classification)

24-byte interleaved point record:

| offset | field          | type          |
|-------:|----------------|---------------|
| 0      | position       | 3 × Float32   |
| 12     | intensity      | Float32 (0–1) |
| 16     | rgba           | 4 × UInt8     |
| 20     | classification | UInt8 (ASPRS) |
| 21     | flags          | UInt8         |
| 22     | (padding)      | 2 bytes       |
"""
struct PointVertex
    position::Vec3f
    intensity::Float32
    rgba::RGBA8
    classification::UInt8
    flags::UInt8
    _pad::UInt16
end
PointVertex(p, intensity=0f0, rgba::RGBA8=(0xff, 0xff, 0xff, 0xff), class::Integer=0) =
    PointVertex(Vec3f(p), Float32(intensity), rgba, UInt8(class), 0x00, 0x0000)

"32-byte interleaved mesh vertex: position, normal, uv."
struct MeshVertex
    position::Vec3f
    normal::Vec3f
    uv::Vec2f
end
MeshVertex(p, n) = MeshVertex(Vec3f(p), Vec3f(n), Vec2f(0, 0))

"Per-segment instance record for screen-space lines: endpoints and an RGBA color (40 bytes)."
struct LineVertex
    a::Vec3f
    b::Vec3f
    color::Vec4f
end

@assert sizeof(PointVertex) == 24
# every GPU attribute offset must be a multiple of 4 (see point_layout!)
@assert all(fieldoffset(PointVertex, i) % 4 == 0 for i in (1, 2, 3, 4))
@assert sizeof(MeshVertex) == 32
@assert sizeof(LineVertex) == 40

# ---------------------------------------------------------------------------
# Layers

abstract type Layer end

"""
    PointCloud(vertices; name="points", point_size=2, color_mode=COLOR_HEIGHT)

A point cloud in the scene's local frame. `vertices` is the interleaved
`Vector{PointVertex}` that goes to the GPU as is.
"""
mutable struct PointCloud <: Layer
    name::String
    vertices::Vector{PointVertex}
    point_size::Float32
    color_mode::Int32
    visible::Bool
    gps_time::Vector{Float64}   # optional, empty when absent
    model::Mat4f                # pose: layer → scene frame (see set_pose!)
    rev::Int                    # content revision (see touch!)
end
PointCloud(name, vertices, point_size, color_mode, visible, gps_time) =
    PointCloud(name, vertices, point_size, color_mode, visible, gps_time, IDENTITY4, 0)

@enum ColorMode::Int32 COLOR_HEIGHT = 0 COLOR_INTENSITY = 1 COLOR_CLASSIFICATION = 2 COLOR_RGB = 3 COLOR_AGE = 4

function PointCloud(vertices::Vector{PointVertex}; name="points", point_size=2.0,
                    color_mode=COLOR_HEIGHT, gps_time=Float64[])
    PointCloud(String(name), vertices, Float32(point_size), Int32(color_mode), true, gps_time)
end

"Point cloud from a 3×N (or N×3) matrix or a vector of points, with optional attributes."
function PointCloud(positions::AbstractVector; intensity=nothing, rgb=nothing,
                    classification=nothing, kwargs...)
    n = length(positions)
    v = Vector{PointVertex}(undef, n)
    @inbounds for i in 1:n
        it = intensity === nothing ? 0f0 : Float32(intensity[i])
        c = rgb === nothing ? (0xff, 0xff, 0xff, 0xff) : _rgba8(rgb[i])
        cl = classification === nothing ? 0 : classification[i]
        v[i] = PointVertex(positions[i], it, c, cl)
    end
    hasrgb = rgb !== nothing
    PointCloud(v; color_mode = get(kwargs, :color_mode, hasrgb ? COLOR_RGB : COLOR_HEIGHT),
               (k => val for (k, val) in kwargs if k != :color_mode)...)
end
function PointCloud(m::AbstractMatrix{<:Real}; kwargs...)
    size(m, 1) == 3 || size(m, 2) == 3 || throw(ArgumentError("expected a 3×N or N×3 matrix, got $(size(m))"))
    pts = size(m, 1) == 3 ? [Vec3f(m[1, i], m[2, i], m[3, i]) for i in axes(m, 2)] :
                            [Vec3f(m[i, 1], m[i, 2], m[i, 3]) for i in axes(m, 1)]
    PointCloud(pts; kwargs...)
end

_rgba8(c::RGBA8) = c
_rgba8(c::NTuple{3, UInt8}) = (c..., 0xff)
_rgba8(c) = length(c) == 3 ?
    (_u8(c[1]), _u8(c[2]), _u8(c[3]), 0xff) : (_u8(c[1]), _u8(c[2]), _u8(c[3]), _u8(c[4]))
_u8(x::UInt8) = x
_u8(x::AbstractFloat) = round(UInt8, clamp(x, 0, 1) * 255)
_u8(x::Integer) = UInt8(clamp(x, 0, 255))

Base.length(pc::PointCloud) = length(pc.vertices)
positions(pc::PointCloud) = (v.position for v in pc.vertices)
bounds(pc::PointCloud) = bounds_of(positions(pc))

"""
    TriMesh(vertices, indices; name, color, wireframe)

Indexed triangle mesh. `indices` holds 0-based `UInt32` vertex indices, three
per triangle, exactly as `glDrawElements` reads them.
"""
mutable struct TriMesh <: Layer
    name::String
    vertices::Vector{MeshVertex}
    indices::Vector{UInt32}
    color::Vec4f
    wireframe::Bool
    visible::Bool
    model::Mat4f
    rev::Int
end
TriMesh(vertices, indices; name="mesh", color=Vec4f(0.72, 0.74, 0.78, 1), wireframe=false) =
    TriMesh(String(name), vertices, indices, Vec4f(color), wireframe, true, IDENTITY4, 0)

ntriangles(m::TriMesh) = length(m.indices) ÷ 3
bounds(m::TriMesh) = bounds_of(v.position for v in m.vertices)
@inline function triangle(m::TriMesh, t::Integer)
    i = 3 * (t - 1)
    @inbounds (m.vertices[m.indices[i+1]+1].position,
               m.vertices[m.indices[i+2]+1].position,
               m.vertices[m.indices[i+3]+1].position)
end

"Compute smooth per-vertex normals (area weighted) in place."
function compute_normals!(m::TriMesh)
    acc = zeros(Vec3f, length(m.vertices))
    for t in 1:ntriangles(m)
        i = 3 * (t - 1)
        a, b, c = m.indices[i+1] + 1, m.indices[i+2] + 1, m.indices[i+3] + 1
        n = cross(m.vertices[b].position - m.vertices[a].position,
                  m.vertices[c].position - m.vertices[a].position)
        acc[a] += n; acc[b] += n; acc[c] += n
    end
    for i in eachindex(m.vertices)
        v = m.vertices[i]
        l = norm(acc[i])
        m.vertices[i] = MeshVertex(v.position, l > 0 ? acc[i] / l : Vec3f(0, 0, 1), v.uv)
    end
    return m
end

"A polyline in the local frame with one color."
struct Polyline
    points::Vector{Vec3f}
    color::Vec4f
end

"""
    LineLayer(polylines; name, width)

Polylines (GIS boundaries, roads, trajectories) drawn as screen-space,
anti-aliased ribbons `width` pixels wide.
"""
mutable struct LineLayer <: Layer
    name::String
    polylines::Vector{Polyline}
    width::Float32
    visible::Bool
    model::Mat4f
    rev::Int
end
LineLayer(polylines::Vector{Polyline}; name="lines", width=2.0) =
    LineLayer(String(name), polylines, Float32(width), true, IDENTITY4, 0)

bounds(l::LineLayer) = reduce(union, (bounds_of(p.points) for p in l.polylines); init=AABB())

"Flatten polylines into per-segment instance records for the GPU."
function segments(l::LineLayer)
    n = sum((max(length(p.points) - 1, 0) for p in l.polylines); init=0)
    out = Vector{LineVertex}(undef, n)
    k = 0
    for p in l.polylines, i in 1:length(p.points)-1
        out[k += 1] = LineVertex(p.points[i], p.points[i+1], p.color)
    end
    return out
end
