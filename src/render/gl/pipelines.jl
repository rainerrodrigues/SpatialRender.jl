# Per-modality draw calls. Each `draw!` binds its program state and issues
# the GL calls for one layer; nothing here allocates.

"""
    RenderSettings(; kwargs...)

- `background`: clear color.
- `point_size_mode`: `:pixels` (constant screen size) or `:world` (metres,
  shrinks with distance; `world_point_size`).
- `round_points`: circular splats with sphere shading (`false`: squares).
- `lod_density`: points drawn per pixel² of a chunk's screen footprint;
  `Inf` disables level of detail.
- `wireframe_color`: overlay color for meshes with `wireframe = true`.
- `grid`: draw a ground grid at z = 0.
"""
Base.@kwdef mutable struct RenderSettings
    background::Vec4f = Vec4f(0.08, 0.09, 0.11, 1)
    point_size_mode::Symbol = :pixels
    world_point_size::Float32 = 0.05f0
    round_points::Bool = true
    lod_density::Float64 = 1.5
    wireframe_color::Vec4f = Vec4f(0.05, 0.05, 0.08, 0.6)
    grid::Bool = true
    highlight::Tuple{Any, Int} = (nothing, 0)   # (layer, 1-based point index)
end

struct FrameContext
    vp::Mat4f
    frustum::Frustum
    eye::Vec3f
    viewport::Vec2f
    proj_scale::Float32
    pixels_per_radian::Float32
end

# Frustum / eye in a layer's own coordinates: planes of VP·M are the
# frustum expressed in the layer frame, so chunk boxes never need transforming.
@inline function local_view(fc::FrameContext, M::Mat4f)
    M === IDENTITY4 && return fc.frustum, fc.eye
    return frustum_from_matrix(fc.vp * M), transform_point(inverse_pose(M), fc.eye)
end

function _point_uniforms!(p::GLProgram, fc::FrameContext, s::RenderSettings, M::Mat4f,
                          size, color_mode, zr::Vec2f)
    glUseProgram(p.id)
    setuniform(p, :u_vp, fc.vp)
    setuniform(p, :u_model, M)
    world = s.point_size_mode == :world
    setuniform(p, :u_size_mode, world ? 1 : 0)
    setuniform(p, :u_point_size, world ? s.world_point_size : size)
    setuniform(p, :u_proj_scale, fc.proj_scale)
    setuniform(p, :u_color_mode, color_mode)
    setuniform(p, :u_zrange, zr)
    setuniform(p, :u_round, s.round_points ? 1 : 0)
end

function _zrange(M, box)
    b = transform_box(M, box)
    isempty(b) ? Vec2f(0, 1) : Vec2f(b.lo[3], b.hi[3])
end

function draw!(g::GPUPoints, pc::PointCloud, p::GLProgram, fc::FrameContext, s::RenderSettings)
    fr, eye = local_view(fc, pc.model)
    vis = cull!(g.index, fr; eye,
                pixels_per_radian=s.lod_density == Inf ? Inf : fc.pixels_per_radian,
                density=s.lod_density)
    vis.n == 0 && return 0
    _point_uniforms!(p, fc, s, pc.model, pc.point_size, pc.color_mode, _zrange(pc.model, g.index.bounds))
    setuniform(p, :u_seq, 0)
    hl = s.highlight[1] === pc ? s.highlight[2] - 1 : -1
    setuniform(p, :u_highlight, hl)
    glBindVertexArray(g.vao)
    r = vis.ranges
    GC.@preserve r glMultiDrawArrays(GL_POINTS, pointer(r.first), pointer(r.count), vis.n)
    return vis.npoints
end

function draw!(g::GPUStream, sp::StreamingPointCloud, p::GLProgram, fc::FrameContext, s::RenderSettings)
    sync_stream!(g, sp)                    # upload dirty ranges only
    n = length(sp)
    n == 0 && return 0
    box = bounds(sp)
    fr, _ = local_view(fc, sp.model)
    intersects(fr, box) || return 0
    _point_uniforms!(p, fc, s, sp.model, sp.point_size, sp.color_mode, _zrange(sp.model, box))
    setuniform(p, :u_seq, sp.seq)
    setuniform(p, :u_max_age, sp.max_age)
    setuniform(p, :u_highlight, -1)
    glBindVertexArray(g.vao)
    glDrawArrays(GL_POINTS, 0, n)
    return n
end

function draw!(g::GPUMesh, m::TriMesh, p::GLProgram, fc::FrameContext, s::RenderSettings)
    fr, _ = local_view(fc, m.model)
    intersects(fr, g.box) || return 0
    glUseProgram(p.id)
    setuniform(p, :u_vp, fc.vp)
    setuniform(p, :u_model, m.model)
    setuniform(p, :u_normal, normal_matrix(m.model))
    setuniform(p, :u_eye, fc.eye)
    setuniform(p, :u_color, m.color)
    setuniform(p, :u_flat, 0)
    glBindVertexArray(g.vao)
    if m.wireframe
        glEnable(GL_POLYGON_OFFSET_FILL)
        glPolygonOffset(1f0, 1f0)          # push faces back so edges win
    end
    glDrawElements(GL_TRIANGLES, g.count, GL_UNSIGNED_INT, C_NULL)
    if m.wireframe
        glDisable(GL_POLYGON_OFFSET_FILL)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        setuniform(p, :u_flat, 1)
        setuniform(p, :u_color, s.wireframe_color)
        glDrawElements(GL_TRIANGLES, g.count, GL_UNSIGNED_INT, C_NULL)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    end
    return g.count ÷ 3
end

function draw!(g::GPULines, l::LineLayer, p::GLProgram, fc::FrameContext, s::RenderSettings)
    g.count == 0 && return 0
    fr, _ = local_view(fc, l.model)
    intersects(fr, g.box) || return 0
    glUseProgram(p.id)
    setuniform(p, :u_vp, fc.vp)
    setuniform(p, :u_model, l.model)
    setuniform(p, :u_viewport, fc.viewport)
    setuniform(p, :u_width, l.width)
    glBindVertexArray(g.vao)
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, g.count)
    return g.count
end

"A ground grid as a LineLayer: lines every `step` metres over `±half` (plus axes)."
function grid_layer(half::Real, step::Real; z::Real=0)
    lines = Polyline[]
    n = floor(Int, half / step)
    h = n * step
    minor = Vec4f(0.35, 0.37, 0.42, 0.35)
    for i in -n:n
        i == 0 && continue
        c = i % 10 == 0 ? Vec4f(0.45, 0.47, 0.52, 0.6) : minor
        push!(lines, Polyline([Vec3f(i * step, -h, z), Vec3f(i * step, h, z)], c))
        push!(lines, Polyline([Vec3f(-h, i * step, z), Vec3f(h, i * step, z)], c))
    end
    push!(lines, Polyline([Vec3f(0, 0, z), Vec3f(h, 0, z)], Vec4f(0.9, 0.25, 0.25, 0.9)))   # +X / east
    push!(lines, Polyline([Vec3f(0, 0, z), Vec3f(0, h, z)], Vec4f(0.3, 0.85, 0.3, 0.9)))    # +Y / north
    push!(lines, Polyline([Vec3f(-h, 0, z), Vec3f(0, 0, z)], minor))
    push!(lines, Polyline([Vec3f(0, -h, z), Vec3f(0, 0, z)], minor))
    return LineLayer(lines; name="grid", width=1.0)
end
