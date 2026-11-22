# Scene → GPU synchronisation and the per-frame render pass.

"""
    Renderer(scene; settings=RenderSettings())

Owns the GL programs and per-layer GPU objects for one scene in the current
GL context. Layers are uploaded lazily whenever `scene.version` changes;
removed layers are freed.
"""
mutable struct Renderer
    scene::Scene
    settings::RenderSettings
    points::GLProgram
    mesh::GLProgram
    lines::GLProgram
    gpu::IdDict{Layer, GPULayer}
    corners::GLBuffer
    synced_version::Int
    grid::Union{Nothing, Tuple{LineLayer, GPULines}}
    stats::NamedTuple{(:points, :triangles, :segments), NTuple{3, Int}}
    box::AABB                 # scene bounds in the scene frame, updated per frame
    revs::IdDict{Layer, Int}  # content revision each GPU object was built from
    edl_program::GLProgram
    edl::Any                  # Nothing or EDLTarget, created at first use
    labels::GLProgram
    hud::Vector{String}       # status lines drawn bottom-left (set by the viewer)
    hud_gpu::Any              # GPUHud, created on first use
end

"Offscreen color + depth textures the opaque passes render into for EDL."
mutable struct EDLTarget
    fbo::GLuint
    color::GLuint
    depth::GLuint
    vao::GLuint
    w::Int
    h::Int
end

function _tex(internal, format, type, w, h)
    id = Ref{GLuint}(0)
    glGenTextures(1, id)
    glBindTexture(GL_TEXTURE_2D, id[])
    glTexImage2D(GL_TEXTURE_2D, 0, internal, w, h, 0, format, type, C_NULL)
    for (k, v) in ((GL_TEXTURE_MIN_FILTER, GL_NEAREST), (GL_TEXTURE_MAG_FILTER, GL_NEAREST),
                   (GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE), (GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE))
        glTexParameteri(GL_TEXTURE_2D, k, v)
    end
    return id[]
end

function EDLTarget(w, h)
    fbo = Ref{GLuint}(0)
    glGenFramebuffers(1, fbo)
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[])
    c = _tex(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, w, h)
    d = _tex(GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, w, h)
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, c, 0)
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, d, 0)
    glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE || error("EDL framebuffer incomplete")
    vao = Ref{GLuint}(0); glGenVertexArrays(1, vao)
    return EDLTarget(fbo[], c, d, vao[], w, h)
end

function release!(t::EDLTarget)
    glDeleteFramebuffers(1, Ref(t.fbo))
    glDeleteTextures(2, GLuint[t.color, t.depth])
    glDeleteVertexArrays(1, Ref(t.vao))
end

function _edl_target!(r, w, h)
    t = r.edl
    t isa EDLTarget && t.w == w && t.h == h && return t
    t isa EDLTarget && release!(t)
    r.edl = EDLTarget(w, h)
end

function Renderer(scene::Scene; settings::RenderSettings=RenderSettings())
    r = Renderer(scene, settings, GLProgram(POINT_SHADER), GLProgram(MESH_SHADER),
                 GLProgram(LINE_SHADER), IdDict{Layer, GPULayer}(),
                 GLBuffer(GL_ARRAY_BUFFER, QUAD_CORNERS), -1, nothing,
                 (points=0, triangles=0, segments=0), AABB(), IdDict{Layer, Int}(),
                 GLProgram(EDL_SHADER), nothing, GLProgram(LABEL_SHADER), String[], nothing)
    glUseProgram(r.points.id)
    setuniform(r.points, :u_class_colors, CLASS_COLORS)
    sync!(r)
    return r
end

"Upload new layers, re-upload edited ones (`touch!`), free removed ones."
function sync!(r::Renderer)
    structural = r.synced_version != r.scene.version
    if structural
        live = Set{UInt}(objectid(l) for l in r.scene.layers)
        for (l, g) in collect(r.gpu)
            objectid(l) in live || (release!(g); delete!(r.gpu, l); delete!(r.revs, l))
        end
    end
    for l in r.scene.layers
        g = get(r.gpu, l, nothing)
        if g !== nothing
            r.revs[l] == l.rev && continue
            release!(g)                                   # contents changed
        end
        r.gpu[l] = l isa LineLayer || l isa Annotations ? upload(l, r.corners) : upload(l)
        r.revs[l] = l.rev
        structural = true
    end
    structural || return r
    r.box = world_box(r)
    if (old = r.grid) !== nothing
        release!(old[2]); r.grid = nothing
    end
    b = r.box
    rad = isempty(b) ? 10.0 : max(Float64(radius(b)), 1.0)
    step = 10.0^floor(log10(rad / 5))
    gl = grid_layer(min(ceil(1.5rad / step), 100) * step, step;
                    z = isempty(b) ? 0.0 : min(0.0, Float64(b.lo[3])))
    r.grid = (gl, upload(gl, r.corners))
    r.synced_version = r.scene.version
    return r
end

# Layer bounds in its own frame, from data cached at upload (never O(points)).
local_box(g::GPUPoints, l) = g.index.bounds
local_box(g::GPUMesh, l) = g.box
local_box(g::GPULines, l) = g.box
local_box(g::GPUStream, l) = bounds(l)
local_box(g::GPUAnnotations, l) = bounds(l)

"Are any out-of-core layers still loading nodes for the last frame's view?"
streaming(r::Renderer) = any(g -> g isa GPUOctree && streaming(g), values(r.gpu))

"Finish all outstanding out-of-core loads (blocking)."
drain!(r::Renderer) = foreach(g -> g isa GPUOctree && drain!(g), values(r.gpu))

"Scene bounds with current poses (cheap: one box per layer)."
world_box(r::Renderer) = reduce(union, (transform_box(l.model, local_box(r.gpu[l], l))
                                        for l in r.scene.layers if l.visible && haskey(r.gpu, l));
                                init=AABB())

"Accelerators for picking: `(layer, BVH | PointIndex)` pairs."
pick_items(r::Renderer) = [(l, _pick_accel(g)) for (l, g) in r.gpu if _pickable(g)]
_pickable(g) = g isa GPUMesh || g isa GPUPoints || g isa GPUStream || g isa GPUOctree
_pick_accel(g::GPUMesh) = g.bvh
_pick_accel(g::GPUPoints) = g.index
_pick_accel(g::GPUStream) = nothing
_pick_accel(g::GPUOctree) = g

"""
    render_frame!(renderer, camera, width, height)

Draw one frame into the currently bound framebuffer: opaque meshes, then
point clouds, then blended lines.
"""
function render_frame!(r::Renderer, cam::Camera, w::Integer, h::Integer)
    lock(UPDATE_LOCK) do
        _render_frame!(r, cam, w, h)
    end
end

function _render_frame!(r::Renderer, cam::Camera, w::Integer, h::Integer)
    sync!(r)
    r.box = world_box(r)
    s = r.settings
    cam.aspect = w / max(h, 1)
    fit_clip!(cam, union(r.box, _grid_box(r)))
    vp = view_projection(cam)
    fc = FrameContext(vp, frustum_from_matrix(vp), Vec3f(cam.position), Vec2f(w, h),
                      Float32(h / (2tan(cam.fov / 2))), Float32(pixels_per_radian(cam, h)))
    target = Ref{GLint}(0)
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, target)     # window or caller's FBO
    glViewport(0, 0, w, h)
    bg = s.background
    glClearColor(bg[1], bg[2], bg[3], bg[4])
    s.edl && glBindFramebuffer(GL_FRAMEBUFFER, _edl_target!(r, w, h).fbo)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS)
    glEnable(GL_PROGRAM_POINT_SIZE)
    glDisable(GL_BLEND); glDisable(GL_CULL_FACE)

    np = nt = ns = 0
    for l in r.scene.layers                      # pass 1: meshes
        l isa TriMesh && l.visible && (nt += draw!(r.gpu[l], l, r.mesh, fc, s))
    end
    for l in r.scene.layers                      # pass 2: points
        (l isa PointCloud || l isa StreamingPointCloud || l isa OctreeCloud) && l.visible &&
            (np += draw!(r.gpu[l], l, r.points, fc, s))
    end
    if s.edl                                     # composite with eye-dome lighting
        glBindFramebuffer(GL_FRAMEBUFFER, target[])
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        _edl_pass!(r, cam)
    end
    glEnable(GL_BLEND)                           # pass 3: anti-aliased lines
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    grid = r.grid
    s.grid && grid !== nothing && draw!(grid[2], grid[1], r.lines, fc, s)
    glDepthMask(GL_TRUE)
    for l in r.scene.layers
        l isa LineLayer && l.visible && (ns += draw!(r.gpu[l], l, r.lines, fc, s))
    end
    glDisable(GL_DEPTH_TEST)                     # pass 4: annotations, always on top
    for l in r.scene.layers
        l isa Annotations && l.visible && _draw_annotations!(r, r.gpu[l], l, fc)
    end
    s.hud && !isempty(r.hud) && _draw_hud!(r, fc)
    glEnable(GL_DEPTH_TEST)
    glDisable(GL_BLEND)
    glBindVertexArray(0)
    r.stats = (points=np, triangles=nt, segments=ns)
    return r.stats
end

function _draw_annotations!(r::Renderer, g::GPUAnnotations, a::Annotations, fc::FrameContext)
    s = r.settings
    if g.lines.count > 0
        p = r.lines
        glUseProgram(p.id)
        setuniform(p, :u_vp, fc.vp); setuniform(p, :u_model, a.model)
        setuniform(p, :u_viewport, fc.viewport); setuniform(p, :u_width, a.width)
        glBindVertexArray(g.lines.vao)
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, g.lines.count)
    end
    g.count == 0 && return
    p = r.labels
    glUseProgram(p.id)
    setuniform(p, :u_vp, fc.vp); setuniform(p, :u_model, a.model)
    setuniform(p, :u_viewport, fc.viewport)
    setuniform(p, :u_halo_color, Vec4f(s.background[1] * 0.5f0, s.background[2] * 0.5f0, s.background[3] * 0.5f0, 0.85f0))
    glBindVertexArray(g.vao)
    for (halo, width) in ((1, 4.0f0), (0, 1.6f0))      # dark outline, then the text
        setuniform(p, :u_halo, halo)
        setuniform(p, :u_width, width)
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, g.count)
    end
end

function _draw_hud!(r::Renderer, fc::FrameContext)
    r.hud_gpu === nothing && (r.hud_gpu = GPUHud(r.corners))
    g = r.hud_gpu
    set_text!(g, r.hud)
    g.count == 0 && return
    p = r.labels
    glUseProgram(p.id)
    setuniform(p, :u_screen, 1)
    setuniform(p, :u_viewport, fc.viewport)
    setuniform(p, :u_halo_color, Vec4f(0, 0, 0, 0.6))
    glBindVertexArray(g.vao)
    for (halo, width) in ((1, 3.5f0), (0, 1.4f0))
        setuniform(p, :u_halo, halo); setuniform(p, :u_width, width)
        glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, g.count)
    end
    setuniform(p, :u_screen, 0)
end

function _edl_pass!(r::Renderer, cam::Camera)
    t = r.edl; p = r.edl_program; s = r.settings
    glUseProgram(p.id)
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, t.color)
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, t.depth)
    setuniform(p, :u_color, 0)
    setuniform(p, :u_depth, 1)
    setuniform(p, :u_near_far, Vec2f(cam.near, cam.far))
    setuniform(p, :u_ortho, cam.ortho ? 1 : 0)
    setuniform(p, :u_strength, s.edl_strength)
    setuniform(p, :u_radius, s.edl_radius)
    glDepthFunc(GL_ALWAYS)                       # copy depth so lines still occlude
    glBindVertexArray(t.vao)
    glDrawArrays(GL_TRIANGLES, 0, 3)
    glDepthFunc(GL_LESS)
    glActiveTexture(GL_TEXTURE0)
end

_grid_box(r::Renderer) = (g = r.grid; g === nothing || !r.settings.grid ? AABB() : g[2].box)

function release!(r::Renderer)
    if r.hud_gpu isa GPUHud
        glDeleteVertexArrays(1, Ref(r.hud_gpu.vao)); release!(r.hud_gpu.instances); r.hud_gpu = nothing
    end
    r.edl isa EDLTarget && (release!(r.edl); r.edl = nothing)
    foreach(release!, values(r.gpu)); empty!(r.gpu)
    (g = r.grid) === nothing || release!(g[2])
    release!(r.corners)
end
