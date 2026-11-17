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
end

function Renderer(scene::Scene; settings::RenderSettings=RenderSettings())
    r = Renderer(scene, settings, GLProgram(POINT_SHADER), GLProgram(MESH_SHADER),
                 GLProgram(LINE_SHADER), IdDict{Layer, GPULayer}(),
                 GLBuffer(GL_ARRAY_BUFFER, QUAD_CORNERS), -1, nothing,
                 (points=0, triangles=0, segments=0), AABB(), IdDict{Layer, Int}())
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
        r.gpu[l] = l isa LineLayer ? upload(l, r.corners) : upload(l)
        r.revs[l] = l.rev
        structural = true
    end
    structural || return r
    r.box = world_box(r)
    if r.grid !== nothing
        release!(r.grid[2]); r.grid = nothing
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

"Scene bounds with current poses (cheap: one box per layer)."
world_box(r::Renderer) = reduce(union, (transform_box(l.model, local_box(r.gpu[l], l))
                                        for l in r.scene.layers if l.visible && haskey(r.gpu, l));
                                init=AABB())

"Accelerators for picking: `(layer, BVH | PointIndex)` pairs."
pick_items(r::Renderer) =
    [(l, g isa GPUMesh ? g.bvh : g.index) for (l, g) in r.gpu if g isa GPUMesh || g isa GPUPoints]

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
    glViewport(0, 0, w, h)
    bg = s.background
    glClearColor(bg[1], bg[2], bg[3], bg[4])
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LESS)
    glEnable(GL_PROGRAM_POINT_SIZE)
    glDisable(GL_BLEND); glDisable(GL_CULL_FACE)

    np = nt = ns = 0
    for l in r.scene.layers                      # pass 1: meshes
        l isa TriMesh && l.visible && (nt += draw!(r.gpu[l], l, r.mesh, fc, s))
    end
    for l in r.scene.layers                      # pass 2: points
        (l isa PointCloud || l isa StreamingPointCloud) && l.visible &&
            (np += draw!(r.gpu[l], l, r.points, fc, s))
    end
    glEnable(GL_BLEND)                           # pass 3: anti-aliased lines
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glDepthMask(GL_FALSE)
    s.grid && r.grid !== nothing && draw!(r.grid[2], r.grid[1], r.lines, fc, s)
    glDepthMask(GL_TRUE)
    for l in r.scene.layers
        l isa LineLayer && l.visible && (ns += draw!(r.gpu[l], l, r.lines, fc, s))
    end
    glDisable(GL_BLEND)
    glBindVertexArray(0)
    r.stats = (points=np, triangles=nt, segments=ns)
    return r.stats
end

_grid_box(r::Renderer) = r.grid === nothing || !r.settings.grid ? AABB() : r.grid[2].box

function release!(r::Renderer)
    foreach(release!, values(r.gpu)); empty!(r.gpu)
    r.grid === nothing || release!(r.grid[2])
    release!(r.corners)
end
