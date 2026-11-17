# Native GLFW window, input wiring, the interactive loop, and offscreen
# rendering / screenshots.

const CONTEXT_COUNTER = Ref{UInt}(0)

function _window_hints(; visible::Bool=true, samples::Int=4)
    GLFW.WindowHint(GLFW.CONTEXT_VERSION_MAJOR, 4)
    GLFW.WindowHint(GLFW.CONTEXT_VERSION_MINOR, 1)
    GLFW.WindowHint(GLFW.OPENGL_PROFILE, GLFW.OPENGL_CORE_PROFILE)
    GLFW.WindowHint(GLFW.OPENGL_FORWARD_COMPAT, true)
    GLFW.WindowHint(GLFW.VISIBLE, visible)
    GLFW.WindowHint(GLFW.SAMPLES, samples)
end

function _make_current(win)
    GLFW.MakeContextCurrent(win)
    CONTEXT_COUNTER[] += 1
    CURRENT_CONTEXT[] = CONTEXT_COUNTER[]
    return CURRENT_CONTEXT[]
end

"Run `f()` with a hidden 1×1 GL 4.1 context current (validation, offscreen work)."
function with_hidden_context(f)
    _window_hints(visible=false, samples=0)
    win = GLFW.CreateWindow(1, 1, "SpatialRender (hidden)")
    ctx = _make_current(win)
    try
        return f()
    finally
        drop_context_programs!(ctx)
        GLFW.DestroyWindow(win)
    end
end

"""
    Viewer

An open window: the GLFW handle, the [`Renderer`](@ref) and the
[`CameraController`](@ref). Returned by [`open_viewer`](@ref) when
`block=false` so the scene can be changed while the window is open.
"""
mutable struct Viewer
    window::GLFW.Window
    renderer::Renderer
    controller::CameraController
    context::UInt
    last_hit::RayHit
    open::Bool
end

const KEYMAP = Dict(GLFW.KEY_W => :w, GLFW.KEY_A => :a, GLFW.KEY_S => :s, GLFW.KEY_D => :d,
                    GLFW.KEY_Q => :q, GLFW.KEY_E => :e, GLFW.KEY_LEFT_SHIFT => :shift,
                    GLFW.KEY_RIGHT_SHIFT => :shift, GLFW.KEY_TAB => :tab)

const HELP = """
SpatialRender controls
  left drag      orbit (orbit mode) / look (fly mode)
  right drag     pan            (also middle drag, shift + left drag)
  scroll         zoom (orbit) / fly speed (fly)
  W A S D Q E    fly (shift: faster)       Tab   orbit <-> fly
  double-click   pick: set pivot and print the point
  F  frame all    T  top view    O  ortho / perspective
  1 height  2 intensity  3 classification  4 RGB   (point colors)
  + / -  point size    P  pixel / world point size    L  LOD on/off
  M  mesh wireframe    G  grid    H  help    S-shot: F12    Esc  close
"""

"""
    open_viewer(scene; width=1280, height=800, title, block=true, camera=nothing, settings) -> Viewer

Open a native window and render `scene` interactively (see `HELP` for
controls; press H in the window). With `block=false` the function returns
the `Viewer` immediately; call `run!(viewer)` or `step!(viewer)` to drive
it, e.g. from a robot's own update loop.
"""
function open_viewer(scene::Scene; width::Integer=1280, height::Integer=800,
                     title::AbstractString="SpatialRender", block::Bool=true,
                     camera::Union{Nothing, Camera}=nothing, settings::RenderSettings=RenderSettings(),
                     vsync::Bool=true)
    _window_hints()
    win = GLFW.CreateWindow(width, height, title)
    ctx = _make_current(win)
    GLFW.SwapInterval(vsync ? 1 : 0)
    glEnable(GL_MULTISAMPLE)
    r = Renderer(scene; settings)
    cam = something(camera, Camera())
    camera === nothing && frame!(cam, r.box)
    ctl = CameraController(cam)
    fw, fh = GLFW.GetFramebufferSize(win)
    set_viewport!(ctl, fw, fh)
    v = Viewer(win, r, ctl, ctx, NO_HIT, true)
    _install_callbacks!(v)
    block || return v
    run!(v)
    return v
end

function _install_callbacks!(v::Viewer)
    win = v.window; ctl = v.controller
    GLFW.SetFramebufferSizeCallback(win, (_, w, h) -> set_viewport!(ctl, w, h))
    GLFW.SetCursorPosCallback(win, (_, x, y) -> begin
        # cursor coordinates are in window units; scale to framebuffer pixels
        sx, sy = _content_scale(win)
        cursor!(ctl, x * sx, y * sy)
    end)
    GLFW.SetScrollCallback(win, (_, dx, dy) -> scroll!(ctl, dy))
    GLFW.SetMouseButtonCallback(win, (_, button, action, mods) -> begin
        b = button == GLFW.MOUSE_BUTTON_LEFT ? :left : button == GLFW.MOUSE_BUTTON_RIGHT ? :right : :middle
        shift = (Int(mods) & Int(GLFW.MOD_SHIFT)) != 0
        ev = mouse_button!(ctl, b, action == GLFW.PRESS; shift, time=time())
        ev == :double_click && pick!(v, ctl.input.cursor[1], ctl.input.cursor[2]; set_pivot=true)
    end)
    GLFW.SetKeyCallback(win, (_, key, scancode, action, mods) -> _on_key(v, key, action))
end

function _content_scale(win)
    ww, wh = GLFW.GetWindowSize(win)
    fw, fh = GLFW.GetFramebufferSize(win)
    return (ww > 0 ? fw / ww : 1.0, wh > 0 ? fh / wh : 1.0)
end

function _on_key(v::Viewer, key, action)
    ctl = v.controller; r = v.renderer; s = r.settings; cam = ctl.camera
    sym = get(KEYMAP, key, nothing)
    if sym !== nothing && action != GLFW.REPEAT
        key!(ctl, sym, action == GLFW.PRESS)
        # in fly mode WASD drive the camera, so the letter shortcuts below
        # only fire in orbit mode for those keys
        ctl.mode == FlyMode && sym in (:w, :a, :s, :d, :q, :e) && return
    end
    action == GLFW.PRESS || action == GLFW.REPEAT || return
    pcs = collect(layers(r.scene, PointCloud))
    if key == GLFW.KEY_ESCAPE
        GLFW.SetWindowShouldClose(v.window, true)
    elseif key == GLFW.KEY_F
        frame!(cam, r.box)
    elseif key == GLFW.KEY_T
        tgt = target(cam)
        cam.position = tgt + Vec3d(0, -1e-6, cam.distance)
        look_at!(cam, tgt)
    elseif key == GLFW.KEY_O
        cam.ortho = !cam.ortho
    elseif key in (GLFW.KEY_1, GLFW.KEY_2, GLFW.KEY_3, GLFW.KEY_4)
        m = Int32(Int(key) - Int(GLFW.KEY_1))
        foreach(pc -> pc.color_mode = m, pcs)
    elseif key in (GLFW.KEY_EQUAL, GLFW.KEY_KP_ADD)
        s.world_point_size *= 1.25f0
        foreach(pc -> pc.point_size = min(pc.point_size + 1, 64), pcs)
    elseif key in (GLFW.KEY_MINUS, GLFW.KEY_KP_SUBTRACT)
        s.world_point_size /= 1.25f0
        foreach(pc -> pc.point_size = max(pc.point_size - 1, 1), pcs)
    elseif key == GLFW.KEY_P
        s.point_size_mode = s.point_size_mode == :pixels ? :world : :pixels
    elseif key == GLFW.KEY_L
        s.lod_density = isinf(s.lod_density) ? RenderSettings().lod_density : Inf
        @info "level of detail $(isinf(s.lod_density) ? "off" : "on")"
    elseif key == GLFW.KEY_M
        for m in layers(r.scene, TriMesh)
            m.wireframe = !m.wireframe
        end
    elseif key == GLFW.KEY_G
        s.grid = !s.grid
    elseif key == GLFW.KEY_H
        print(HELP)
    elseif key == GLFW.KEY_F12
        path = "spatialrender_$(round(Int, time())).png"
        fw, fh = GLFW.GetFramebufferSize(v.window)
        write_png(path, read_framebuffer(fw, fh))
        @info "saved $path"
    end
    ctl.dirty = true
end

"""
    pick!(viewer, x, y; set_pivot=false) -> RayHit

Cast a ray through framebuffer pixel `(x, y)` against all meshes (BVH) and
point clouds (chunk index, 4-pixel tolerance). Prints the hit in local and
source coordinates and highlights picked points.
"""
function pick!(v::Viewer, x::Real, y::Real; set_pivot::Bool=false)
    ctl = v.controller; cam = ctl.camera
    w, h = ctl.viewport
    ray = screen_ray(cam, x, y, w, h)
    hit = raycast(pick_items(v.renderer), ray; angle = 4 * cam.fov / h)
    v.last_hit = hit
    ishit(hit) || return hit
    p = hit.position
    world = to_world(v.renderer.scene.frame, Vec3d(p))
    kind = hit.layer isa PointCloud ? "point" : "triangle"
    @info @sprintf("picked %s %d of \"%s\"  local (%.3f, %.3f, %.3f)  source (%.6f, %.6f, %.3f)  range %.2f m",
                   kind, hit.index, hit.layer.name, p..., world..., hit.t)
    v.renderer.settings.highlight = hit.layer isa PointCloud ? (hit.layer, hit.index) : (nothing, 0)
    set_pivot && set_pivot!(ctl, p)
    return hit
end

"""
    step!(viewer) -> Bool

Process events and draw one frame. Returns `false` once the window was
closed (and releases it).
"""
function step!(v::Viewer)
    v.open || return false
    if GLFW.WindowShouldClose(v.window)
        close(v); return false
    end
    GLFW.PollEvents()
    _make_current_if_needed(v)
    now = time()
    dt = min(now - _last_time(v, now), 0.1)
    update!(v.controller, dt)
    fw, fh = GLFW.GetFramebufferSize(v.window)
    st = render_frame!(v.renderer, v.controller.camera, fw, fh)
    GLFW.SwapBuffers(v.window)
    _title!(v, st, dt)
    return true
end

const _LAST_TIME = IdDict{Viewer, Float64}()
_last_time(v, now) = (t = get(_LAST_TIME, v, now); _LAST_TIME[v] = now; t)

function _make_current_if_needed(v::Viewer)
    if CURRENT_CONTEXT[] != v.context
        GLFW.MakeContextCurrent(v.window)
        CURRENT_CONTEXT[] = v.context
    end
end

const _TITLE_T = Ref(0.0)
function _title!(v::Viewer, st, dt)
    t = time()
    t - _TITLE_T[] < 0.5 && return
    _TITLE_T[] = t
    mode = v.controller.mode == OrbitMode ? "orbit" : "fly"
    GLFW.SetWindowTitle(v.window, @sprintf("SpatialRender  |  %s  |  %.0f fps  |  %s pts  %s tris  %s segs",
        mode, 1 / max(dt, 1e-6), _si(st.points), _si(st.triangles), _si(st.segments)))
end
_si(n) = n >= 1_000_000 ? @sprintf("%.1fM", n / 1e6) : n >= 1000 ? @sprintf("%.1fk", n / 1e3) : string(n)

"Run the event / render loop until the window is closed."
function run!(v::Viewer)
    print(HELP)
    while step!(v)
        yield()       # let other Julia tasks (e.g. data feeds) run
    end
    return v
end

function Base.close(v::Viewer)
    v.open || return
    _make_current_if_needed(v)
    release!(v.renderer)
    drop_context_programs!(v.context)
    GLFW.DestroyWindow(v.window)
    delete!(_LAST_TIME, v)
    v.open = false
    return
end

# ---------------------------------------------------------------------------
# Offscreen rendering

"Read the current framebuffer as an `h×w` matrix of RGB tuples (top row first)."
function read_framebuffer(w::Integer, h::Integer)
    buf = Vector{UInt8}(undef, 3w * h)
    glPixelStorei(GL_PACK_ALIGNMENT, 1)
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf)
    img = Matrix{NTuple{3, UInt8}}(undef, h, w)
    for y in 1:h, x in 1:w
        k = 3 * ((h - y) * w + (x - 1))
        img[y, x] = (buf[k+1], buf[k+2], buf[k+3])
    end
    return img
end

"""
    render_offscreen(scene; width=1024, height=768, camera=nothing, settings) -> Matrix{NTuple{3,UInt8}}

Render one frame into a hidden framebuffer and return it as an image
(rows top to bottom). With `camera=nothing` the camera frames the scene
from the south-west, 30° above the horizon. Useful for CI snapshots and
batch reports.
"""
function render_offscreen(scene::Scene; width::Integer=1024, height::Integer=768,
                          camera::Union{Nothing, Camera}=nothing,
                          settings::RenderSettings=RenderSettings())
    with_hidden_context() do
        fbo = Ref{GLuint}(0); rb = Ref{GLuint}(0); db = Ref{GLuint}(0)
        glGenFramebuffers(1, fbo); glBindFramebuffer(GL_FRAMEBUFFER, fbo[])
        glGenRenderbuffers(1, rb); glBindRenderbuffer(GL_RENDERBUFFER, rb[])
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[])
        glGenRenderbuffers(1, db); glBindRenderbuffer(GL_RENDERBUFFER, db[])
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, db[])
        glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE ||
            error("offscreen framebuffer incomplete")
        r = Renderer(scene; settings)
        cam = camera
        if cam === nothing
            b = r.box
            c = isempty(b) ? Vec3d(0, 0, 0) : Vec3d(center(b))
            cam = Camera(position = c + Vec3d(-1, -1, 0.82), target = c, aspect = width / height)
            frame!(cam, b)
        end
        render_frame!(r, cam, width, height)
        glFinish()
        img = read_framebuffer(width, height)
        release!(r)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glDeleteRenderbuffers(1, rb); glDeleteRenderbuffers(1, db); glDeleteFramebuffers(1, fbo)
        return img
    end
end

"""
    screenshot(path, scene; kwargs...) -> path

[`render_offscreen`](@ref) and save the image as PNG.
"""
screenshot(path::AbstractString, scene::Scene; kwargs...) = write_png(path, render_offscreen(scene; kwargs...))

# ---------------------------------------------------------------------------
# Dependency-free PNG writer (zlib "stored" blocks: larger files, no codec)

const _CRC_TABLE = let t = Vector{UInt32}(undef, 256)
    for n in 0:255
        c = UInt32(n)
        for _ in 1:8
            c = (c & 1) != 0 ? 0xedb88320 ⊻ (c >> 1) : c >> 1
        end
        t[n+1] = c
    end
    t
end
function _crc32(data)
    c = 0xffffffff
    for b in data
        c = _CRC_TABLE[((c ⊻ b) & 0xff)+1] ⊻ (c >> 8)
    end
    return c ⊻ 0xffffffff
end
function _adler32(data)
    a, b = UInt32(1), UInt32(0)
    for x in data
        a = (a + x) % 65521; b = (b + a) % 65521
    end
    return (b << 16) | a
end

"""
    write_png(path, img::Matrix{NTuple{3,UInt8}}) -> path
"""
function write_png(path::AbstractString, img::AbstractMatrix{NTuple{3, UInt8}})
    h, w = size(img)
    raw = Vector{UInt8}(undef, h * (1 + 3w))
    k = 0
    for y in 1:h
        raw[k += 1] = 0x00                       # filter: none
        for x in 1:w, c in img[y, x]
            raw[k += 1] = c
        end
    end
    z = IOBuffer()
    write(z, 0x78, 0x01)
    n = length(raw); i = 1
    while i <= n
        len = min(65535, n - i + 1)
        last = i + len > n
        write(z, UInt8(last), htol(UInt16(len)), htol(~UInt16(len)))
        write(z, view(raw, i:i+len-1))
        i += len
    end
    write(z, hton(_adler32(raw)))
    idat = take!(z)
    chunk(io, typ, data) = (write(io, hton(UInt32(length(data)))); td = vcat(Vector{UInt8}(typ), data);
                            write(io, td); write(io, hton(_crc32(td))))
    open(path, "w") do io
        write(io, UInt8[0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])
        ihdr = IOBuffer()
        write(ihdr, hton(UInt32(w)), hton(UInt32(h)), UInt8(8), UInt8(2), UInt8(0), UInt8(0), UInt8(0))
        chunk(io, "IHDR", take!(ihdr))
        chunk(io, "IDAT", idat)
        chunk(io, "IEND", UInt8[])
    end
    return path
end
