# Native GLFW window, input wiring and the interactive loop. Offscreen rendering
# is in offscreen.jl, the PNG writer in png.jl.

const CONTEXT_COUNTER = Ref{UInt}(0)

function _window_hints(; visible::Bool=true, samples::Int=4)
    configure_gpu!()                  # once per session: pick a hardware GPU on WSL
    GLFW.DefaultWindowHints()
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

"""
    gl_info() -> (vendor, renderer, version, software)

Describe the current GL context; `software` is true for CPU rasterisers
(llvmpipe, softpipe, SwiftShader, Microsoft Basic Render).
"""
function gl_info()
    str(x) = (p = glGetString(x); p == C_NULL ? "" : unsafe_string(p))
    ren = str(GL_RENDERER)
    sw = occursin(r"llvmpipe|softpipe|swrast|SwiftShader|Basic Render"i, ren)
    return (vendor=str(GL_VENDOR), renderer=ren, version=str(GL_VERSION), software=sw)
end

function warn_if_software()
    info = gl_info()
    info.software && @warn """OpenGL is running on the CPU ($(info.renderer)); frame rates will be low.
        GPU selection: $(GPU_SELECTION[])
        On WSL2, Mesa needs GALLIUM_DRIVER=d3d12 and, on hybrid laptops,
        MESA_D3D12_DEFAULT_ADAPTER_NAME=<NVIDIA|AMD|Intel> to skip an adapter whose driver fails.
        If no adapter works, update the Windows GPU driver and run `wsl --update`."""
    return info
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
    measuring::Bool                         # measure mode (R)
    pending::Union{Nothing, Vec3f}          # first point of a measurement in progress
    annotations::Union{Nothing, Annotations}
    player::Any                             # a BagPlayer (or nothing), ticked every frame
    fps::Float64                            # smoothed frame rate for the overlay
    last_time::Float64                      # wall clock of the previous frame (NaN before the first)
    title_time::Float64                     # last overlay / title refresh
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
  1 height  2 intensity  3 classification  4 RGB  5 scan age   (point colors)
  + / -  point size    P  pixel / world point size    L  LOD on/off
  M  mesh wireframe    G  grid    B  eye-dome lighting    H  help
  R  measure mode: click two points for distance, Δz and slope; Backspace undoes
  I  status overlay    F12  screenshot    Esc  close
  bag replay:  Space play/pause   <- ->  seek 1 s (shift 10 s)   [ ]  slower/faster   Home  restart
"""

"""
    open_viewer(scene; width=1280, height=800, title, block=true, camera=nothing, settings,
                vsync=true, player=nothing, visible=true) -> Viewer

Open a native window and render `scene` interactively (see `HELP` for
controls; press H in the window). `visible=false` keeps the window hidden
(automated tests, scripted captures). With `block=false` the function returns
the `Viewer` immediately; call `run!(viewer)` or `step!(viewer)` to drive
it, e.g. from a robot's own update loop.
"""
function open_viewer(scene::Scene; width::Integer=1280, height::Integer=800,
                     title::AbstractString="SpatialRender", block::Bool=true,
                     camera::Union{Nothing, Camera}=nothing, settings::RenderSettings=RenderSettings(),
                     vsync::Bool=true, player=nothing, visible::Bool=true)
    width > 0 && height > 0 || throw(ArgumentError("window size must be positive, got $(width)×$(height)"))
    _window_hints(; visible)
    win = GLFW.CreateWindow(width, height, title)
    ctx = _make_current(win)
    GLFW.SwapInterval(vsync ? 1 : 0)
    glEnable(GL_MULTISAMPLE)
    warn_if_software()
    r = Renderer(scene; settings)
    cam = something(camera, Camera())
    camera === nothing && frame!(cam, r.box)
    ctl = CameraController(cam)
    fw, fh = GLFW.GetFramebufferSize(win)
    set_viewport!(ctl, fw, fh)
    v = Viewer(win, r, ctl, ctx, NO_HIT, true, false, nothing, nothing, player, 0.0, NaN, 0.0)
    _install_callbacks!(v)
    block || return v
    run!(v)
    return v
end

# An exception inside a GLFW callback would unwind through C into the render
# loop and close the window; report it and keep the viewer running instead.
function _safely(f, what)
    try
        f()
    catch e
        e isa InterruptException && rethrow()
        @error "SpatialRender viewer: error while handling $what" exception=(e, catch_backtrace()) maxlog=10
    end
    return nothing
end

function _on_mouse_button(v::Viewer, button, action, mods)
    ctl = v.controller
    b = button == GLFW.MOUSE_BUTTON_LEFT ? :left : button == GLFW.MOUSE_BUTTON_RIGHT ? :right : :middle
    shift = (Int(mods) & Int(GLFW.MOD_SHIFT)) != 0
    ev = mouse_button!(ctl, b, action == GLFW.PRESS; shift, time=time())
    x, y = ctl.input.cursor
    if ev == :double_click
        pick!(v, x, y; set_pivot=true)
    elseif ev == :click && v.measuring
        measure_click!(v, x, y)
    end
end

function _install_callbacks!(v::Viewer)
    win = v.window; ctl = v.controller
    GLFW.SetFramebufferSizeCallback(win, (_, w, h) -> _safely(() -> set_viewport!(ctl, w, h), "a resize"))
    GLFW.SetCursorPosCallback(win, (_, x, y) -> _safely("mouse motion") do
        # cursor coordinates are in window units; scale to framebuffer pixels
        sx, sy = _content_scale(win)
        cursor!(ctl, x * sx, y * sy)
    end)
    GLFW.SetScrollCallback(win, (_, dx, dy) -> _safely(() -> scroll!(ctl, dy), "scrolling"))
    GLFW.SetMouseButtonCallback(win, (_, button, action, mods) ->
        _safely(() -> _on_mouse_button(v, button, action, mods), "a mouse button"))
    GLFW.SetKeyCallback(win, (_, key, scancode, action, mods) ->
        _safely(() -> _on_key(v, key, action), "a key press"))
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
    pcs = [l for l in r.scene.layers if l isa PointCloud || l isa StreamingPointCloud || l isa OctreeCloud]
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
    elseif key in (GLFW.KEY_1, GLFW.KEY_2, GLFW.KEY_3, GLFW.KEY_4, GLFW.KEY_5)
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
    elseif key == GLFW.KEY_B
        s.edl = !s.edl
    elseif key == GLFW.KEY_R
        v.measuring = !v.measuring
        v.measuring || _cancel_pending!(v)
        @info v.measuring ? "measure mode: click two points (Backspace undoes, R exits)" : "measure mode off"
    elseif key == GLFW.KEY_BACKSPACE && (ann = v.annotations) !== nothing
        v.pending = nothing
        undo!(ann)
    elseif key == GLFW.KEY_H
        print(HELP)
    elseif key == GLFW.KEY_I
        s.hud = !s.hud
    elseif v.player !== nothing && key in (GLFW.KEY_SPACE, GLFW.KEY_LEFT, GLFW.KEY_RIGHT,
                                           GLFW.KEY_LEFT_BRACKET, GLFW.KEY_RIGHT_BRACKET, GLFW.KEY_HOME)
        _player_key!(v.player, key, :shift in ctl.input.held)
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
    kind = hit.layer isa TriMesh ? "triangle" : "point"
    @info @sprintf("picked %s %d of \"%s\"  local (%.3f, %.3f, %.3f)  source (%.6f, %.6f, %.3f)  range %.2f m",
                   kind, hit.index, hit.layer.name, p..., world..., hit.t)
    v.renderer.settings.highlight = hit.layer isa PointCloud ? (hit.layer, hit.index) : (nothing, 0)
    set_pivot && set_pivot!(ctl, p)
    return hit
end

function _player_key!(p, key, shift)
    if key == GLFW.KEY_SPACE
        toggle!(p)
    elseif key == GLFW.KEY_LEFT || key == GLFW.KEY_RIGHT
        step = (shift ? 10.0 : 1.0) * (key == GLFW.KEY_LEFT ? -1 : 1)
        seek!(p, current_time(p) + step)
    elseif key == GLFW.KEY_LEFT_BRACKET
        p.rate = max(p.rate / 2, 1 / 64)
    elseif key == GLFW.KEY_RIGHT_BRACKET
        p.rate = min(p.rate * 2, 64.0)
    elseif key == GLFW.KEY_HOME
        seek!(p, p.t0)
    end
end

"The viewer's annotation layer, created on first use."
function annotations!(v::Viewer)
    a = v.annotations
    if a === nothing
        a = add!(v.renderer.scene, Annotations(name="measurements"))
        v.annotations = a
    end
    return a::Annotations
end

_cancel_pending!(v::Viewer) = v.pending === nothing || (v.pending = nothing; undo!(annotations!(v)))

"""
    measure_click!(viewer, x, y) -> Union{Nothing, Tuple}

Measure-mode click at framebuffer pixel `(x, y)`: the first click marks a
point, the second adds the measurement and returns `(distance, dz, slope)`.
"""
function measure_click!(v::Viewer, x::Real, y::Real)
    hit = pick!(v, x, y)
    ishit(hit) || return nothing
    a = annotations!(v)
    if v.pending === nothing
        v.pending = hit.position
        annotate!(a, hit.position, ""; color=Vec4f(1, 0.85, 0.2, 1))   # provisional marker
        return nothing
    end
    p = v.pending::Vec3f
    v.pending = nothing
    undo!(a)                                                          # replace the marker
    res = measure!(a, p, hit.position)
    @info @sprintf("distance %.3f m   Δz %+.3f m   slope %.1f°", res...)
    return res
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
    dt = isnan(v.last_time) ? 0.0 : min(now - v.last_time, 0.1)
    v.last_time = now
    update!(v.controller, dt)
    v.player === nothing || tick!(v.player, dt)
    fw, fh = GLFW.GetFramebufferSize(v.window)
    st = render_frame!(v.renderer, v.controller.camera, fw, fh)
    GLFW.SwapBuffers(v.window)
    _title!(v, st, dt)
    return true
end


function _make_current_if_needed(v::Viewer)
    if CURRENT_CONTEXT[] != v.context
        GLFW.MakeContextCurrent(v.window)
        CURRENT_CONTEXT[] = v.context
    end
end

function _title!(v::Viewer, st, dt)
    dt > 0 && (v.fps = v.fps == 0 ? 1 / dt : 0.9 * v.fps + 0.1 / dt)
    t = time()
    t - v.title_time < 0.25 && return                # overlay text 4× per second
    v.title_time = t
    mode = v.controller.mode == OrbitMode ? "orbit" : "fly"
    stats = @sprintf("%s  %.0f fps  %s pts  %s tris  %s segs%s", mode, v.fps, _si(st.points),
                     _si(st.triangles), _si(st.segments), v.measuring ? "  [measure]" : "")
    lines = String[stats]
    v.player === nothing || push!(lines, status(v.player))
    v.renderer.hud = lines
    GLFW.SetWindowTitle(v.window, "SpatialRender  |  " * stats)
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
    v.open = false
    return
end
