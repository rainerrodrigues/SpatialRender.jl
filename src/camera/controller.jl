# Interaction state machine. Window-system independent: the GLFW callbacks
# in the viewer translate events into these calls, and tests drive them
# directly.

@enum CameraMode OrbitMode FlyMode
@enum DragState IDLE ORBITING PANNING LOOKING

"Keyboard / mouse state shared between callbacks and the frame update."
mutable struct InputState
    held::Set{Symbol}           # held keys, e.g. :w, :shift
    buttons::NTuple{3, Bool}    # left, right, middle
    cursor::Vec2f
    last_cursor::Vec2f
    scroll::Float64             # accumulated since last update
    last_click_time::Float64
end
InputState() = InputState(Set{Symbol}(), (false, false, false), Vec2f(0, 0), Vec2f(0, 0), 0.0, -Inf)

"""
    CameraController(camera; mode=OrbitMode)

Maps input to camera motion.

| input                         | orbit mode            | fly mode                  |
|-------------------------------|-----------------------|---------------------------|
| left drag                     | orbit about pivot     | look around               |
| right / middle / shift-left   | pan                   | pan                       |
| scroll                        | zoom to pivot         | change fly speed          |
| W A S D, Q E                  | —                     | move, down / up           |
| shift (held)                  | —                     | move 4× faster            |
| Tab                           | switch to fly         | switch to orbit           |
| F                             | frame scene           | frame scene               |
| double-click                  | pivot on picked point | pivot on picked point     |
"""
mutable struct CameraController
    camera::Camera
    mode::CameraMode
    state::DragState
    input::InputState
    rotate_speed::Float64       # radians per pixel
    fly_speed::Float64          # metres per second
    viewport::NTuple{2, Int}
    dirty::Bool                 # camera changed since last frame
end
CameraController(cam::Camera; mode::CameraMode=OrbitMode) =
    CameraController(cam, mode, IDLE, InputState(), 0.005, 5.0, (1280, 800), true)

function set_viewport!(ctl::CameraController, w::Integer, h::Integer)
    ctl.viewport = (max(w, 1), max(h, 1))
    ctl.camera.aspect = ctl.viewport[1] / ctl.viewport[2]
    ctl.dirty = true
end

function toggle_mode!(ctl::CameraController)
    ctl.mode = ctl.mode == OrbitMode ? FlyMode : OrbitMode
    ctl.state = IDLE
    ctl.dirty = true
    return ctl.mode
end

"""
    mouse_button!(ctl, button, pressed; shift=false, time=0) -> Symbol

`button` is `:left`, `:right` or `:middle`. Returns `:double_click` on a
second left press within 0.3 s (the viewer then picks a new pivot),
otherwise `:none`.
"""
function mouse_button!(ctl::CameraController, button::Symbol, pressed::Bool; shift::Bool=false, time::Real=0.0)
    inp = ctl.input
    l, r, m = inp.buttons
    button == :left && (l = pressed)
    button == :right && (r = pressed)
    button == :middle && (m = pressed)
    inp.buttons = (l, r, m)
    inp.last_cursor = inp.cursor
    ev = :none
    if pressed
        if button == :left && !shift
            ctl.state = ctl.mode == OrbitMode ? ORBITING : LOOKING
            time - inp.last_click_time < 0.3 && (ev = :double_click)
            inp.last_click_time = time
        else
            ctl.state = PANNING
        end
    elseif !(l || r || m)
        ctl.state = IDLE
    end
    return ev
end

"Cursor moved to `(x, y)` pixels."
function cursor!(ctl::CameraController, x::Real, y::Real)
    inp = ctl.input
    inp.cursor = Vec2f(x, y)
    d = inp.cursor - inp.last_cursor
    inp.last_cursor = inp.cursor
    (d[1] == 0 && d[2] == 0) && return ctl
    c = ctl.camera
    s = ctl.rotate_speed
    if ctl.state == ORBITING
        orbit!(c, -s * d[1], -s * d[2])
    elseif ctl.state == LOOKING
        fly_look!(c, -s * d[1], -s * d[2])
    elseif ctl.state == PANNING
        pan!(c, d[1], d[2], ctl.viewport[2])
    else
        return ctl
    end
    ctl.dirty = true
    return ctl
end

"Scroll wheel by `dy` notches (positive = away from the user / zoom in)."
function scroll!(ctl::CameraController, dy::Real)
    if ctl.mode == OrbitMode
        dolly!(ctl.camera, dy)
    else
        ctl.fly_speed = clamp(ctl.fly_speed * 1.2^dy, 0.01, 1e5)
    end
    ctl.dirty = true
    return ctl
end

"Key press / release (`key` as a lowercase Symbol: `:w`, `:shift`, `:tab`, …)."
function key!(ctl::CameraController, key::Symbol, pressed::Bool)
    pressed ? push!(ctl.input.held, key) : delete!(ctl.input.held, key)
    pressed && key == :tab && toggle_mode!(ctl)
    return ctl
end

"""
    update!(ctl, dt) -> Bool

Apply held-key fly motion for a frame of `dt` seconds. Returns whether the
camera changed since the last call (and clears the flag).
"""
function update!(ctl::CameraController, dt::Real)
    h = ctl.input.held
    if ctl.mode == FlyMode && !isempty(h)
        v = Vec3d((:d in h) - (:a in h), (:e in h) - (:q in h), (:w in h) - (:s in h))
        if any(!iszero, v)
            sp = ctl.fly_speed * ((:shift in h) ? 4.0 : 1.0) * dt
            fly_move!(ctl.camera, sp * normalize(v))
            ctl.dirty = true
        end
    end
    d = ctl.dirty
    ctl.dirty = false
    return d
end

"Make `p` the orbit pivot without moving the camera (turns it to face `p`)."
function set_pivot!(ctl::CameraController, p)
    look_at!(ctl.camera, Vec3d(p))
    ctl.dirty = true
    return ctl
end
