# Quaternion camera. World frame: X east / right, Y north / forward, Z up.
# Camera frame (OpenGL convention): looks down -Z, +Y up, +X right.

"""
    Camera(; position, target, fov=50°, near=0.05, far=1e4)

Perspective (or orthographic) camera with a quaternion orientation.
`distance` is the distance to the pivot used by orbit, pan and zoom.
"""
mutable struct Camera
    position::Vec3d
    orientation::Quat       # camera → world
    distance::Float64       # to the orbit pivot, along the view direction
    fov::Float64            # vertical, radians
    near::Float64
    far::Float64
    ortho::Bool
    aspect::Float64
end

function Camera(; position=Vec3d(10, -10, 8), target=Vec3d(0, 0, 0), fov=deg2rad(50.0),
                near=0.05, far=1e4, aspect=16 / 9)
    c = Camera(Vec3d(position), Quat(), 1.0, fov, near, far, false, aspect)
    look_at!(c, Vec3d(target))
    return c
end

forward(c::Camera) = rotate(c.orientation, Vec3d(0, 0, -1))
right(c::Camera) = rotate(c.orientation, Vec3d(1, 0, 0))
up(c::Camera) = rotate(c.orientation, Vec3d(0, 1, 0))
target(c::Camera) = c.position + c.distance * forward(c)

"""
    look_at!(cam, target; up=Z) -> cam

Point the camera at `target`, keeping the horizon level (world Z up).
"""
function look_at!(c::Camera, tgt; worldup=Vec3d(0, 0, 1))
    f = Vec3d(tgt) - c.position
    d = norm(f)
    d < 1e-12 && return c
    f /= d
    r = cross(f, worldup)
    norm(r) < 1e-9 && (r = cross(f, Vec3d(0, 1, 0)))  # looking straight up/down
    r = normalize(r)
    u = cross(r, f)
    c.orientation = Quat(SMatrix{3, 3, Float64, 9}(r..., u..., (-f)...))
    c.distance = d
    return c
end

"World → camera matrix (Float32, column-major, ready for `glUniformMatrix4fv`)."
function view_matrix(c::Camera)
    R = rotation_matrix(c.orientation)'   # world → camera
    t = -(R * c.position)
    Mat4f(R[1, 1], R[2, 1], R[3, 1], 0,
          R[1, 2], R[2, 2], R[3, 2], 0,
          R[1, 3], R[2, 3], R[3, 3], 0,
          t[1], t[2], t[3], 1)
end

"Camera → clip matrix, OpenGL convention (NDC z ∈ [-1, 1])."
function projection_matrix(c::Camera)
    n, f, a = c.near, c.far, c.aspect
    if c.ortho
        h = c.distance * tan(c.fov / 2)
        w = h * a
        return Mat4f(1 / w, 0, 0, 0,  0, 1 / h, 0, 0,  0, 0, -2 / (f - n), 0,  0, 0, -(f + n) / (f - n), 1)
    end
    t = 1 / tan(c.fov / 2)
    return Mat4f(t / a, 0, 0, 0,  0, t, 0, 0,  0, 0, -(f + n) / (f - n), -1,  0, 0, -2f * n / (f - n), 0)
end

view_projection(c::Camera) = projection_matrix(c) * view_matrix(c)

"Pixels per radian of view angle for a viewport `height` pixels tall (for LOD and picking)."
pixels_per_radian(c::Camera, height) = height / c.fov

"""
    orbit!(cam, dyaw, dpitch)

Turntable orbit about the pivot: yaw about world Z, pitch about the camera's
right axis, clamped short of the poles so the view never flips.
"""
function orbit!(c::Camera, dyaw::Real, dpitch::Real)
    tgt = target(c)
    q = Quat(Vec3d(0, 0, 1), dyaw) * c.orientation
    # current elevation of the view direction
    f = rotate(q, Vec3d(0, 0, -1))
    elev = asin(clamp(f[3], -1, 1))
    lim = deg2rad(89.0)
    dpitch = clamp(dpitch, -lim - elev, lim - elev)
    r = rotate(q, Vec3d(1, 0, 0))
    q = normalize(Quat(r, dpitch) * q)
    c.orientation = q
    c.position = tgt - c.distance * rotate(q, Vec3d(0, 0, -1))
    return c
end

"""
    pan!(cam, dx, dy, viewport_height)

Move camera and pivot parallel to the image plane by `dx`, `dy` pixels,
so the point under the cursor at the pivot depth follows the mouse.
"""
function pan!(c::Camera, dx::Real, dy::Real, height::Real)
    s = 2 * c.distance * tan(c.fov / 2) / height
    c.position += s * (-dx * right(c) + dy * up(c))
    return c
end

"""
    dolly!(cam, steps; min_distance=1e-3)

Move toward (`steps > 0`) or away from the pivot geometrically (10% per step).
"""
function dolly!(c::Camera, steps::Real; min_distance=1e-3)
    tgt = target(c)
    c.distance = max(c.distance * 0.9^steps, min_distance)
    c.position = tgt - c.distance * forward(c)
    return c
end

"""
    fly_move!(cam, v)

Move by `v` in camera axes (`v[1]` right, `v[2]` up, `v[3]` forward), metres.
"""
function fly_move!(c::Camera, v)
    c.position += v[1] * right(c) + v[2] * Vec3d(0, 0, 1) + v[3] * forward(c)
    return c
end

"""
    fly_look!(cam, dyaw, dpitch)

Rotate the view in place (first-person mouse look), horizon kept level.
"""
function fly_look!(c::Camera, dyaw::Real, dpitch::Real)
    q = Quat(Vec3d(0, 0, 1), dyaw) * c.orientation
    f = rotate(q, Vec3d(0, 0, -1))
    elev = asin(clamp(f[3], -1, 1))
    lim = deg2rad(89.0)
    dpitch = clamp(dpitch, -lim - elev, lim - elev)
    c.orientation = normalize(Quat(rotate(q, Vec3d(1, 0, 0)), dpitch) * q)
    return c
end

"""
    frame!(cam, box; margin=1.0)

Keep the view direction and move so `box` fills the view; the pivot moves
to the box centre. Also sets near / far planes for the box.
"""
function frame!(c::Camera, b::AABB; margin=1.0)
    isempty(b) && return c
    r = max(Float64(radius(b)), 1e-3)
    half = min(c.fov, 2 * atan(tan(c.fov / 2) * c.aspect)) / 2
    c.distance = margin * r / sin(half)
    c.position = Vec3d(center(b)) - c.distance * forward(c)
    fit_clip!(c, b)
    return c
end

"Choose near/far planes that enclose `box` with good depth precision."
function fit_clip!(c::Camera, b::AABB)
    isempty(b) && return c
    r = Float64(radius(b))
    d = norm(Vec3d(center(b)) - c.position)
    c.far = max(d + 2r, 10.0)
    c.near = clamp((d - r) * 0.5, c.far * 1e-5, max(c.far * 1e-2, 1e-3))
    return c
end

"""
    screen_ray(cam, x, y, width, height) -> Ray

Ray through pixel `(x, y)` (origin top-left, as GLFW reports the cursor).
"""
function screen_ray(c::Camera, x::Real, y::Real, w::Real, h::Real)
    nx = 2x / w - 1
    ny = 1 - 2y / h
    th = tan(c.fov / 2)
    if c.ortho
        hh = c.distance * th
        o = c.position + nx * hh * c.aspect * right(c) + ny * hh * up(c)
        return Ray(o, forward(c))
    end
    d = nx * th * c.aspect * right(c) + ny * th * up(c) + forward(c)
    return Ray(c.position, d)
end
