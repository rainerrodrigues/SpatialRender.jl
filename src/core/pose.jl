# Layer poses and thread-safe updates.
#
# Every layer stores its vertices in its own frame plus a 4×4 pose (`model`)
# into the scene frame, so a robot model, a sensor rig or a scan can move
# without touching its vertex data: a pose change is one uniform upload.
#
# Producers (sensor callbacks, simulation tasks, other threads) change
# layers through `set_pose!`, `touch!`, `push_scan!`, `add!` and `delete!`,
# which all take `UPDATE_LOCK`; the renderer holds the same lock while it
# syncs and draws a frame, so it never sees a half-written pose or buffer.

"Serialises scene / layer updates against the render loop."
const UPDATE_LOCK = ReentrantLock()

"""
    with_update_lock(f)

Run `f()` holding the scene update lock: use it to change several layer
fields at once (e.g. color and visibility) atomically with respect to the
renderer.
"""
with_update_lock(f) = lock(f, UPDATE_LOCK)

"""
    pose_matrix(position, orientation=Quat(); scale=1) -> Mat4f

Homogeneous transform `T * R * S`.
"""
function pose_matrix(position, q::Quat=Quat(); scale::Real=1.0)
    R = rotation_matrix(q) * scale
    p = Vec3d(position)
    Mat4f(R[1, 1], R[2, 1], R[3, 1], 0,
          R[1, 2], R[2, 2], R[3, 2], 0,
          R[1, 3], R[2, 3], R[3, 3], 0,
          p[1], p[2], p[3], 1)
end

"""
    set_pose!(layer, position, orientation=Quat(); scale=1) -> layer
    set_pose!(layer, M::AbstractMatrix) -> layer

Place `layer` in the scene: its vertices are mapped by the pose into the
scene frame. Thread-safe; takes effect on the next frame.

```julia
robot = add!(scene, load_stl("rover.stl"))
for (p, yaw) in telemetry
    set_pose!(robot, p, Quat(Vec3d(0, 0, 1), yaw))
end
```
"""
set_pose!(l::Layer, position, q::Quat=Quat(); scale::Real=1.0) =
    set_pose!(l, pose_matrix(position, q; scale))
function set_pose!(l::Layer, M::AbstractMatrix)
    size(M) == (4, 4) || throw(ArgumentError("a pose must be a 4×4 matrix"))
    m = Mat4f(M)
    lock(() -> (l.model = m), UPDATE_LOCK)
    return l
end

"Current pose of a layer (layer → scene frame)."
pose(l::Layer) = l.model

"""
    touch!(layer) -> layer

Mark the layer's contents (vertices, indices, polylines) as changed after
editing them in place; the renderer re-uploads it on the next frame.
"""
touch!(l::Layer) = (lock(() -> (l.rev += 1), UPDATE_LOCK); l)

"Apply a 4×4 affine transform to a point."
@inline transform_point(M::Mat4f, p) =
    (q = M * SVector(Float32(p[1]), Float32(p[2]), Float32(p[3]), 1f0); Vec3f(q[1], q[2], q[3]))

"Axis-aligned bounds of a transformed box (all eight corners)."
function transform_box(M::Mat4f, b::AABB)
    isempty(b) && return b
    M == IDENTITY4 && return b
    out = AABB()
    for x in (b.lo[1], b.hi[1]), y in (b.lo[2], b.hi[2]), z in (b.lo[3], b.hi[3])
        out = union(out, transform_point(M, Vec3f(x, y, z)))
    end
    return out
end

"Bounds of a layer in the scene frame (its pose applied)."
world_bounds(l::Layer) = transform_box(l.model, bounds(l))

"Inverse of an affine pose (rotation · scale + translation)."
function inverse_pose(M::Mat4f)
    A = SMatrix{3, 3, Float32, 9}(M[1, 1], M[2, 1], M[3, 1], M[1, 2], M[2, 2], M[3, 2], M[1, 3], M[2, 3], M[3, 3])
    Ai = inv(A)
    t = -(Ai * Vec3f(M[1, 4], M[2, 4], M[3, 4]))
    Mat4f(Ai[1, 1], Ai[2, 1], Ai[3, 1], 0, Ai[1, 2], Ai[2, 2], Ai[3, 2], 0,
          Ai[1, 3], Ai[2, 3], Ai[3, 3], 0, t[1], t[2], t[3], 1)
end

"Upper-left 3×3 inverse-transpose: transforms normals correctly under scaling."
function normal_matrix(M::Mat4f)
    A = SMatrix{3, 3, Float32, 9}(M[1, 1], M[2, 1], M[3, 1], M[1, 2], M[2, 2], M[3, 2], M[1, 3], M[2, 3], M[3, 3])
    return SMatrix{3, 3, Float32, 9}(inv(A)')
end
