# Minimal unit quaternion for camera orientation (no gimbal lock).

"""
    Quat(w, x, y, z)

Rotation quaternion `w + xi + yj + zk`.
"""
struct Quat
    w::Float64
    x::Float64
    y::Float64
    z::Float64
end
Quat() = Quat(1, 0, 0, 0)

"Rotation of `θ` radians about `axis`."
function Quat(axis::AbstractVector, θ::Real)
    a = normalize(Vec3d(axis))
    s, c = sincos(θ / 2)
    Quat(c, s * a[1], s * a[2], s * a[3])
end

Base.:*(a::Quat, b::Quat) = Quat(
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w)
Base.conj(q::Quat) = Quat(q.w, -q.x, -q.y, -q.z)
LinearAlgebra.norm(q::Quat) = sqrt(q.w^2 + q.x^2 + q.y^2 + q.z^2)
LinearAlgebra.normalize(q::Quat) = (n = norm(q); Quat(q.w / n, q.x / n, q.y / n, q.z / n))

"Rotate a vector."
function rotate(q::Quat, v::AbstractVector)
    u = Vec3d(q.x, q.y, q.z)
    v = Vec3d(v)
    t = 2 * cross(u, v)
    return v + q.w * t + cross(u, t)
end
Base.:*(q::Quat, v::SVector{3}) = rotate(q, v)

"3×3 rotation matrix."
function rotation_matrix(q::Quat)
    w, x, y, z = q.w, q.x, q.y, q.z
    SMatrix{3, 3, Float64, 9}(1 - 2(y^2 + z^2), 2(x * y + w * z), 2(x * z - w * y),
                              2(x * y - w * z), 1 - 2(x^2 + z^2), 2(y * z + w * x),
                              2(x * z + w * y), 2(y * z - w * x), 1 - 2(x^2 + y^2))
end

"Quaternion from an orthonormal rotation matrix (Shepperd's method)."
function Quat(R::SMatrix{3, 3})
    t = R[1, 1] + R[2, 2] + R[3, 3]
    if t > 0
        s = sqrt(t + 1) * 2
        return normalize(Quat(s / 4, (R[3, 2] - R[2, 3]) / s, (R[1, 3] - R[3, 1]) / s, (R[2, 1] - R[1, 2]) / s))
    elseif R[1, 1] > R[2, 2] && R[1, 1] > R[3, 3]
        s = sqrt(1 + R[1, 1] - R[2, 2] - R[3, 3]) * 2
        return normalize(Quat((R[3, 2] - R[2, 3]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[1, 3] + R[3, 1]) / s))
    elseif R[2, 2] > R[3, 3]
        s = sqrt(1 + R[2, 2] - R[1, 1] - R[3, 3]) * 2
        return normalize(Quat((R[1, 3] - R[3, 1]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[2, 3] + R[3, 2]) / s))
    else
        s = sqrt(1 + R[3, 3] - R[1, 1] - R[2, 2]) * 2
        return normalize(Quat((R[2, 1] - R[1, 2]) / s, (R[1, 3] + R[3, 1]) / s, (R[2, 3] + R[3, 2]) / s, s / 4))
    end
end
