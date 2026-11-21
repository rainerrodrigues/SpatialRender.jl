# Shared rover / LiDAR simulation used by examples/live.jl and examples/replay.jl:
# a heightfield with cylindrical pillars, a ray-cast 16-beam LiDAR, a wedge
# rover mesh and a looping path.

using SpatialRender
using SpatialRender: MeshVertex
using Random

terrain(x, y) = 0.8sin(x / 7) * cos(y / 9) + 0.15sin(x / 1.7 + y / 2.3)
pillars = [(12.0, 6.0, 1.2), (-8.0, 14.0, 1.6), (4.0, -15.0, 1.0), (-16.0, -6.0, 2.0)]

# Range along a ray from `o` in direction `d`: ray-march the heightfield and
# test the pillars (vertical cylinders), like a real LiDAR return.
function cast(o, d; maxr=40.0)
    best = maxr
    for (cx, cy, r) in pillars
        ox, oy = o[1] - cx, o[2] - cy
        a = d[1]^2 + d[2]^2; b = 2(ox * d[1] + oy * d[2]); c = ox^2 + oy^2 - r^2
        disc = b^2 - 4a * c
        if a > 1e-9 && disc >= 0
            t = (-b - sqrt(disc)) / 2a
            0 < t < best && o[3] + t * d[3] < terrain(cx, cy) + 6 && (best = t)
        end
    end
    t = 0.2
    while t < best
        p = o + t * d
        if p[3] <= terrain(p[1], p[2])
            return t
        end
        t += 0.15 + 0.02t
    end
    return best < maxr ? best : nothing
end

const BEAMS = range(deg2rad(-25), deg2rad(5), length=16)
function lidar_scan(pos, yaw, rng; azimuths=720)
    pts = Vec3f[]; inten = Float32[]
    for az in range(0, 2π, length=azimuths + 1)[1:end-1], el in BEAMS
        dl = Vec3d(cos(az) * cos(el), sin(az) * cos(el), sin(el))          # sensor frame
        dw = Vec3d(cos(yaw) * dl[1] - sin(yaw) * dl[2], sin(yaw) * dl[1] + cos(yaw) * dl[2], dl[3])
        r = cast(pos, dw)
        r === nothing && continue
        r += 0.02randn(rng)
        push!(pts, Vec3f(r * dl)); push!(inten, Float32(clamp(1.2 - r / 40, 0, 1)))
    end
    return pts, inten
end

function rover_mesh()
    # a simple wedge body, 2 × 1.2 × 0.6 m, nose along +X
    P = Vec3f[(-1, -0.6, 0), (1, -0.4, 0), (1, 0.4, 0), (-1, 0.6, 0),
              (-1, -0.6, 0.6), (0.6, -0.4, 0.5), (0.6, 0.4, 0.5), (-1, 0.6, 0.6)]
    F = [(1, 3, 2), (1, 4, 3), (5, 6, 7), (5, 7, 8), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6),
         (3, 4, 8), (3, 8, 7), (4, 1, 5), (4, 5, 8)]
    v = MeshVertex[]; idx = UInt32[]
    for f in F, k in f
        push!(v, MeshVertex(P[k], Vec3f(0, 0, 1))); push!(idx, UInt32(length(v) - 1))
    end
    m = TriMesh(v, idx; name="rover", color=Vec4f(0.95, 0.55, 0.15, 1))
    return compute_normals!(m)
end

path(t) = (r = 10 + 3sin(t / 4); Vec3d(r * cos(t / 6), r * sin(t / 6), 0))
function pose_at(t)
    p = path(t); q = path(t + 0.05)
    yaw = atan(q[2] - p[2], q[1] - p[1])
    return Vec3d(p[1], p[2], terrain(p[1], p[2]) + 0.1), yaw
end
