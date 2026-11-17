# Live data: a rover drives a loop while a simulated 16-beam LiDAR streams
# scans into a ring buffer from a background task. The rover mesh moves by
# pose only (no re-upload); each scan is placed with the pose it was taken at.
#
#   julia -t auto --project=. examples/live.jl            # interactive
#   julia -t auto --project=. examples/live.jl live.png   # screenshot after 15 s simulated

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

scene = Scene()
rover = add!(scene, rover_mesh())
live = add!(scene, StreamingPointCloud(1_500_000; name="lidar", point_size=2, max_age=120))
trail = add!(scene, trajectory([path(t) + Vec3d(0, 0, terrain(path(t)[1], path(t)[2]) + 0.05)
                                for t in 0:0.2:6π * 2]; colormap=:solid, color=Vec4f(1, 1, 1, 0.35), width=1.5))
for (cx, cy, r) in pillars           # ground-truth pillar outlines, for comparison
    add!(scene, LineLayer([Polyline([Vec3f(cx + r * cos(a), cy + r * sin(a), terrain(cx, cy) + 6)
                                     for a in range(0, 2π, length=40)], Vec4f(0.3, 0.9, 1, 0.8))];
                          name="pillar", width=2))
end

function step_sim!(t, rng)
    pos, yaw = pose_at(t)
    set_pose!(rover, pos, Quat(Vec3d(0, 0, 1), yaw))
    sensor = pos + Vec3d(0, 0, 1.0)
    pts, inten = lidar_scan(sensor, yaw, rng)
    push_scan!(live, pts; intensity=inten, pose=(sensor, Quat(Vec3d(0, 0, 1), yaw)))
end

rng = Xoshiro(3)
if isempty(ARGS)
    v = open_viewer(scene; title="SpatialRender live", block=false)
    producer = Threads.@spawn begin                # 10 Hz sensor feed on another thread
        t = 0.0
        while v.open
            step_sim!(t, rng); t += 0.1
            sleep(0.1)
        end
    end
    run!(v)
    wait(producer)
else
    for t in 0:0.1:15
        step_sim!(t, rng)
    end
    cam = Camera(position=Vec3d(-30, -32, 26), target=Vec3d(0, 0, 0))
    screenshot(ARGS[1], scene; width=1280, height=720, camera=cam)
    @info "wrote $(ARGS[1]): $(length(live)) points from $(live.seq) scans"
end
