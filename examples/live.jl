# Live data: a rover drives a loop while a simulated 16-beam LiDAR streams
# scans into a ring buffer from a background task. The rover mesh moves by
# pose only (no re-upload); each scan is placed with the pose it was taken at.
#
#   julia -t auto --project=. examples/live.jl            # interactive
#   julia -t auto --project=. examples/live.jl live.png   # screenshot after 15 s simulated

include(joinpath(@__DIR__, "sim_rover.jl"))

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
