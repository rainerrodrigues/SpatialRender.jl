# Replay a ROS 2 bag (MCAP or rosbag2 directory).
#
#   julia -t auto --project=. examples/replay.jl my_bag/            # your recording
#   julia -t auto --project=. examples/replay.jl                    # records a demo bag first
#   julia -t auto --project=. examples/replay.jl demo.mcap shot.png # screenshot at t = 12 s
#
# Without arguments a 20 s demo bag is recorded from the rover simulation with
# the topics a real robot publishes: /tf_static (base_link → lidar),
# /tf (map → odom → base_link), /odom, /lidar/points (PointCloud2),
# /plan (Path) and /gps/fix (NavSatFix).

include(joinpath(@__DIR__, "sim_rover.jl"))

const LAT0, LON0 = 47.3769, 8.5417

"Record the rover simulation as a ROS 2 MCAP bag."
function record_demo_bag(file; seconds=20.0)
    w = MCAPWriter(file; compression="zstd")
    sid(name) = add_schema!(w, name, "ros2msg", Vector{UInt8}(SpatialRender.ROS2_SCHEMAS[name]))
    ch_tf = add_channel!(w, "/tf", sid("tf2_msgs/msg/TFMessage"))
    ch_static = add_channel!(w, "/tf_static", sid("tf2_msgs/msg/TFMessage"))
    ch_odom = add_channel!(w, "/odom", sid("nav_msgs/msg/Odometry"))
    ch_cloud = add_channel!(w, "/lidar/points", sid("sensor_msgs/msg/PointCloud2"))
    ch_plan = add_channel!(w, "/plan", sid("nav_msgs/msg/Path"))
    ch_fix = add_channel!(w, "/gps/fix", sid("sensor_msgs/msg/NavSatFix"))
    ns(t) = round(Int64, (1.7e9 + t) * 1e9)                  # stamps around a realistic epoch
    yawq(y) = Quat(Vec3d(0, 0, 1), y)
    # map → odom: odometry drifted 0.8 m and 3° from the map at start
    map_odom = StampedTransform(ns(0), "map", "odom", Vec3d(0.8, -0.5, 0), yawq(deg2rad(3)))
    odom_from_map = inv(Rigid(map_odom.translation, map_odom.rotation))
    write_message!(w, ch_static, ns(0), encode_tf([StampedTransform(ns(0), "base_link", "lidar",
                                                                      Vec3d(0, 0, 1.0), Quat())]))
    enu = ENUFrame(LAT0, LON0, 0.0)
    rng = Xoshiro(7)
    for k in 0:round(Int, seconds * 50)                       # 50 Hz master clock
        t = k / 50
        pos, yaw = pose_at(t)                                  # ground truth in the map frame
        base = odom_from_map * Rigid(pos, yawq(yaw))           # pose in odom
        q = Quat(base.R)
        tfm = [StampedTransform(ns(t), "odom", "base_link", base.t, q)]
        k % 50 == 0 && push!(tfm, StampedTransform(ns(t), "map", "odom", map_odom.translation, map_odom.rotation))
        write_message!(w, ch_tf, ns(t), encode_tf(tfm))
        if k % 2 == 0                                           # 25 Hz odometry
            write_message!(w, ch_odom, ns(t), encode_odometry(StampedTransform(ns(t), "odom", "base_link", base.t, q)))
        end
        if k % 5 == 0                                           # 10 Hz LiDAR, in the sensor frame
            pts, inten = lidar_scan(pos + Vec3d(0, 0, 1.0), yaw, rng; azimuths=540)
            ring = [mod(i - 1, 16) for i in eachindex(pts)]
            write_message!(w, ch_cloud, ns(t), encode_pointcloud2(ns(t), "lidar", pts; intensity=inten .* 255, ring))
        end
        if k % 10 == 0                                          # 5 Hz GNSS with 0.5 m noise
            g = to_world(enu, pos + Vec3d(0.5randn(rng), 0.5randn(rng), 0))
            write_message!(w, ch_fix, ns(t), encode_navsatfix(ns(t), "gps", g[2], g[1], 400.0 + g[3]))
        end
        if k % 50 == 0                                          # 1 Hz plan: the next 8 s of path
            plan = [Vec3d(path(t + s)[1], path(t + s)[2], terrain(path(t + s)[1], path(t + s)[2]) + 0.1) for s in 0:0.25:8]
            write_message!(w, ch_plan, ns(t), encode_path(ns(t), "map", plan))
        end
    end
    close(w)
    return file
end

if abspath(PROGRAM_FILE) == @__FILE__        # run as a script (not when included)
    bag = isempty(ARGS) ? record_demo_bag(joinpath(tempdir(), "spatialrender_demo.mcap")) : ARGS[1]
    if length(ARGS) > 1
        p = open_bag(bag)
        display(p.bag)
        seek!(p, 12.0)
        cam = Camera(position=Vec3d(-28, -30, 24), target=Vec3d(0, 0, 0))
        screenshot(ARGS[2], p.scene; width=1280, height=720, camera=cam)
        @info "wrote $(ARGS[2]) at t = $(current_time(p)) s"
    else
        replay(bag; history=4.0)
    end
end
