# Live data and poses

```julia
scene = Scene()
rover = add!(scene, load_stl("rover.stl"))
lidar = add!(scene, StreamingPointCloud(2_000_000; max_age = 100))   # newest 2M points
v = open_viewer(scene; block = false)

Threads.@spawn for msg in sensor_feed           # any thread: updates are locked
    set_pose!(rover, msg.position, msg.orientation)
    push_scan!(lidar, msg.points; pose = (msg.sensor_position, msg.orientation))
end
run!(v)
```

`examples/live.jl` simulates this with a ray-cast 16-beam LiDAR at 10 Hz.
Use `touch!(layer)` after editing a layer's vertices in place.

```@docs; canonical=false
set_pose!
pose_matrix
touch!
StreamingPointCloud
push_scan!
```
