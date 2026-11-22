# Precompile workload: run a small, representative slice of the package while it
# is being precompiled, so the compiled code is cached and first use is fast.
# Nothing here needs a display or GPU: the OpenGL paths can't run at install
# time, so their main methods are compiled from signatures instead.

using PrecompileTools: @setup_workload, @compile_workload

@setup_workload begin
    dir = mktempdir()
    pts = [Vec3f(0.1i, 0.05i % 7, 0.01i % 3) for i in 1:2_000]
    @compile_workload begin
        # point clouds: construction, LAS round trip, filters, index, queries
        pc = PointCloud(pts; intensity=fill(0.5, length(pts)), classification=fill(2, length(pts)))
        las = save_las(joinpath(dir, "p.las"), pc; frame=OffsetFrame(5e5, 4e6, 100.0))
        cloud, fr = load_las(las)
        statistical_outlier_mask(cloud); radius_outlier_mask(cloud; radius=0.5)
        voxel_downsample(cloud, 0.5); ground_mask(cloud; threshold=0.2)
        (StatisticalOutlierFilter() |> VoxelDownsample(0.3))(cloud)
        ix = build_point_index(cloud; chunk_size=256)
        nearest_points(ix, Vec3f(1, 1, 0), 4); points_in_radius(ix, Vec3f(1, 1, 0), 1.0)
        cam = Camera(position=Vec3d(-5, -5, 5), target=Vec3d(0, 0, 0))
        f = frustum_from_matrix(view_projection(cam))
        cull!(ix, f; eye=Vec3f(cam.position), pixels_per_radian=500.0)
        raycast([(cloud, ix)], screen_ray(cam, 32, 24, 64, 48))
        build_octree(cloud, joinpath(dir, "oct"); node_capacity=500)
        # meshes, lines, frames
        tri = TriMesh([MeshVertex(Vec3f(0, 0, 0), Vec3f(0, 0, 1)), MeshVertex(Vec3f(1, 0, 0), Vec3f(0, 0, 1)),
                       MeshVertex(Vec3f(0, 1, 0), Vec3f(0, 0, 1))], UInt32[0, 1, 2])
        load_stl(save_stl(joinpath(dir, "t.stl"), tri))
        raycast(build_bvh(tri), Ray(Vec3f(0.2, 0.2, 1), Vec3f(0, 0, -1)))
        trajectory([Vec3d(i, 0, 0) for i in 1:5]; times=1.0:5.0, max_gap=2.0)
        enu = ENUFrame(47.0, 8.0); to_world(enu, to_local(enu, (8.001, 47.001, 0.0)))
        write(joinpath(dir, "g.geojson"), """{"type":"LineString","coordinates":[[8,47],[8.001,47.001]]}""")
        load_geojson(joinpath(dir, "g.geojson"))
        # scene, annotations, camera control, labels
        s = Scene(); add!(s, cloud); add!(s, tri)
        notes = add!(s, Annotations()); measure!(notes, Vec3f(0, 0, 0), Vec3f(1, 1, 1)); annotate!(notes, Vec3f(0, 0, 0), "a")
        label_segments(notes); hud_segments(["fps 60"])
        ctl = CameraController(cam); cursor!(ctl, 10, 10); mouse_button!(ctl, :left, true); cursor!(ctl, 20, 15)
        mouse_button!(ctl, :left, false); scroll!(ctl, 1); update!(ctl, 0.016)
        live = StreamingPointCloud(10_000); push_scan!(live, pts[1:100]; pose=(Vec3d(1, 0, 0), Quat()))
        # ROS 2 bag: write, index, decode, replay
        w = MCAPWriter(joinpath(dir, "b.mcap"))
        ctf = add_channel!(w, "/tf", add_schema!(w, "tf2_msgs/msg/TFMessage"))
        cpc = add_channel!(w, "/points", add_schema!(w, "sensor_msgs/msg/PointCloud2"))
        cod = add_channel!(w, "/odom", add_schema!(w, "nav_msgs/msg/Odometry"))
        for k in 0:5
            t = k * 100_000_000
            write_message!(w, ctf, t, encode_tf([StampedTransform(t, "map", "base_link", Vec3d(0.1k, 0, 0), Quat())]))
            write_message!(w, cpc, t, encode_pointcloud2(t, "base_link", pts[1:50]))
            write_message!(w, cod, t, encode_odometry(StampedTransform(t, "map", "base_link", Vec3d(0.1k, 0, 0), Quat())))
        end
        close(w)
        p = open_bag(joinpath(dir, "b.mcap")); seek!(p, 0.3); play!(p); tick!(p, 0.1); status(p)
        write_png(joinpath(dir, "i.png"), fill((0x10, 0x20, 0x30), 4, 4))
    end
    rm(dir; recursive=true, force=true)
end

# OpenGL paths: compile without running (no context exists at install time)
for sig in (
        Tuple{typeof(render_frame!), Renderer, Camera, Int, Int},
        Tuple{typeof(_render_frame!), Renderer, Camera, Int32, Int32},
        Tuple{typeof(sync!), Renderer},
        Tuple{typeof(upload), PointCloud},
        Tuple{typeof(upload), TriMesh},
        Tuple{typeof(upload), StreamingPointCloud},
        Tuple{typeof(upload), LineLayer, GLBuffer},
        Tuple{typeof(upload), Annotations, GLBuffer},
        Tuple{typeof(draw!), GPUPoints, PointCloud, GLProgram, FrameContext, RenderSettings},
        Tuple{typeof(draw!), GPUMesh, TriMesh, GLProgram, FrameContext, RenderSettings},
        Tuple{typeof(draw!), GPULines, LineLayer, GLProgram, FrameContext, RenderSettings},
        Tuple{typeof(draw!), GPUStream, StreamingPointCloud, GLProgram, FrameContext, RenderSettings},
        Tuple{typeof(step!), Viewer},
        Tuple{typeof(_on_key), Viewer, GLFW.Key, GLFW.Action},
        Tuple{typeof(read_framebuffer), Int, Int},
    )
    precompile(sig) || @debug "precompile failed for $sig"
end
