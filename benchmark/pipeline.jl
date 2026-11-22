# Core pipeline throughput on synthetic data (no GPU or display needed).
#
#   julia -t auto --project benchmark/pipeline.jl [npoints]
#
# CI runs this and puts the table in the job summary.

using SpatialRender
using Printf, Random, Statistics

const SR = SpatialRender
n = length(ARGS) > 0 ? parse(Int, ARGS[1]) : 2_000_000
dir = mktempdir()
rng = Xoshiro(1)

terrain(x, y) = 4sin(x / 40) * cos(y / 55) + 0.01x
pts = [(x = 400rand(rng) - 200; y = 400rand(rng) - 200; Vec3f(x, y, terrain(x, y) + 0.03randn(rng))) for _ in 1:n]
cloud = PointCloud(pts; intensity=rand(rng, n), classification=rand(rng, 0:9, n))

rows = Tuple{String, Float64, String}[]
function bench(f, name; repeat=1, per=nothing)
    f()                                            # compile
    GC.gc()
    t = minimum(@elapsed(f()) for _ in 1:repeat)
    rate = per === nothing ? "" : @sprintf("%.1f M/s", per / t / 1e6)
    push!(rows, (name, t, rate))
end

las = joinpath(dir, "bench.las")
save_las(las, cloud; frame=OffsetFrame(5e5, 4e6, 100.0))
bench(() -> load_las(las), "load_las"; repeat=3, per=n)
bench(() -> build_point_index(SR.subset(cloud, 1:n)), "build_point_index"; per=n)
ix = build_point_index(SR.subset(cloud, 1:n))
cam = Camera(position=Vec3d(-150, -160, 90), target=Vec3d(0, 0, 0), aspect=16 / 9); cam.far = 2000
fr = frustum_from_matrix(view_projection(cam))
bench(() -> for _ in 1:1000; cull!(ix, fr; eye=Vec3f(cam.position), pixels_per_radian=900.0); end,
      "cull! ×1000"; repeat=3)
bench(() -> statistical_outlier_mask(cloud), "statistical outlier filter"; per=n)
bench(() -> radius_outlier_mask(cloud; radius=1.0), "radius outlier filter"; per=n)
bench(() -> voxel_downsample(cloud, 0.5), "voxel_downsample"; per=n)
bench(() -> build_octree(las, joinpath(dir, "oct_$(rand(UInt32))"); bucket_points=500_000), "build_octree"; per=n)

# ROS 2 bag: write and index 100 PointCloud2 messages of 50k points
bag = joinpath(dir, "bench.mcap")
scan = pts[1:50_000]
bench(function ()
    w = MCAPWriter(bag)
    ch = add_channel!(w, "/points", add_schema!(w, "sensor_msgs/msg/PointCloud2"))
    for k in 1:100
        write_message!(w, ch, k * 100_000_000, encode_pointcloud2(k * 100_000_000, "lidar", scan))
    end
    close(w)
end, "MCAP write (100 × 50k pts, zstd)")
bench(function ()
    f = open_mcap(bag)
    for m in f.messages
        decode_pointcloud2(message_data(f, m))
    end
end, "MCAP read + decode"; per=100 * 50_000)

println(@sprintf("SpatialRender pipeline, %d points, %d threads, Julia %s", n, Threads.nthreads(), VERSION))
println(rpad("stage", 34), lpad("time", 10), lpad("rate", 13))
for (name, t, rate) in rows
    println(rpad(name, 34), lpad(@sprintf("%.3f s", t), 10), lpad(rate, 13))
end
