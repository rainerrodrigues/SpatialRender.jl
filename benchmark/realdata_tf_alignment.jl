# Real-data check of TF placement with /tf_static: LiDAR scans placed through
# map <- body (/tf) <- base_link <- os_sensor <- os_lidar (/tf_static) are compared
# with the dataset's own registered map (0.1 m voxels).
#
# Data: "Forest walks 2026-08-12" (Esfandiyar & Belter, CC BY 4.0),
# https://doi.org/10.5281/zenodo.24378954 - download walks_2026-08-12_companion.zip
# (2.1 GB) and extract sample/, map/ and metadata/. Then:
#
#   julia --project -t auto benchmark/realdata_tf_alignment.jl \
#       companion/sample/walks_2026-08-12_sample60s companion
#
# Result on 2026-11-22: median 4.3 cm, p90 6.0 cm, 99.6 % of points within 10 cm
# (the map's voxel size limits this); without /tf_static: median 20 cm, 30 %.
using SpatialRender, Statistics, Printf, LinearAlgebra, Mmap
using SpatialRender: Rigid, lookup_transform, decode_pointcloud2, message_data

function main(sample, comp)
p = open_bag(sample); bag = p.bag; tree = p.tree
ch = only(k for (k, c) in bag.channels if c.topic == "/ouster/points")
msgs = [m for m in bag.messages if m.channel == ch]

# area walked in the sample, padded by the analysed range
pos = [lookup_transform(tree, "map", "body", m.log_time).t for m in msgs]
lo = reduce((a, b) -> min.(a, b), pos) .- 26; hi = reduce((a, b) -> max.(a, b), pos) .+ 26

# reference map (binary PCD: x y z intensity, Float32), cropped to that area
path = joinpath(comp, "map", "walks_2026-08-12_map_local_enu.pcd")
io = open(path); hdr = 0; n = 0
while true
    l = readline(io); hdr = position(io)
    startswith(l, "POINTS") && (n = parse(Int, split(l)[2]))
    startswith(l, "DATA") && break
end
bytes = Mmap.mmap(io, Vector{UInt8}, 16n, hdr)
raw = (ntuple(j -> ltoh(reinterpret(Float32, bytes[16i-15+4(j-1):16i-12+4(j-1)])[1]), 4) for i in 1:n)
ref = Vec3f[]
for (x, y, z, _) in raw
    (lo[1] <= x <= hi[1] && lo[2] <= y <= hi[2] && lo[3] <= z <= hi[3]) && push!(ref, Vec3f(x, y, z))
end
@printf("reference map: %d of %d points in the walked area\n", length(ref), n)
mappc = PointCloud(ref); ix = build_point_index(mappc; chunk_size=1024)

function residuals(k, place)       # NN distance from a scan's points (2–25 m) to the map
    st, frame, pts = decode_pointcloud2(message_data(bag, msgs[k]))
    T = place(st, frame)
    d = Float64[]
    for (i, v) in enumerate(pts)
        i % 5 == 0 || continue
        r = norm(v.position); 2 < r < 25 || continue
        _, dist = nearest_points(ix, Vec3f(T(v.position)), 1)
        isempty(dist) || push!(d, Float64(first(dist)))
    end
    return d
end
variants = [
    "SpatialRender TF (with /tf_static)" => (st, f) -> lookup_transform(tree, "map", f, st),
    "pose at recording time (+102 ms)"   => (st, f) -> lookup_transform(tree, "map", f, st + 102_000_000),
    "without /tf_static (scan at body)"  => (st, f) -> lookup_transform(tree, "map", "body", st),
    "mounting inverted"                  => (st, f) -> lookup_transform(tree, "map", "body", st) * inv(lookup_transform(tree, "body", f, st)),
]
ks = round.(Int, range(1, length(msgs), length=40))
for (name, place) in variants
    meds = Float64[]; all = Float64[]
    for k in ks
        d = residuals(k, place); push!(meds, median(d)); append!(all, d)
    end
    sort!(all)
    @printf("%-36s median %5.1f cm  p90 %6.1f cm  within 10 cm %5.1f %%  worst scan median %5.1f cm\n",
            name, 100median(all), 100all[round(Int, 0.9length(all))], 100count(<(0.1), all) / length(all), 100maximum(meds))
end
end
main(ARGS...)
