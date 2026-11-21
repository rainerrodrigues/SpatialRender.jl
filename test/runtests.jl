using SpatialRender
using SpatialRender: AABB, center, radius, ntriangles, triangle, ray_box, nchunks, rotate,
                     rotation_matrix, forward, target, mouse_button!, cursor!, scroll!, key!,
                     update!, set_viewport!, ishit
using StaticArrays, LinearAlgebra, Random, Test

const TMP = mktempdir()
tmp(name) = joinpath(TMP, name)

# A unit-ish test mesh: subdivided sphere-like blob of random triangles.
function random_mesh(rng, n)
    verts = MeshVertex[]
    idx = UInt32[]
    for t in 1:n
        c = Vec3f(randn(rng, 3)...) * 3
        for k in 1:3
            push!(verts, MeshVertex(c + Vec3f(randn(rng, 3)...) * 0.5f0, Vec3f(0, 0, 1)))
            push!(idx, UInt32(length(verts) - 1))
        end
    end
    TriMesh(verts, idx)
end

# Allocation checks run inside functions: at test-block scope Julia 1.10
# counts the boxing of captured variables, not the call itself.
alloc_raycast(bvh, r) = (raycast(bvh, r); @allocated raycast(bvh, r))
alloc_cull(ix, fr, eye) = (cull!(ix, fr; eye, pixels_per_radian=800.0);
                           @allocated cull!(ix, fr; eye, pixels_per_radian=800.0))

brute_ray(m, r) = minimum((SpatialRender._moller_trumbore(r, a, b - a, c - a) for (a, b, c) in
                           (triangle(m, t) for t in 1:ntriangles(m))); init=Inf32)

@testset "SpatialRender" begin

@testset "vertex layouts" begin
    @test isbitstype(PointVertex) && sizeof(PointVertex) == 24
    @test fieldoffset(PointVertex, 3) == 16 && fieldoffset(PointVertex, 4) == 20
    @test sizeof(MeshVertex) == 32 && sizeof(LineVertex) == 40
    # D3D12-backed GL (Mesa on WSL) converts vertex buffers on the CPU, per
    # draw, when an attribute offset is not 4-byte aligned: keep them aligned
    for T in (PointVertex, MeshVertex, LineVertex, SpatialRender.LabelSegment)
        @test all(fieldoffset(T, i) % 4 == 0 for i in 1:fieldcount(T) if sizeof(fieldtype(T, i)) >= 4 || i == 1)
        @test sizeof(T) % 4 == 0
    end
end

@testset "frames" begin
    for (lat, lon, h) in ((0.0, 0.0, 0.0), (47.3769, 8.5417, 408.0), (-33.86, 151.21, -20.0), (80.0, -120.0, 3000.0))
        lat2, lon2, h2 = ecef_to_geodetic(geodetic_to_ecef(lat, lon, h))
        @test lat2 ≈ lat atol = 1e-9
        @test lon2 ≈ lon atol = 1e-9
        @test h2 ≈ h atol = 1e-4
    end
    f = ENUFrame(47.0, 8.0, 500.0)
    @test norm(to_local(f, (8.0, 47.0, 500.0))) < 1e-6
    north = to_local(f, (8.0, 47.0 + 1 / 111_200, 500.0))          # ≈ 1 m north
    @test abs(north[1]) < 1e-3 && 0.98 < north[2] < 1.02 && abs(north[3]) < 1e-3
    east = to_local(f, (8.001, 47.0, 500.0))
    @test east[1] > 70 && abs(east[2]) < 0.01
    up = to_local(f, (8.0, 47.0, 510.0))
    @test up ≈ Vec3d(0, 0, 10) atol = 1e-6
    p = Vec3d(123.4, -56.7, 8.9)
    @test to_local(f, to_world(f, p)) ≈ p atol = 1e-6
    o = OffsetFrame(500_000.0, 5_200_000.0, 400.0)
    @test to_local(o, (500_001.25, 5_200_002.5, 401.0)) == Vec3d(1.25, 2.5, 1.0)
end

@testset "STL" begin
    m = TriMesh([MeshVertex(Vec3f(0, 0, 0), Vec3f(0, 0, 1)), MeshVertex(Vec3f(1, 0, 0), Vec3f(0, 0, 1)),
                 MeshVertex(Vec3f(0, 1, 0), Vec3f(0, 0, 1)), MeshVertex(Vec3f(0, 0, 1), Vec3f(0, 0, 1))],
                UInt32[0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2])
    save_stl(tmp("tet.stl"), m)
    @test filesize(tmp("tet.stl")) == 84 + 50 * 4
    hard = load_stl(tmp("tet.stl"))
    @test ntriangles(hard) == 4 && length(hard.vertices) == 12
    @test all(norm(v.normal) ≈ 1 for v in hard.vertices)
    soft = load_stl(tmp("tet.stl"); smooth=true)
    @test length(soft.vertices) == 4 && ntriangles(soft) == 4
    mm = load_stl(tmp("tet.stl"); scale=1e-3)
    @test bounds(mm).hi ≈ Vec3f(1e-3, 1e-3, 1e-3)

    write(tmp("tri.stl"), """
    solid test
      facet normal 0 0 1
        outer loop
          vertex 0 0 0
          vertex 1 0 0
          vertex 0 1 0
        endloop
      endfacet
    endsolid test
    """)
    a = load_stl(tmp("tri.stl"))
    @test ntriangles(a) == 1 && a.vertices[1].normal ≈ Vec3f(0, 0, 1)
    @test_throws ErrorException load_mesh("part.step")
end

@testset "OBJ" begin
    write(tmp("quad.obj"), """
    # a quad with uv and normals, then a triangle with negative indices
    v 0 0 0
    v 1 0 0
    v 1 1 0
    v 0 1 0
    vt 0 0
    vt 1 0
    vt 1 1
    vt 0 1
    vn 0 0 1
    f 1/1/1 2/2/1 3/3/1 4/4/1
    v 0 0 1
    f -1 -5 -4
    """)
    m = load_obj(tmp("quad.obj"))
    @test ntriangles(m) == 3
    @test m.vertices[3].uv == Vec2f(1, 1)
    @test bounds(m).hi == Vec3f(1, 1, 1)
end

@testset "LAS" begin
    rng = Xoshiro(3)
    n = 5000
    origin = Vec3d(612_345.0, 5_123_456.0, 310.0)        # UTM-like coordinates
    world = [origin + Vec3d(100rand(rng), 100rand(rng), 20rand(rng)) for _ in 1:n]
    fr = OffsetFrame(origin)
    pc = PointCloud([Vec3f(to_local(fr, p)) for p in world];
                    intensity=rand(rng, n), classification=rand(rng, 0:9, n),
                    rgb=[(rand(rng, UInt8), rand(rng, UInt8), rand(rng, UInt8)) for _ in 1:n])
    pc.gps_time = collect(range(1000.0, 1010.0, length=n))
    save_las(tmp("t.las"), pc; frame=fr, scale=0.001)
    h = read_las_header(tmp("t.las"))
    @test h.npoints == n && h.point_format == 3 && h.version == v"1.2"
    back, f2 = load_las(tmp("t.las"); frame=fr, gps_time=true)
    @test length(back) == n
    err = maximum(norm(back.vertices[i].position - pc.vertices[i].position) for i in 1:n)
    @test err < 1e-3                                     # mm quantisation, no Float32 loss
    @test all(back.vertices[i].classification == pc.vertices[i].classification for i in 1:n)
    @test all(back.vertices[i].rgba == pc.vertices[i].rgba for i in 1:n)
    @test back.gps_time ≈ pc.gps_time
    @test back.color_mode == Int32(COLOR_RGB)
    # default frame: centred on the file's bounds, so local coordinates are small
    c, fc = load_las(tmp("t.las"))
    @test fc isa OffsetFrame && maximum(maximum(abs.(v.position)) for v in c.vertices) < 60
    @test length(first(load_las(tmp("t.las"); stride=10))) == n ÷ 10
    sel, _ = load_las(tmp("t.las"); classes=[2])
    @test length(sel) == count(v -> v.classification == 2, pc.vertices) > 0
    # load! adopts the LAS frame for an empty scene
    s = Scene(); load!(s, tmp("t.las"))
    @test s.frame isa OffsetFrame && length(s.layers) == 1
end

@testset "GeoJSON" begin
    write(tmp("roads.geojson"), """
    {"type": "FeatureCollection", "features": [
      {"type": "Feature", "properties": {"stroke": "#ff0000"},
       "geometry": {"type": "LineString", "coordinates": [[8.0, 47.0], [8.001, 47.0], [8.001, 47.001]]}},
      {"type": "Feature", "properties": {},
       "geometry": {"type": "Polygon", "coordinates": [[[8.0, 47.0], [8.0005, 47.0], [8.0005, 47.0005], [8.0, 47.0]]]}},
      {"type": "Feature", "properties": null,
       "geometry": {"type": "MultiPoint", "coordinates": [[8.0, 47.0, 10.0], [8.0002, 47.0002]]}}
    ]}
    """)
    f = ENUFrame(47.0, 8.0)
    ll, pts, fr = load_geojson(tmp("roads.geojson"); frame=f)
    @test fr === f
    @test length(ll.polylines) == 2
    @test ll.polylines[1].color == Vec4f(1, 0, 0, 1)
    @test norm(ll.polylines[1].points[1]) < 1e-3
    @test 70 < ll.polylines[1].points[2][1] < 80        # 0.001° lon at 47° ≈ 76 m
    @test length(pts) == 2 && isapprox(pts.vertices[1].position[3], 10; atol=1e-3)
    @test length(SpatialRender.segments(ll)) == 2 + 3
    s = Scene(); load!(s, tmp("roads.geojson"))
    @test s.frame isa ENUFrame && length(s.layers) == 2
end

@testset "trajectory" begin
    t = trajectory([Vec3d(i, 0, 0) for i in 0:10])
    @test length(t.polylines) == 10
    @test t.polylines[1].color != t.polylines[end].color
    @test length(trajectory(rand(3, 5); colormap=:solid).polylines) == 1
end

@testset "filters" begin
    rng = Xoshiro(7)
    plane = [Vec3f(20rand(rng) - 10, 20rand(rng) - 10, 0.02randn(rng)) for _ in 1:20_000]
    boxes = [Vec3f(rand(rng) + 2, rand(rng) + 2, 2rand(rng) + 0.5) for _ in 1:3000]
    outl = [Vec3f(60rand(rng) - 30, 60rand(rng) - 30, 30 + 10rand(rng)) for _ in 1:20]
    pc = PointCloud(vcat(plane, boxes, outl))
    sor = keep_mask(StatisticalOutlierFilter(k=8, nsigma=2.0), pc)
    @test count(.!sor[end-19:end]) == 20                 # all sparse outliers removed
    @test count(sor[1:end-20]) > 0.95 * (length(pc) - 20)

    rad = keep_mask(RadiusOutlierFilter(radius=1.0, min_neighbors=3), pc)
    @test !any(rad[end-19:end])

    @test count(voxel_downsample(pc, 1.0)) <= 2 * 20 * 20 + 2 * 2 * 3 + 20
    plane_fit = fit_ground_plane(pc; threshold=0.1)
    @test plane_fit !== nothing && plane_fit[1][3] > 0.999 && abs(plane_fit[2]) < 0.02
    g = ground_mask(pc; threshold=0.1)
    @test count(g[1:20_000]) > 19_900 && count(g[20_001:end]) < 50
    above = apply_filter(GroundPlaneFilter(threshold=0.1), pc)
    @test 3000 <= length(above) < 3100
    cl = apply_filter(GroundPlaneFilter(threshold=0.1, mode=:classify), pc)
    @test length(cl) == length(pc) && count(v -> v.classification == 2, cl.vertices) > 19_900

    crop = apply_filter(CropBox(Vec3f(1.5, 1.5, 0.3), Vec3f(3.5, 3.5, 3)), pc)
    @test length(crop) == 3000
    pipe = GroundPlaneFilter(threshold=0.1) |> StatisticalOutlierFilter()
    out = apply_filter(pipe, pc)
    @test length(out) < length(above) && length(out) > 2800
    @test count(keep_mask(pipe, pc)) == length(out)

    if Base.find_package("KaosCleaner") === nothing
        @test_throws ErrorException keep_mask(AttributeOutlierFilter(), pc)
    end
end

@testset "BVH ray casting" begin
    rng = Xoshiro(11)
    m = random_mesh(rng, 2000)
    bvh = build_bvh(m)
    @test sort(bvh.tris) == 1:ntriangles(m)
    @test SpatialRender.depth(bvh) < 40
    hits = 0
    for _ in 1:300
        r = Ray(Vec3f(randn(rng, 3)...) * 12, Vec3f(randn(rng, 3)...))
        t, tri = raycast(bvh, r)
        tb = brute_ray(m, r)
        @test t ≈ tb || (isinf(t) && isinf(tb))
        isfinite(t) && (hits += 1)
    end
    @test hits > 20
    r = Ray(Vec3f(0, 0, 50), Vec3f(0, 0, -1))
    @test alloc_raycast(bvh, r) == 0
end

@testset "point index, culling, LOD, picking" begin
    rng = Xoshiro(5)
    pts = [Vec3f(100rand(rng), 100rand(rng), 5rand(rng)) for _ in 1:100_000]
    pc = PointCloud(pts)
    orig = Set(pts)
    ix = build_point_index(pc; chunk_size=1000)
    @test Set(v.position for v in pc.vertices) == orig            # a permutation
    @test sum(ix.chunks.count) == length(pc) && nchunks(ix) >= 100
    for c in eachindex(ix.chunks)                                   # chunk boxes are tight
        rng_ = ix.chunks.first[c]+1:ix.chunks.first[c]+ix.chunks.count[c]
        @test all(contains_point(ix.chunks.box[c], pc.vertices[k].position) for k in rng_)
    end

    cam = Camera(position=Vec3d(50, 50, 20), target=Vec3d(100, 100, 0), aspect=1.5)
    cam.far = 500
    vp = view_projection(cam)
    fr = frustum_from_matrix(vp)
    vis = cull!(ix, fr)
    drawn = falses(length(pc))
    for i in 1:vis.n
        drawn[vis.ranges.first[i]+1:vis.ranges.first[i]+vis.ranges.count[i]] .= true
    end
    inside = [intersects(fr, v.position) for v in pc.vertices]
    @test all(drawn[inside])                                        # conservative
    @test count(drawn) < length(pc)                                 # but culls
    @test vis.npoints == count(drawn)
    @test vis.n <= nchunks(ix)
    @test alloc_cull(ix, fr, Vec3f(cam.position)) == 0
    lod = cull!(ix, fr; eye=Vec3f(cam.position), pixels_per_radian=200.0, density=0.05)
    @test lod.npoints < count(drawn)

    # every frustum corner direction
    @test intersects(fr, Vec3f(80, 80, 0))
    @test !intersects(fr, Vec3f(10, 10, 0))                        # behind the camera

    # picking a known point
    k = 4242
    p = pc.vertices[k].position
    ray = Ray(p + Vec3f(0, 0, 50), Vec3f(0, 0, -1))
    t, i = pick_point(pc, ix, ray; angle=1e-5)
    @test i != 0 && isapprox(pc.vertices[i].position[1], p[1]; atol=1e-2)
    @test t <= 50 + 1e-3
    hit = raycast([(pc, ix)], ray; angle=1e-5)
    @test ishit(hit) && hit.layer === pc
    @test !ishit(raycast([(pc, ix)], Ray(Vec3f(0, 0, 100), Vec3f(0, 0, 1))))
    idx, d = nearest_points(ix, p, 3)
    @test idx[1] == k && d[1] == 0
    @test k in points_in_radius(ix, p, 0.1)
end

@testset "camera" begin
    q = Quat(Vec3d(1, 2, 3), 0.7)
    @test rotation_matrix(Quat(rotation_matrix(q))) ≈ rotation_matrix(q)
    @test rotate(q, Vec3d(1, 0, 0)) ≈ rotation_matrix(q) * Vec3d(1, 0, 0)

    cam = Camera(position=Vec3d(10, -10, 5), target=Vec3d(1, 2, 3))
    V = view_matrix(cam)
    t = V * SVector(1f0, 2f0, 3f0, 1f0)
    @test abs(t[1]) < 1e-4 && abs(t[2]) < 1e-4 && t[3] < 0      # target on -Z
    @test target(cam) ≈ Vec3d(1, 2, 3)
    clip = view_projection(cam) * SVector(1f0, 2f0, 3f0, 1f0)
    @test abs(clip[1] / clip[4]) < 1e-5 && abs(clip[2] / clip[4]) < 1e-5

    d0 = cam.distance
    orbit!(cam, 0.5, 0.2)
    @test target(cam) ≈ Vec3d(1, 2, 3) atol = 1e-9
    @test norm(cam.position - Vec3d(1, 2, 3)) ≈ d0
    for _ in 1:50; orbit!(cam, 0.0, 0.3); end                        # pitch clamped
    @test forward(cam)[3] > -1 && SpatialRender.up(cam)[3] > 0
    tgt = target(cam)
    pan!(cam, 100, 0, 800)
    @test target(cam) != tgt && cam.distance ≈ d0
    dolly!(cam, 3)
    @test cam.distance ≈ d0 * 0.9^3

    r = screen_ray(cam, 640, 400, 1280, 800)
    @test r.dir ≈ Vec3f(forward(cam)) atol = 1e-5
    b = AABB(Vec3f(-1, -1, -1), Vec3f(1, 1, 1))
    frame!(cam, b)
    @test target(cam) ≈ Vec3d(0, 0, 0) atol = 1e-9
    fr = frustum_from_matrix(view_projection(cam))
    @test all(intersects(fr, Vec3f(x, y, z)) for x in (-1, 1), y in (-1, 1), z in (-1, 1))

    cam.ortho = true
    P = projection_matrix(cam)
    @test P[4, 4] == 1
end

@testset "controller state machine" begin
    cam = Camera(position=Vec3d(0, -10, 0), target=Vec3d(0, 0, 0))
    ctl = CameraController(cam)
    set_viewport!(ctl, 800, 600)
    @test cam.aspect ≈ 800 / 600
    cursor!(ctl, 100, 100)
    @test mouse_button!(ctl, :left, true; time=1.0) == :none
    @test ctl.state == SpatialRender.ORBITING
    p0 = cam.position
    cursor!(ctl, 150, 100)
    @test cam.position != p0 && norm(cam.position) ≈ 10
    mouse_button!(ctl, :left, false)
    @test ctl.state == SpatialRender.IDLE
    @test mouse_button!(ctl, :left, true; time=2.0) == :none       # too slow: no double click
    mouse_button!(ctl, :left, false)
    @test mouse_button!(ctl, :left, true; time=2.2) == :double_click
    mouse_button!(ctl, :left, false)
    mouse_button!(ctl, :right, true)
    @test ctl.state == SpatialRender.PANNING
    mouse_button!(ctl, :right, false)
    scroll!(ctl, 2)
    @test cam.distance < 10
    @test update!(ctl, 0.016) == true && update!(ctl, 0.016) == false

    key!(ctl, :tab, true); key!(ctl, :tab, false)
    @test ctl.mode == FlyMode
    p0 = cam.position
    key!(ctl, :w, true)
    update!(ctl, 0.5)
    @test norm(cam.position - p0) ≈ ctl.fly_speed * 0.5
    key!(ctl, :w, false); key!(ctl, :shift, true); key!(ctl, :d, true)
    p0 = cam.position
    update!(ctl, 0.5)
    @test norm(cam.position - p0) ≈ 4 * ctl.fly_speed * 0.5
    key!(ctl, :d, false)
    mouse_button!(ctl, :left, true)
    @test ctl.state == SpatialRender.LOOKING
end

@testset "poses" begin
    using SpatialRender: transform_point, inverse_pose, transform_box, IDENTITY4
    M = pose_matrix(Vec3d(10, 0, 2), Quat(Vec3d(0, 0, 1), π / 2); scale=2.0)
    @test transform_point(M, Vec3f(1, 0, 0)) ≈ Vec3f(10, 2, 2) atol = 1e-5
    @test inverse_pose(M) * M ≈ IDENTITY4 atol = 1e-5
    m = TriMesh([MeshVertex(Vec3f(-1, -1, 0), Vec3f(0, 0, 1)), MeshVertex(Vec3f(1, -1, 0), Vec3f(0, 0, 1)),
                 MeshVertex(Vec3f(0, 1, 0), Vec3f(0, 0, 1))], UInt32[0, 1, 2])
    @test world_bounds(m) == bounds(m)
    set_pose!(m, Vec3d(100, 50, 3))
    @test world_bounds(m).lo ≈ Vec3f(99, 49, 3)
    s = Scene(); add!(s, m)
    @test bounds(s).hi ≈ Vec3f(101, 51, 3)
    # picking a posed, scaled mesh: ray straight down onto it
    set_pose!(m, Vec3d(100, 50, 3), Quat(); scale=3.0)
    bvh = build_bvh(m)
    hit = raycast([(m, bvh)], Ray(Vec3f(100, 50, 20), Vec3f(0, 0, -1)))
    @test ishit(hit) && isapprox(hit.t, 17; atol=1e-4) && isapprox(hit.position, Vec3f(100, 50, 3); atol=1e-4)
    @test !ishit(raycast([(m, bvh)], Ray(Vec3f(0, 0, 20), Vec3f(0, 0, -1))))
    r0 = m.rev; touch!(m); @test m.rev == r0 + 1
    @test_throws ArgumentError set_pose!(m, zeros(3, 3))
end

@testset "streaming point cloud" begin
    sp = StreamingPointCloud(10)
    @test length(sp) == 0 && isempty(bounds(sp))
    push_scan!(sp, [Vec3f(i, 0, 0) for i in 1:4])
    @test length(sp) == 4 && sp.dirty == [1:4] && sp.seq == 1
    push_scan!(sp, [Vec3f(i, 0, 0) for i in 1:4]; pose=(Vec3d(0, 0, 5), Quat()))
    @test sp.buffer[5].position == Vec3f(1, 0, 5) && sp.buffer[5].flags == 0x02
    empty!(sp.dirty)
    push_scan!(sp, [Vec3f(0, 0, -1) for _ in 1:4])          # wraps: 9,10 then 1,2
    @test sp.dirty == [9:10, 1:2]
    @test length(sp) == 10 && sp.written == 12
    @test length(sp.scans) == 3                               # first scan only partly overwritten
    push_scan!(sp, [Vec3f(0, 0, 9) for _ in 1:3])            # overwrites 3..5
    @test length(sp.scans) == 3 && sp.scans[1].seq == 2      # scan 1 fully gone
    @test bounds(sp).hi[3] == 9
    big = [Vec3f(i, 0, 0) for i in 1:25]                     # larger than the ring
    push_scan!(sp, big)
    @test sp.buffer[mod1(sp.written, 10)].position == Vec3f(25, 0, 0) && length(sp.scans) == 1
    # many tiny pushes coalesce into one dirty range
    empty!(sp.dirty)
    for _ in 1:100; push_scan!(sp, [Vec3f(0, 0, 0)]); end
    @test length(sp.dirty) <= 65
    # concurrent producers do not corrupt the ring
    sp2 = StreamingPointCloud(100_000)
    @sync for t in 1:4
        Threads.@spawn for _ in 1:200
            push_scan!(sp2, fill(Vec3f(t, 0, 0), 37))
        end
    end
    @test sp2.written == 4 * 200 * 37 && sp2.seq == 800
    @test all(v -> v.position[1] in 1:4, sp2.buffer[1:length(sp2)])
    # picking a posed streaming layer (no GPU needed: brute force over the ring)
    sp3 = StreamingPointCloud(100)
    push_scan!(sp3, [Vec3f(x, 0, 0) for x in -5:1:5])
    set_pose!(sp3, Vec3d(0, 10, 0))
    h = raycast([(sp3, nothing)], Ray(Vec3f(3, 10, 50), Vec3f(0, 0, -1)); angle=1e-3)
    @test ishit(h) && h.position ≈ Vec3f(3, 10, 0) && h.t ≈ 50
    @test !ishit(raycast([(sp3, nothing)], Ray(Vec3f(3, 0, 50), Vec3f(0, 0, -1)); angle=1e-3))
end

@testset "font and annotations" begin
    using SpatialRender: text_segments, text_extent, GLYPHS, label_segments
    # every glyph stays inside its cell (descenders and brackets may dip to -1)
    for (c, strokes) in GLYPHS, st in strokes, (x, y) in st
        @test 0 <= x <= 4 && -1 <= y <= 6.5
    end
    @test isempty(text_segments(" "))
    @test text_segments("a") == [(SpatialRender.SMALL_CAPS * p, SpatialRender.SMALL_CAPS * q)
                                 for (p, q) in text_segments("A")]
    @test text_segments("§") == text_segments("?")             # unknown → ?
    two = text_segments("1\n1")
    @test minimum(min(p[2], q[2]) for (p, q) in two) < -SpatialRender.GLYPH_LINE + 0.1
    @test text_extent("ab\ncde") == Vec2f(18, 6 + SpatialRender.GLYPH_LINE)

    a = Annotations()
    d, dz, slope = measure!(a, Vec3f(0, 0, 0), Vec3f(3, 4, 0))
    @test d ≈ 5 && dz == 0 && slope == 0
    d, dz, slope = measure!(a, Vec3f(0, 0, 0), Vec3f(1, 0, 1))
    @test d ≈ sqrt(2) && dz ≈ 1 && slope ≈ 45
    @test occursin("1.414 m", a.labels[end].text) && occursin("45.0°", a.labels[end].text)
    annotate!(a, Vec3f(1, 2, 3), "hello")
    @test length(a.lines) == 2 && length(a.labels) == 7
    @test bounds(a).hi == Vec3f(3, 4, 3)
    r0 = a.rev
    undo!(a)
    @test length(a.labels) == 6 && a.rev > r0
    undo!(a); undo!(a); undo!(a)                                 # extra undo is a no-op
    @test isempty(a.lines) && isempty(a.labels)
    annotate!(a, Vec3f(0, 0, 0), "X"; marker=true)
    segs = label_segments(a)
    @test count(s -> s.width > 0, segs) == 1                     # one backing plate
    @test length(segs) == 2 + 1 + 2                              # marker ×, plate, X strokes
end

@testset "out-of-core octree" begin
    using SpatialRender: node_points, select_nodes!, split_axes, _bucket_digits
    @test split_axes(Vec3f(400, 400, 12)) == (true, true, false)
    @test split_axes(Vec3f(10, 10, 10)) == (true, true, true)
    @test _bucket_digits((3, 0, 0), [(true, true, false), (true, true, false)], (2, 2, 0)) == "11"
    rng = Xoshiro(9)
    n = 200_000
    pts = [Vec3f(400rand(rng) - 200, 300rand(rng) - 150, 3sin(i / 1000) + 0.05randn(rng)) for i in 1:n]
    pc = PointCloud(pts; intensity=rand(rng, n))
    for (src, dir) in ((pc, tmp("oct_pc")), (save_las(tmp("oct.las"), pc; frame=OffsetFrame(5e5, 4e6, 100.0)), tmp("oct_las")))
        build_octree(src, dir; node_capacity=4000, bucket_points=20_000)
        o = OctreeCloud(dir)
        @test length(o) == n
        @test o.nodes[1].name == "r" && length(o.nodes[1].children) == 4     # flat data → quadtree root
        bd = maximum(length(nd.name) - 1 for nd in o.nodes if nd.depth == 0 || true)
        # nothing lost: the bucket level and below store every point exactly once
        buckets = minimum(nd.depth for nd in o.nodes if isempty(nd.children))
        @test all(nd.count <= 4000 for nd in o.nodes)
        inside = all(SpatialRender.contains_point(SpatialRender.expand(o.nodes[i].box, 1e-3), v.position)
                     for i in eachindex(o.nodes) for v in node_points(o, i))
        @test inside
        lvl = maximum(nd.depth for nd in o.nodes)
        @test lvl >= 2
        if src isa PointCloud
            # positions survive exactly (relative to the stored origin)
            orig = Set(round.(v.position .- Vec3f(o.frame.origin), digits=3) for v in pc.vertices)
            k = 0
            for v in node_points(o, length(o.nodes))
                k += round.(v.position, digits=3) in orig
            end
            @test k == o.nodes[end].count
        end
    end
    o = OctreeCloud(tmp("oct_pc"); point_budget=30_000)
    # every point stored once below the coarse levels
    d = SpatialRender.JSON3.read(read(joinpath(tmp("oct_pc"), "octree.json"), String)).bucket_depth
    @test sum(nd.count for nd in o.nodes if nd.depth >= d) == n
    cam = Camera(position=Vec3d(0, -400, 250), target=Vec3d(0, 0, 0), aspect=1.5)
    vp = view_projection(cam)
    out = Int32[]; wanted = Int32[]
    loaded = Set{Int32}([1])
    select_nodes!(out, wanted, o, frustum_from_matrix(vp), Vec3f(cam.position), 800f0, i -> i in loaded)
    @test out == [1] && !isempty(wanted) && all(w -> w in o.nodes[1].children, wanted)
    union!(loaded, Int32.(eachindex(o.nodes)))            # pretend everything is resident
    select_nodes!(out, wanted, o, frustum_from_matrix(vp), Vec3f(cam.position), 800f0, i -> i in loaded)
    @test isempty(wanted) && sum(o.nodes[i].count for i in out) <= 30_000 && length(out) > 1
    # a view of empty space selects nothing
    away = Camera(position=Vec3d(0, 0, 5000), target=Vec3d(0, 0, 10_000))
    select_nodes!(out, wanted, o, frustum_from_matrix(view_projection(away)), Vec3f(away.position), 800f0, i -> true)
    @test isempty(out)
end

@testset "GPU selection" begin
    # outside WSL, or with the user's own GALLIUM_DRIVER, nothing is changed
    if !SpatialRender.is_wsl_gpu()
        @test occursin("not WSL", configure_gpu!(force=true))
    end
    @test SpatialRender._is_software("llvmpipe (LLVM 21.1.8, 256 bits)")
    @test !SpatialRender._is_software("D3D12 (NVIDIA GeForce GTX 1650)")
    withenv("SPATIALRENDER_GPU_AUTOCONFIG" => "0") do
        @test occursin("disabled", configure_gpu!(force=true))
    end
    SpatialRender.GPU_CONFIGURED[] = false            # let the GL tests below configure normally
end

@testset "MCAP container" begin
    using SpatialRender: schema_name
    for comp in ("", "zstd", "lz4")
        path = tmp("t_$comp.mcap")
        w = MCAPWriter(path; compression=comp, chunk_size=256)      # tiny chunks → many of them
        s1 = add_schema!(w, "std_msgs/msg/String"); s2 = add_schema!(w, "foo/msg/Bar", "ros2msg", UInt8[1, 2, 3])
        a = add_channel!(w, "/a", s1); b = add_channel!(w, "/b", s2; metadata=Dict("k" => "v"))
        payloads = Dict{Tuple{Int, Int}, Vector{UInt8}}()
        for i in 1:200
            ch = isodd(i) ? a : b
            t = 1_000 * (201 - i)                                     # written newest first
            d = rand(UInt8, rand(1:40)); payloads[(ch, t)] = d
            write_message!(w, ch, t, d; sequence=i)
        end
        close(w)
        f = open_mcap(path)
        @test length(f.messages) == 200 && length(f.chunks) > 5
        @test issorted([m.log_time for m in f.messages])                # index is time-sorted
        @test time_range(f) == (1_000, 200_000)
        @test all(message_data(f, m) == payloads[(m.channel, m.log_time)] for m in f.messages)
        ts = topics(f)
        @test [t.topic for t in ts] == ["/a", "/b"] && all(t.count == 100 for t in ts)
        @test schema_name(f, b) == "foo/msg/Bar" && f.schemas[s2].data == UInt8[1, 2, 3]
        @test f.channels[b].metadata["k"] == "v"
    end
    @test_throws ErrorException open_mcap(save_stl(tmp("notmcap.stl"), random_mesh(Xoshiro(1), 2)))
    # a rosbag2 directory resolves to its .mcap file
    mkpath(tmp("bagdir")); cp(tmp("t_zstd.mcap"), tmp("bagdir/bagdir_0.mcap"); force=true)
    write(tmp("bagdir/metadata.yaml"), "rosbag2_bagfile_information: {}\n")
    @test length(open_mcap(tmp("bagdir")).messages) == 200
end

@testset "ROS 2 CDR messages" begin
    q = normalize(Quat(0.9, 0.1, -0.2, 0.3))
    t = StampedTransform(1_700_000_000_123_456_789, "odom", "base_link", Vec3d(1.5, -2, 0.25), q)
    tf = decode_tf(encode_tf([t, StampedTransform(5, "base_link", "lidar", Vec3d(0, 0, 1), Quat())]))
    @test length(tf) == 2 && tf[1].stamp == t.stamp && tf[1].parent == "odom" && tf[1].child == "base_link"
    @test tf[1].translation == t.translation && tf[1].rotation.w ≈ q.w && tf[2].child == "lidar"
    o = decode_odometry(encode_odometry(t))
    @test o.parent == "odom" && o.child == "base_link" && o.translation == t.translation && o.rotation.z ≈ q.z
    ps = decode_pose_stamped(encode_pose_stamped(t))
    @test ps.parent == "odom" && ps.translation == t.translation
    st, frame, pts = decode_path(encode_path(42, "map", [Vec3d(i, 2i, 0) for i in 1:5]))
    @test st == 42 && frame == "map" && pts[5] == Vec3d(5, 10, 0)
    st, frame, status, lat, lon, alt = decode_navsatfix(encode_navsatfix(7, "gps", 47.1, 8.2, 410.5; status=-1))
    @test (st, frame, status, lat, lon, alt) == (7, "gps", -1, 47.1, 8.2, 410.5)
    cloud = [Vec3f(i, -i, 0.5i) for i in 1:100]
    st, frame, v = decode_pointcloud2(encode_pointcloud2(99, "lidar", cloud; intensity=fill(127.5, 100), ring=[i % 16 for i in 1:100]))
    @test st == 99 && frame == "lidar" && length(v) == 100
    @test v[10].position == Vec3f(10, -10, 5) && v[10].intensity ≈ 0.5 && v[10].classification == 10
    # a different layout: float64 x/y/z, uint8 intensity, packed rgb, a NaN point
    w = SpatialRender.CDRWriter(); SpatialRender.cdr_header!(w, 1, "cam")
    SpatialRender.cdr!(w, UInt32(1)); SpatialRender.cdr!(w, UInt32(3))
    fields = (("x", 0, 8), ("y", 8, 8), ("z", 16, 8), ("intensity", 24, 2), ("rgb", 28, 7))
    SpatialRender.cdr!(w, UInt32(length(fields)))
    for (nm, off, dt) in fields
        SpatialRender.cdr_string!(w, nm); SpatialRender.cdr!(w, UInt32(off)); SpatialRender.cdr!(w, UInt8(dt)); SpatialRender.cdr!(w, UInt32(1))
    end
    SpatialRender.cdr!(w, UInt8(0)); SpatialRender.cdr!(w, UInt32(32)); SpatialRender.cdr!(w, UInt32(96)); SpatialRender.cdr!(w, UInt32(96))
    for (x, it, rgb) in ((1.0, 0x00, 0x00ff0000), (NaN, 0x10, 0x0000ff00), (3.0, 0xff, 0x000000ff))
        write(w.io, x, 2.0, 3.0, it, UInt8[0, 0, 0], rgb)
    end
    SpatialRender.cdr!(w, UInt8(0))
    _, _, v2 = decode_pointcloud2(SpatialRender.cdr_bytes(w))
    @test length(v2) == 2                                            # NaN dropped
    @test v2[1].rgba == (0xff, 0x00, 0x00, 0xff) && v2[2].rgba == (0x00, 0x00, 0xff, 0xff)
    @test v2[2].intensity == 1f0 && v2[2].position == Vec3f(3, 2, 3)
    @test_throws ErrorException decode_tf(UInt8[0x00, 0x00, 0x00, 0x00, 0x01])   # big-endian
end

@testset "TF frame tree" begin
    tree = FrameTree()
    add_transform!(tree, StampedTransform(0, "map", "odom", Vec3d(10, 0, 0), Quat(Vec3d(0, 0, 1), π / 2)))
    for (k, x) in enumerate(0:4)                                     # base_link moves along odom x
        add_transform!(tree, StampedTransform(Int64(k - 1) * 1_000_000_000, "odom", "base_link", Vec3d(x, 0, 0), Quat()))
    end
    add_transform!(tree, StampedTransform(0, "base_link", "lidar", Vec3d(0, 0, 1), Quat()); static=true)
    SpatialRender.finalize!(tree)
    T = lookup_transform(tree, "map", "lidar", 2_500_000_000)        # between samples: odom x = 2.5
    @test T(Vec3d(0, 0, 0)) ≈ Vec3d(10, 2.5, 1) atol = 1e-9            # odom rotated 90° in map
    @test lookup_transform(tree, "lidar", "map", 2_500_000_000)(T(Vec3d(1, 2, 3))) ≈ Vec3d(1, 2, 3) atol = 1e-9
    @test lookup_transform(tree, "map", "lidar", -5)(Vec3d(0, 0, 0)) ≈ Vec3d(10, 0, 1) atol = 1e-9    # holds before start
    @test lookup_transform(tree, "map", "nowhere", 0) === nothing
    @test SpatialRender.default_fixed_frame(tree) == "map"
    q = slerp(Quat(), Quat(Vec3d(0, 0, 1), π / 2), 0.5)
    @test SpatialRender.rotation_matrix(q) * Vec3d(1, 0, 0) ≈ Vec3d(cos(π / 4), sin(π / 4), 0)
end

@testset "bag player" begin
    using SpatialRender: status, current_time, StampedTransform
    # 4 s bag: base_link moves +1 m/s along map x; a one-point cloud at the
    # sensor origin every 0.1 s; odometry; a path; GNSS 1 m/s east
    path = tmp("player.mcap")
    w = MCAPWriter(path)
    sid(n) = add_schema!(w, n)
    c_tf = add_channel!(w, "/tf", sid("tf2_msgs/msg/TFMessage"))
    c_pc = add_channel!(w, "/points", sid("sensor_msgs/msg/PointCloud2"))
    c_od = add_channel!(w, "/odom", sid("nav_msgs/msg/Odometry"))
    c_pa = add_channel!(w, "/plan", sid("nav_msgs/msg/Path"))
    c_fx = add_channel!(w, "/fix", sid("sensor_msgs/msg/NavSatFix"))
    enu = ENUFrame(47.0, 8.0)
    for k in 0:40
        t = k * 100_000_000
        x = k / 10
        write_message!(w, c_tf, t, encode_tf([StampedTransform(t, "map", "base_link", Vec3d(x, 0, 0), Quat())]))
        write_message!(w, c_pc, t, encode_pointcloud2(t, "base_link", [Vec3f(0, 0, 0)]))
        write_message!(w, c_od, t, encode_odometry(StampedTransform(t, "map", "base_link", Vec3d(x, 0, 0), Quat())))
        k % 10 == 0 && write_message!(w, c_pa, t, encode_path(t, "map", [Vec3d(x, 0, 0), Vec3d(x + 5, 0, 0)]))
        g = to_world(enu, Vec3d(x, 0, 0))
        k % 5 == 0 && write_message!(w, c_fx, t, encode_navsatfix(t, "gps", g[2], g[1], 0.0))
    end
    close(w)
    p = open_bag(path; history=1.0)
    @test p.fixed_frame == "map" && SpatialRender.duration(p) ≈ 4.0
    cloud = only(values(p.clouds))
    @test length(cloud) == 1                                         # the t = 0 scan
    seek!(p, 2.0)
    xs = sort([cloud.buffer[i].position[1] for i in 1:length(cloud)])
    @test xs ≈ collect(1.0:0.1:2.0) atol = 1e-5                       # last 1 s of scans, placed via TF
    marker = only(tr.marker for tr in p.tracks)
    @test pose(marker)[1, 4] ≈ 2.0 atol = 1e-5
    plan = only(l for l in p.scene.layers if l.name == "/plan")
    @test plan.polylines[1].points[1] ≈ Vec3f(2, 0, 0)
    gnss = only(l for l in p.scene.layers if endswith(l.name, "(ENU)"))
    @test gnss.polylines[1].points[1] ≈ Vec3f(0, 0, 0) atol = 1e-3    # anchored to the pose track
    @test gnss.polylines[1].points[end][1] ≈ 4.0 atol = 0.05
    play!(p); tick!(p, 0.55)                                         # wall seconds × rate 1
    @test isapprox(current_time(p), 2.55; atol=1e-6) && length(cloud) == 11 + 5
    p.rate = 4.0; tick!(p, 10.0)                                     # runs off the end: stops
    @test current_time(p) ≈ 4.0 && !p.playing
    seek!(p, 0.5)                                                    # backwards: rebuilt
    @test sort([cloud.buffer[i].position[1] for i in 1:length(cloud)]) ≈ collect(0.0:0.1:0.5) atol = 1e-5
    @test occursin("PAUSE", status(p)) && occursin("frame map", status(p))
    p.loop = true; play!(p); seek!(p, 3.95); tick!(p, 1.0)
    @test current_time(p) == 0.0                                      # wrapped
    # topic filter and explicit fixed frame
    q = open_bag(path; topics=["/points"], fixed_frame="base_link")
    @test length(q.scene.layers) == 1 && q.fixed_frame == "base_link"
end

@testset "PNG writer" begin
    img = [(UInt8(x), UInt8(y), 0x80) for y in 1:3, x in 1:4]
    write_png(tmp("t.png"), img)
    b = read(tmp("t.png"))
    @test b[1:8] == UInt8[0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]
    @test ntoh(reinterpret(UInt32, b[17:20])[1]) == 4      # width
    @test SpatialRender._crc32(Vector{UInt8}("IEND")) == 0xae426082
end

# --- GPU tests: only when an OpenGL 4.1 context can be created ---------------
gl_available = try
    SpatialRender.with_hidden_context(() -> true)
catch
    false
end

@testset "OpenGL backend" begin
    if !gl_available
        @info "no OpenGL 4.1 context available; skipping GPU tests"
        @test_skip false
    else
        res = validate_shaders(backend=:gl)
        @test length(res) == 2 * length(SpatialRender.ALL_SHADERS)
        for r in res
            r.ok || @error "shader $(r.name) $(r.stage)" r.log
            @test r.ok
        end
        bad = ShaderSource(:bad, SpatialRender.GLSL_HEADER * "void main() { gl_Position = vec4(undefined_thing); }",
                           SpatialRender.GLSL_HEADER * "out vec4 f; void main() { f = vec4(1); }")
        r = validate_shaders((bad,); backend=:gl)
        @test !r[1].ok && occursin("undefined_thing", r[1].log)

        rng = Xoshiro(2)
        scene = Scene()
        add!(scene, PointCloud([Vec3f(randn(rng), randn(rng), randn(rng)) for _ in 1:20_000]))
        cube = load_stl(save_stl(tmp("c.stl"), random_mesh(rng, 50)))
        cube.wireframe = true
        add!(scene, cube)
        add!(scene, trajectory([Vec3d(cos(t) * 4, sin(t) * 4, 0) for t in range(0, 2π, length=50)]))
        bg = RenderSettings().background
        bg8 = round.(UInt8, bg[1:3] .* 255)
        img = render_offscreen(scene; width=320, height=240)
        @test size(img) == (240, 320)
        nonbg = count(p -> maximum(abs.(Int.(p) .- Int.(Tuple(bg8)))) > 8, img)
        @test nonbg > 0.05 * length(img)
        before = compile_cache_stats()
        render_offscreen(scene; width=64, height=64)
        @test compile_cache_stats().misses == before.misses + length(SpatialRender.ALL_SHADERS)   # new context → recompiled
        screenshot(tmp("s.png"), scene; width=128, height=96)
        @test filesize(tmp("s.png")) > 128 * 96 * 3

        # poses, streams and re-upload after touch!
        s2 = Scene()
        tri = TriMesh([MeshVertex(Vec3f(-1, -1, 0), Vec3f(0, 0, 1)), MeshVertex(Vec3f(1, -1, 0), Vec3f(0, 0, 1)),
                       MeshVertex(Vec3f(0, 1, 0), Vec3f(0, 0, 1))], UInt32[0, 1, 2]; color=Vec4f(1, 0, 0, 1))
        add!(s2, tri)
        live = add!(s2, StreamingPointCloud(1000; point_size=6))
        push_scan!(live, [Vec3f(x, y, 0.5) for x in 2.5:0.1:3.5, y in -1:0.1:1][:])   # beside the triangle
        cam = Camera(position=Vec3d(0, -0.01, 10), target=Vec3d(0, 0, 0), aspect=1.0)
        reddish(img) = count(p -> p[1] > 150 && p[2] < 80 && p[3] < 80, img)
        nogrid = RenderSettings(grid=false)
        a = render_offscreen(s2; width=100, height=100, camera=deepcopy(cam), settings=nogrid)
        set_pose!(tri, Vec3d(50, 0, 0))                       # moved out of view
        b = render_offscreen(s2; width=100, height=100, camera=deepcopy(cam), settings=nogrid)
        @test reddish(a) > 50 && reddish(b) < reddish(a) ÷ 4
        SpatialRender.with_hidden_context() do
            r = Renderer(s2)
            render_frame!(r, deepcopy(cam), 64, 64)
            @test r.stats.points == length(live) && r.stats.triangles == 0
            g = r.gpu[tri]
            render_frame!(r, deepcopy(cam), 64, 64)
            @test r.gpu[tri] === g                                # unchanged: not re-uploaded
            touch!(tri)
            render_frame!(r, deepcopy(cam), 64, 64)
            @test r.gpu[tri] !== g                                # edited: re-uploaded
            push_scan!(live, [Vec3f(0, 0, 1)])
            render_frame!(r, deepcopy(cam), 64, 64)
            @test isempty(live.dirty) && r.stats.points == length(live)
            SpatialRender.release!(r)
        end

        # eye-dome lighting darkens depth edges; labels draw on top of everything
        s3 = Scene()
        add!(s3, PointCloud([Vec3f(x, y, (abs(x) < 0.5 && abs(y) < 0.5) ? 1 : 0)
                             for x in -2:0.02:2 for y in -2:0.02:2]; color_mode=COLOR_INTENSITY))
        cam3 = Camera(position=Vec3d(0, -0.01, 8), target=Vec3d(0, 0, 0), aspect=1.0)
        lum(img) = sum(sum(Int.(p)) for p in img)
        on = render_offscreen(s3; width=96, height=96, camera=deepcopy(cam3), settings=RenderSettings(grid=false))
        off = render_offscreen(s3; width=96, height=96, camera=deepcopy(cam3), settings=RenderSettings(grid=false, edl=false))
        @test lum(on) < lum(off)                                  # edges shaded
        notes = add!(s3, Annotations())
        annotate!(notes, Vec3f(-1.5, -1.5, 0), "HELLO"; color=Vec4f(1, 0, 0, 1), size=14)
        lab = render_offscreen(s3; width=96, height=96, camera=deepcopy(cam3), settings=RenderSettings(grid=false))
        @test count(p -> p[1] > 180 && p[2] < 90, lab) > 20       # red text visible over the points

        # HUD overlay: status lines drawn bottom-left over the scene
        SpatialRender.with_hidden_context() do
            fbo = Ref{SpatialRender.GLuint}(0); SpatialRender.glGenFramebuffers(1, fbo)
            SpatialRender.glBindFramebuffer(SpatialRender.GL_FRAMEBUFFER, fbo[])
            rb = Ref{SpatialRender.GLuint}(0); SpatialRender.glGenRenderbuffers(1, rb)
            SpatialRender.glBindRenderbuffer(SpatialRender.GL_RENDERBUFFER, rb[])
            SpatialRender.glRenderbufferStorage(SpatialRender.GL_RENDERBUFFER, SpatialRender.GL_RGBA8, 320, 120)
            SpatialRender.glFramebufferRenderbuffer(SpatialRender.GL_FRAMEBUFFER, SpatialRender.GL_COLOR_ATTACHMENT0,
                                                    SpatialRender.GL_RENDERBUFFER, rb[])
            r = Renderer(Scene(); settings=RenderSettings(grid=false))
            render_frame!(r, Camera(), 320, 120); blank = SpatialRender.read_framebuffer(320, 120)
            r.hud = ["PLAY 1.00 / 2.00 S", "ORBIT 60 FPS"]
            render_frame!(r, Camera(), 320, 120); withhud = SpatialRender.read_framebuffer(320, 120)
            bright(img) = count(p -> minimum(p) > 180, img)
            @test bright(blank) == 0 && bright(withhud[61:end, 1:160]) > 30 && bright(withhud[1:60, :]) == 0
            SpatialRender.release!(r)
        end

        # out-of-core: offscreen render streams nodes until the view is complete
        s4 = Scene(); load!(s4, tmp("oct_pc"); point_budget=50_000)
        @test s4.frame isa OffsetFrame
        img4 = render_offscreen(s4; width=160, height=120, settings=RenderSettings(grid=false))
        bg8 = round.(UInt8, RenderSettings().background[1:3] .* 255)
        @test count(p -> maximum(abs.(Int.(p) .- Int.(Tuple(bg8)))) > 8, img4) > 0.1 * length(img4)
        SpatialRender.with_hidden_context() do
            r = Renderer(s4)
            cam4 = Camera(position=Vec3d(0, -400, 250), target=Vec3d(0, 0, 0))
            render_frame!(r, cam4, 160, 120)
            for _ in 1:20
                SpatialRender.streaming(r) || break
                SpatialRender.drain!(r); render_frame!(r, cam4, 160, 120)
            end
            @test SpatialRender.OctreeCloud(tmp("oct_pc")).nodes[1].count < r.stats.points <= 50_000   # refined past the root
            # picking hits a point of a drawn node, within the pick cone
            items = SpatialRender.pick_items(r)
            oc = s4.layers[1]
            hit = raycast(items, screen_ray(cam4, 80, 60, 160, 120); angle=4 * cam4.fov / 120)
            @test ishit(hit) && hit.layer === oc
            @test oc.data[hit.index].position ≈ SpatialRender.transform_point(SpatialRender.inverse_pose(oc.model), hit.position) atol = 1e-3
            # adaptive sizes: octant sizes are positive and never larger than the node's own spacing
            g = r.gpu[oc]
            for i in g.draw
                own = SpatialRender.node_spacing(oc, i)
                @test 0 < SpatialRender._effective_spacing(oc, g, i) <= own
            end
            SpatialRender.release!(r)
        end
    end
end

end
