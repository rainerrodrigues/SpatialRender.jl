# Multi-modal demo: a CAD part, a noisy LiDAR scan, GIS polygons and a UAV
# trajectory in one ENU scene.
#
#   julia --project=. examples/demo.jl                 # interactive window
#   julia --project=. examples/demo.jl shot.png        # offscreen screenshot

using SpatialRender
using SpatialRender: MeshVertex
using Random

const LAT0, LON0 = 47.3769, 8.5417          # local ENU origin (degrees)
scene = Scene(frame = ENUFrame(LAT0, LON0, 0.0))
rng = Xoshiro(42)

# --- CAD: a cycloidal reducer disc, extruded from its analytic profile ------
function cycloidal_disc(; lobes=10, R=4.0, Rr=0.45, E=0.25, thickness=0.8, n=1200, at=(0, 0, 3))
    prof = map(range(0, 2π, length=n + 1)[1:end-1]) do t
        ψ = atan(sin((1 - lobes) * t), R / (E * lobes) - cos((1 - lobes) * t))
        Vec3f(R * cos(t) - Rr * cos(t + ψ) - E * cos(lobes * t),
              -R * sin(t) + Rr * sin(t + ψ) + E * sin(lobes * t), 0)
    end
    o = Vec3f(at...)
    v = MeshVertex[]; idx = UInt32[]
    vert!(p) = (push!(v, MeshVertex(p + o, Vec3f(0, 0, 1))); UInt32(length(v) - 1))
    zt = Vec3f(0, 0, thickness)
    ct, cb = vert!(zt), vert!(Vec3f(0, 0, 0))                # cap centres (star-shaped profile)
    top = [vert!(p + zt) for p in prof]; bot = [vert!(p) for p in prof]
    for i in 1:n
        j = i % n + 1
        push!(idx, ct, top[i], top[j], cb, bot[j], bot[i])     # caps
        push!(idx, bot[i], bot[j], top[j], bot[i], top[j], top[i])   # wall
    end
    compute_normals!(TriMesh(v, idx; name="cycloidal disc", color=Vec4f(0.75, 0.78, 0.84, 1)))
end
add!(scene, cycloidal_disc())

# --- LiDAR: rolling terrain + two buildings + sensor noise -----------------
terrain(x, y) = 1.5sin(x / 9) * cos(y / 11) + 0.02x
pts = Vec3f[]; inten = Float32[]; cls = UInt8[]
for _ in 1:400_000
    x, y = 120rand(rng) - 60, 120rand(rng) - 60
    push!(pts, Vec3f(x, y, terrain(x, y) + 0.03randn(rng))); push!(inten, 0.3 + 0.1rand(rng)); push!(cls, 2)
end
for (cx, cy, w, h) in ((25, 20, 12, 9), (-30, -15, 16, 6))
    for _ in 1:40_000
        x, y = cx + w * (rand(rng) - 0.5), cy + w * (rand(rng) - 0.5)
        z = terrain(cx, cy) + h * (rand(rng) < 0.6 ? 1.0 : rand(rng))   # roof + walls
        push!(pts, Vec3f(x, y, z)); push!(inten, 0.7 + 0.2rand(rng)); push!(cls, 6)
    end
end
for _ in 1:400                                                   # birds, multipath
    push!(pts, Vec3f(120rand(rng) - 60, 120rand(rng) - 60, 20 + 30rand(rng))); push!(inten, rand(rng)); push!(cls, 7)
end
raw = PointCloud(pts; intensity=inten, classification=cls, name="lidar", point_size=2.0)
clean = apply_filter(StatisticalOutlierFilter(k=8, nsigma=2.5), raw)
@info "outlier removal: $(length(raw)) → $(length(clean)) points"
add!(scene, clean)

# --- GIS: parcel outlines given in lon/lat, reprojected into ENU ------------
deg(e, n) = (LON0 + e / (111_320 * cosd(LAT0)), LAT0 + n / 110_540)   # metres → lon/lat
parcels = [[deg(15, 10), deg(35, 10), deg(35, 30), deg(15, 30), deg(15, 10)],
           [deg(-42, -27), deg(-18, -27), deg(-18, -3), deg(-42, -3), deg(-42, -27)]]
write(joinpath(tempdir(), "parcels.geojson"), """
{"type": "FeatureCollection", "features": [$(join(["""
 {"type": "Feature", "properties": {"stroke": "#ffd23f"},
  "geometry": {"type": "Polygon", "coordinates": [[$(join(("[$(c[1]), $(c[2]), 2.5]" for c in p), ", "))]]}}""" for p in parcels], ","))
]}""")
gis, _, _ = load_geojson(joinpath(tempdir(), "parcels.geojson"); frame=scene.frame, width=3)
add!(scene, gis)

# --- UAV trajectory: lawnmower survey at 25 m, given as lon/lat/alt -----------
wps = Tuple{Float64, Float64, Float64}[]
for (k, n) in enumerate(-50:10:50)
    es = isodd(k) ? (-50:2:50) : (50:-2:-50)
    append!(wps, ((deg(e, n)..., 25.0 + 0.5sin(e / 7)) for e in es))
end
add!(scene, trajectory(wps; frame=scene.frame, width=3, name="uav survey"))

# --- annotations: a roof-to-ground measurement and labels -------------------
notes = add!(scene, Annotations())
roof = Vec3f(-30, -15, terrain(-30, -15) + 6)
measure!(notes, roof, Vec3f(-41, -15, terrain(-41, -15)))
annotate!(notes, Vec3f(25, 20, terrain(25, 20) + 9.5), "Building B\nroof 9.0 m")
annotate!(notes, Vec3f(0, 0, 4), "cycloidal disc\n10 lobes")

display(scene)
if isempty(ARGS)
    open_viewer(scene; title="SpatialRender demo")
else
    cam = Camera(position=Vec3d(-75, -85, 60), target=Vec3d(0, 0, 0))
    screenshot(ARGS[1], scene; width=1280, height=720, camera=cam)
    length(ARGS) > 1 && screenshot(ARGS[2], scene; width=1280, height=720, camera=cam, settings=RenderSettings(edl=false))
    @info "wrote $(ARGS[1])"
end
