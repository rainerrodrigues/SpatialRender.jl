# SpatialRender.jl

A native OpenGL renderer for **meshes, LiDAR point clouds, GIS vector layers and
trajectories in one scene**, for diagnostic views in robotics and aerial
systems. No browser, no plotting stack: GLFW + OpenGL 4.1 core, interleaved
vertex buffers uploaded straight from Julia arrays, and a GPU-free core
(ingestion, local frames, filters, BVH / k-d tree, culling, picking, camera)
that runs and tests headless.

![demo](docs/demo.png)

![live](docs/live.png)

*`examples/demo.jl` (top): a cycloidal-drive disc (CAD), 480k LiDAR points after
outlier removal, GIS parcels reprojected from lon/lat into ENU, and a UAV
survey trajectory colored by time. `examples/live.jl` (bottom): a rover streaming a simulated LiDAR, points colored by scan age.*

## Quick start

```julia
using SpatialRender

scene = Scene()
load!(scene, "scan.las")              # first georeferenced file sets the local frame
load!(scene, "boundaries.geojson")    # reprojected into the same frame
add!(scene, load_stl("bracket.stl"; scale = 1e-3))   # mm CAD → m
open_viewer(scene)
```

```bash
julia -t auto --project=. examples/demo.jl            # interactive
julia -t auto --project=. examples/demo.jl shot.png   # offscreen screenshot
julia -t auto --project=. examples/live.jl            # rover + streaming LiDAR
julia -t auto --project=. examples/octree.jl big.las  # out-of-core survey
```

## What is in the box

| Stage | Implementation |
|---|---|
| **Ingestion** | Binary/ASCII STL, OBJ (native); LAS 1.0–1.4, point formats 0–10, via `Mmap` with threaded decoding (5M points ≈ 0.06 s); LAZ through the `laszip`/`pdal` CLI; GeoJSON; trajectories from positions or lon/lat/alt |
| **Local frames** | `ENUFrame(lat, lon, h)` (WGS84 → ECEF → ENU in Float64), `OffsetFrame` for UTM/projected data. GPU data is Float32 offsets near the origin, so no precision loss |
| **Preprocessing** | Statistical and radius outlier removal, voxel downsampling, RANSAC ground plane (remove / keep / classify), crop box, composable with `\|>`; `AttributeOutlierFilter` (z-score, IQR, MAD, Hampel) via the KaosCleaner engine |
| **Indexing** | Binned-SAH BVH for meshes (allocation-free ray casts); point clouds are reordered into contiguous k-d tree chunks and backed by `NearestNeighbors.KDTree` for k-NN and radius queries |
| **Culling + LOD** | Frustum culling of chunks into a pre-allocated `StructVector{DrawRange}`, whose `first` and `count` arrays feed `glMultiDrawArrays` directly. Points are shuffled inside each chunk, so drawing a shorter prefix gives a uniform subsample. Culling 5M points takes ~8 µs per frame and allocates nothing |
| **GPU** | Interleaved `PointVertex` (24 B: position, intensity, RGBA, class), `MeshVertex` (32 B: position, normal, uv), per-instance `LineVertex`; isbits layouts handed to `glBufferData` without repacking |
| **Shaders** | Points: pixel- or world-sized round splats colored by height / intensity / ASPRS class / RGB (Turbo map). Meshes: Blinn-Phong headlight, hemisphere ambient, two-sided, wireframe overlay. Lines: screen-space, anti-aliased, near-plane clipped instanced ribbons |
| **Shader tooling** | Programs cached per context by source hash; `validate_shaders()` checks every stage with `glslangValidator` (CI, no GPU) or the live driver |
| **Camera** | Quaternion camera, turntable orbit (pitch-clamped), pan, zoom, fly mode, frame-all, top view, ortho toggle, pick-to-pivot |
| **Poses** | Every layer has a pose (`set_pose!(layer, position, Quat)` or a 4×4 matrix). Moving a layer is one uniform update; culling, LOD and picking run in the layer's own frame |
| **Live data** | `StreamingPointCloud` ring buffer: `push_scan!(layer, points; pose)` from any thread or task. Only dirty ranges are uploaded (`glBufferSubData`), nothing is re-indexed, and memory stays bounded. Optional coloring by scan age |
| **Out-of-core** | `build_octree(las, dir)` converts in streaming passes; peak RAM is one bucket (~4M points by default), not the survey. Nodes split only along long axes, so flat surveys become quadtrees. `OctreeCloud` streams nodes by screen size within a point budget, on a background thread, with LRU eviction |
| **Eye-dome lighting** | Screen-space log-depth shading on points and meshes (`B` toggles it) |
| **Annotations** | Labels (`annotate!`) and measurements (`measure!`: distance, Δz, slope) in an original stroke font, so no font files are needed; measure mode in the viewer |
| **Output** | Interactive window; `render_offscreen` / `screenshot` to PNG (dependency-free writer) for CI and reports |

## Viewer controls

| Input | Orbit mode | Fly mode |
|---|---|---|
| left drag | orbit about the pivot | look |
| right / middle / shift-left drag | pan | pan |
| scroll | zoom | fly speed |
| W A S D, Q E (shift = 4×) | — | move, down / up |
| double-click | pick: set the pivot and log the point in local + source coordinates | same |

`Tab` switches orbit ↔ fly · `F` frame all · `T` top view · `O` ortho ·
`1`–`5` color by height / intensity / class / RGB / scan age · `+`/`-` point size ·
`P` pixel ↔ world point size · `L` LOD on/off · `M` wireframe · `G` grid ·
`B` eye-dome lighting · `R` measure mode (click two points; `Backspace` undoes) ·
`F12` screenshot · `H` help · `Esc` close.

Pass `block = false` to get the `Viewer` back and drive it yourself
(`step!(viewer)`), for example from a robot's update loop. Layers added to the
scene while it is open are uploaded on the next frame.

## Live data and poses

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

## Large surveys

```julia
build_octree("county.las", "county_octree")    # once; streams the file, bounded RAM
scene = Scene()
load!(scene, "county_octree"; point_budget = 4_000_000)
open_viewer(scene)
```

A 3M-point file converts in about 5 s. Coarse octree levels add about 10%
storage over the raw points.

## Point cloud cleanup

```julia
pc, frame = load_las("flight.las"; gps_time = true)
clean = pc |> (GroundPlaneFilter(mode = :classify) |>
               StatisticalOutlierFilter(k = 8, nsigma = 2.0) |>
               VoxelDownsample(0.05))

using KaosCleaner                                      # enables the extension
clean = AttributeOutlierFilter(attributes = [:intensity], method = "hampel")(clean)
```

## Package extensions

| Load | Enables |
|---|---|
| `KaosCleaner` | `AttributeOutlierFilter` (per-attribute 1-D outlier statistics; Hampel windows follow GPS time) |
| `GeometryBasics` + `FileIO` (+ `MeshIO`) | `TriMesh(::GeometryBasics.Mesh)`; `load_mesh` falls back to FileIO for PLY, OFF, … |
| `GeoInterface` (e.g. via `GeoDataFrames`) | `gis_layer(df.geometry; frame = scene.frame)` for shapefiles, GeoPackage, any OGR source |

`GeometryBasics` also exports `Vec3f`; qualify it (`SpatialRender.Vec3f`)
when using both.

## Limits

- **STEP / IGES** are B-rep CAD, not meshes; tessellate them to STL/OBJ first
  (FreeCAD or OpenCASCADE). `load_mesh` says so instead of failing obscurely.
- **LAZ** needs `laszip` or `pdal` on `PATH`.
- **Projected GIS layers** need an `OffsetFrame`, or reprojection to EPSG:4326
  for an `ENUFrame`. There is no PROJ dependency.
- Out-of-core clouds show density steps between refined and unrefined nodes
  when the point budget is much smaller than the visible data. They are not
  pickable yet: double-click and measure mode work on in-memory layers only.
- On WSL2, OpenGL often falls back to Mesa's CPU rasterizer (llvmpipe);
  the viewer warns when that happens. Rendering is correct but slow.
- The backend is OpenGL 4.1 core (the newest macOS supports), not WebGPU.
  Rendering is isolated in `src/render/`, so another backend can replace it
  without touching the core.

## Tests

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

993 tests covering:
- frames and every reader (round trips);
- filters;
- BVH against brute force, culling conservativeness and zero allocation;
- picking, including posed and scaled layers;
- the streaming ring buffer, including concurrent producers;
- the stroke font and annotations;
- octree invariants (no lost points, every point inside its node) and node
  selection;
- the camera, the controller state machine, and PNG output.

The GPU tests (shader validation, offscreen rendering, EDL, poses, re-upload
on `touch!`, octree streaming) run when an OpenGL 4.1 context can be created
and are skipped otherwise. CI runs them under Xvfb with Mesa on Julia 1.10
and the current release.
