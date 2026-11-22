# Changelog

## v0.1.0 (unreleased)

First release.

- Loaders for STL, OBJ, LAS 1.0–1.4 (point formats 0–10, memory-mapped,
  threaded), LAZ (through `laszip` / `pdal`), GeoJSON and trajectories, all
  placed in a local frame (`ENUFrame`, `OffsetFrame`) so Float32 GPU data
  keeps millimetre precision.
- Point cloud filters: statistical and radius outlier removal, voxel
  downsampling, RANSAC ground plane, crop box, composable with `|>`;
  `AttributeOutlierFilter` through the KaosCleaner extension.
- Indexing: SAH BVH for meshes; point clouds reordered into k-d tree chunks
  that drive frustum culling, level of detail, picking and neighbour queries.
- OpenGL 4.1 renderer: point splats coloured by height, intensity, class,
  RGB or scan age; lit meshes with wireframe; anti-aliased screen-space
  lines; eye-dome lighting; stroke-font labels and measurements; a status
  overlay.
- Viewer: quaternion orbit / fly camera, picking, measure mode, screenshots;
  `render_offscreen` works without a display.
- Live data: layer poses (`set_pose!`) and `StreamingPointCloud` ring buffers
  fed by `push_scan!` from any thread.
- Out-of-core point clouds: `build_octree` converts with bounded memory;
  `OctreeCloud` streams nodes within a point budget, with adaptive point sizes.
- ROS 2 bag replay: native MCAP reader and writer (none / zstd / lz4), CDR
  decoding of TF, PointCloud2, Odometry, PoseStamped, Path and NavSatFix, a
  TF frame tree, and a timeline player (`replay`, `open_bag`).
- WSL2: automatic selection of a hardware GPU through Mesa's d3d12 driver.
- Extensions for GeometryBasics/FileIO meshes and GeoInterface geometries
  (GeoDataFrames).

Hardening against real-world files (see *Tested against real-world data* in
the README):

- `open_mcap` reads split rosbag2 recordings (several `.mcap` files) as one
  timeline, in `metadata.yaml` order, unifying channels across files.
- `load_las` reads the complete records of a truncated file (with a warning)
  instead of refusing it, and picks a local origin from the data when the
  header bounding box is wrong.
- `load_geojson` accepts features without a geometry, null geometries and
  coordinates, `[x, y, null]` positions, files with a UTF-8 byte-order mark,
  and files with no coordinates at all; invalid JSON gets a clear error.
- `load_stl` / `load_obj` take `up = :y` for Y-up models.
- PointCloud2 decoding is ~50× faster (a type instability in the per-point loop).

Tested on a real 25-minute autonomous-shuttle recording (nUWAy2), which led to:

- Sensor frames missing from TF (no `/tf_static` recorded) are drawn at the
  robot frame with a warning (`missing_frames = :robot`, or `:origin` /
  `:skip`); unconnected world frames such as `odom` are drawn untransformed;
  `static_transforms = [...]` supplies the real mountings.
- Recording gaps: `trajectory(...; max_gap)` breaks lines across them, the
  player does so for every trail and the GNSS track, and `FrameTree` only
  interpolates across gaps up to `max_gap` (default 2 s).
- `history = :auto` (default) keeps enough scans for sparse / throttled
  LiDAR topics; `skip_gaps = true` jumps over stretches without messages.
- The GNSS track is anchored to a pose track whose frame really reaches the
  fixed frame.

Robustness and documentation:

- Five tutorials (getting started, point clouds, maps and frames, live robot
  data, replaying bags) whose code runs during every docs build.
- JET static analysis is clean and runs in CI; the octree index is validated
  with clear errors (bad JSON, wrong version, truncated `nodes.bin`, …).
- Arguments are validated up front (filters, octree, camera, player, viewer).
- Viewer: an error in a key or mouse handler is logged instead of closing the
  window; per-viewer timers (no shared global state); `open_viewer(...;
  visible = false)`; tested end to end in a hidden window.
- Layers print as one line (`PointCloud "ground" (60000 points)`).
- `save_las` writes colour / GPS time only when present (point formats 0–3),
  so colourless clouds no longer come back as white RGB.
- Scan-age colouring starts from bright red-orange (the dark end of the
  colormap was invisible on the dark background).

Faster first use and notebook display:

- A PrecompileTools workload caches compiled code for loading, filtering,
  indexing, octree building, bag replay and the OpenGL paths: the first
  offscreen render takes 2.4 s instead of 7.7 s, the first viewer frame 0.3 s
  instead of 2.7 s, and the first LAS load and filter 0.02 s instead of 1.7 s.
- `snapshot(scene)` returns a `Snapshot` that Jupyter, VS Code, Pluto and
  Documenter display inline; `write_png(io, img)` writes to any stream. The
  tutorial images now render on the published documentation site.

Checked on a real recording with `/tf_static` (an Ouster OS0-64 forest walk,
CC BY 4.0): LiDAR scans placed through `map → body → base_link → os_sensor →
os_lidar` lie on the dataset's registered map with a median of 4.3 cm (99.6 %
within 10 cm), against 20 cm without `/tf_static`
(`benchmark/realdata_tf_alignment.jl`). Also:

- `MCAPFile` prints as one line inside collections and string interpolation
  (`println(bag)` used to print its raw chunk buffers), and its topic table
  sizes its columns to the longest topic and type name.
