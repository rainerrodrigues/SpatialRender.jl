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
