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
