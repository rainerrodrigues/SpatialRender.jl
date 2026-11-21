"""
    SpatialRender

A native OpenGL renderer for meshes, LiDAR point clouds, GIS vector layers and
trajectories in one scene, built for diagnostic views in robotics and aerial
systems.

The package has two halves:

- a GPU-free core (ingestion, local frames, filters, BVH / k-d tree indexing,
  frustum culling, ray picking, camera) that can be used and tested headless;
- an OpenGL 4.1 core-profile backend (GLFW window, interleaved vertex buffers,
  point / mesh / line shader pipelines, render loop).

Quick start:

```julia
using SpatialRender
scene = Scene()
add!(scene, load_mesh("part.stl"))
load!(scene, "scan.las")   # sets the scene frame from the file
open_viewer(scene)
```
"""
module SpatialRender

using LinearAlgebra
using Mmap
using Printf
using Random
using Statistics
using StaticArrays
import JSON3
using ModernGL
import GLFW

# ---- core (no GPU) --------------------------------------------------------
include("util/compression.jl")
include("core/types.jl")
include("camera/quaternion.jl")
include("core/frames.jl")
include("core/pose.jl")
include("core/scene.jl")
include("core/streaming.jl")
include("core/annotations.jl")

include("ingest/stl.jl")
include("ingest/obj.jl")
include("ingest/las.jl")
include("ingest/geojson.jl")
include("ingest/trajectory.jl")
include("ingest/load.jl")

include("index/neighbors.jl")          # before the filters, which use it
include("preprocess/filters.jl")

include("index/aabb.jl")
include("index/bvh.jl")
include("index/pointindex.jl")
include("index/frustum.jl")
include("index/raycast.jl")

include("octree/build.jl")
include("octree/octree.jl")

include("camera/camera.jl")
include("camera/controller.jl")

# ---- OpenGL backend ------------------------------------------------------
include("render/font.jl")
include("render/gl/shaders_src.jl")
include("render/gl/program.jl")
include("render/gl/buffers.jl")
include("render/gl/pipelines.jl")
include("render/gl/octree_gl.jl")
include("render/gl/renderer.jl")
include("render/gpu_select.jl")
include("render/viewer.jl")

# ---- ROS 2 bag replay ----------------------------------------------------
include("ros/mcap.jl")
include("ros/cdr.jl")
include("ros/tf.jl")
include("ros/player.jl")

# geometry
export Vec3f, Vec3d, Vec2f, Vec4f, Mat4f, AABB
export PointVertex, MeshVertex, LineVertex, intensity
export PointCloud, TriMesh, LineLayer, Polyline, compute_normals!
export Scene, add!, layers, bounds, ColorMode, COLOR_HEIGHT, COLOR_INTENSITY,
       COLOR_CLASSIFICATION, COLOR_RGB, COLOR_AGE
# poses and live data
export set_pose!, pose, pose_matrix, touch!, world_bounds, with_update_lock,
       StreamingPointCloud, push_scan!
# out-of-core point clouds
export build_octree, OctreeCloud
# ROS 2 bags
export open_mcap, MCAPFile, MCAPWriter, add_schema!, add_channel!, write_message!, message_data,
       topics, time_range, FrameTree, add_transform!, lookup_transform, StampedTransform, Rigid,
       open_bag, BagPlayer, play!, pause!, seek!, tick!, replay, current_time,
       decode_pointcloud2, decode_tf, decode_odometry, decode_pose_stamped, decode_path, decode_navsatfix,
       encode_pointcloud2, encode_tf, encode_odometry, encode_pose_stamped, encode_path, encode_navsatfix,
       slerp
# annotations
export Label, Annotations, annotate!, measure!, undo!
# frames
export LocalFrame, ENUFrame, OffsetFrame, IdentityFrame, to_local, to_world,
       geodetic_to_ecef, ecef_to_enu, enu_to_ecef, ecef_to_geodetic
# ingest
export load!, load_mesh, load_stl, save_stl, load_obj, load_las, save_las, load_laz,
       load_geojson, gis_layer, trajectory, turbo, LASHeader, read_las_header
# preprocess
export PointFilter, FilterPipeline, StatisticalOutlierFilter, RadiusOutlierFilter,
       VoxelDownsample, GroundPlaneFilter, AttributeOutlierFilter, CropBox, apply_filter,
       keep_mask, statistical_outlier_mask, radius_outlier_mask, voxel_downsample,
       fit_ground_plane, ground_mask, subset
# index
export BVH, build_bvh, PointIndex, build_point_index, Frustum, frustum_from_matrix,
       intersects, cull!, VisibleRanges, DrawRange, Ray, RayHit, ishit, raycast, pick_point,
       nearest_points, points_in_radius, contains_point
# camera
export Quat, Camera, OrbitMode, FlyMode, view_matrix, projection_matrix, view_projection,
       look_at!, orbit!, pan!, dolly!, fly_move!, fly_look!, frame!, screen_ray,
       CameraController, InputState, set_pivot!
# render
export open_viewer, gl_info, configure_gpu!, Viewer, step!, run!, pick!, measure_click!, Renderer, RenderSettings, render_frame!,
       render_offscreen, screenshot, write_png, compile_cache_stats, validate_shaders,
       ShaderSource

end # module
