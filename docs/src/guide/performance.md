# GPU, memory and performance

## GPU on WSL2

On WSL2, SpatialRender selects a hardware GPU before it opens its first
window. Mesa reaches Windows GPUs through its `d3d12` driver, but by default
it often falls back to the CPU. On hybrid laptops it also picks the
integrated GPU first, and if that GPU's WSL driver fails, Mesa gives up
instead of trying the discrete one. SpatialRender sets `GALLIUM_DRIVER=d3d12`
and tries `MESA_D3D12_DEFAULT_ADAPTER_NAME` = NVIDIA, AMD, Intel, then the
default. Each candidate is checked with a real context; if none works, the
original environment is restored and rendering stays on the CPU.
`configure_gpu!()` reports what was chosen.

- Your own `GALLIUM_DRIVER` is never overridden.
- `SPATIALRENDER_GPU_AUTOCONFIG=0` disables the selection.
- To give *every* OpenGL program in WSL the GPU, add these to `~/.profile`
  (adjust the adapter name to your GPU):

  ```bash
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  ```

Measured on a GTX 1650 laptop (WSLg, 1280×720, eye-dome lighting on):

| Scene | GPU (D3D12) | CPU (llvmpipe) |
|---|---|---|
| 500k points, every point drawn | 5.1 ms (195 fps) | 58 ms |
| 5M points, every point drawn | 14.5 ms (69 fps) | 408 ms |
| 5M points, LOD | 7.2 ms (139 fps) | 79 ms |

All vertex attributes are kept 4-byte aligned. D3D12-backed drivers
otherwise convert vertex buffers on the CPU for every draw: an unaligned
1-byte attribute once made multi-range draws hang and lose the device.
A test guards the layouts.

## Memory

Measured on a 3M-point LAS file:

| Stage | Memory |
|---|---|
| point in RAM and on the GPU | 20 bytes |
| `load_las` (memory-mapped, threaded) | 20 B/pt, the vertices only |
| `build_point_index` | 4 B/pt while building; about 16 KB retained in total |
| statistical / radius outlier filter | ~53 / ~31 B/pt while running, nothing retained (1.7 s / 0.5 s for 3M points on 4 threads) |
| `voxel_downsample` | ~18 B/pt while running |
| `build_octree` | about 60 MB of working memory for 3M points beyond the memory-mapped input: the bucket buffers plus one decode chunk, so it is set by `bucket_points`, not the file size |
| octree at runtime | the memory-mapped node file plus the GPU-resident nodes (at most 2 × the point budget) |

Dependencies beyond Julia's standard library: GLFW and ModernGL (window and
OpenGL), StaticArrays, JSON3 (GeoJSON and octree index), and the small
`Zstd_jll` / `Lz4_jll` C libraries for compressed bags.

## Benchmark

`benchmark/pipeline.jl` measures the core pipeline on synthetic data (no GPU
needed); CI runs it on every push and reports the table in the job summary:

```bash
julia -t auto --project benchmark/pipeline.jl 2000000
```

```@docs; canonical=false
configure_gpu!
gl_info
```
