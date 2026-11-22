# Large surveys (out-of-core)

```julia
build_octree("county.las", "county_octree")    # once; streams the file, bounded RAM
scene = Scene()
load!(scene, "county_octree"; point_budget = 4_000_000)
open_viewer(scene)
```

A 3M-point file converts in about 5 s. Coarse octree levels add about 10%
storage over the raw points.

```@docs; canonical=false
build_octree
OctreeCloud
```
