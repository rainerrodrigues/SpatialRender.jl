# Loading data and local frames

Every loader puts its data into a local Cartesian frame (metres, Z up) near the
origin, so that Float32 GPU coordinates keep millimetre precision. `load!`
picks the frame from the first georeferenced file and reuses it for the rest.

```@docs; canonical=false
load!
load_mesh
load_stl
load_obj
load_las
load_laz
save_las
load_geojson
trajectory
Scene
add!
```

## Frames

```@docs; canonical=false
ENUFrame
OffsetFrame
IdentityFrame
geodetic_to_ecef
ecef_to_geodetic
```

## Point cloud cleanup

```julia
pc, frame = load_las("flight.las"; gps_time = true)
clean = pc |> (GroundPlaneFilter(mode = :classify) |>
               StatisticalOutlierFilter(k = 8, nsigma = 2.0) |>
               VoxelDownsample(0.05))

using KaosCleaner                                      # enables AttributeOutlierFilter
clean = AttributeOutlierFilter(attributes = [:intensity], method = "hampel")(clean)
```

```@docs; canonical=false
StatisticalOutlierFilter
RadiusOutlierFilter
VoxelDownsample
GroundPlaneFilter
CropBox
AttributeOutlierFilter
FilterPipeline
apply_filter
```

## Package extensions

| Load | Enables |
|---|---|
| `KaosCleaner`¹ | `AttributeOutlierFilter` (per-attribute 1-D outlier statistics; Hampel windows follow GPS time) |
| `GeometryBasics` + `FileIO` (+ `MeshIO`) | `TriMesh(::GeometryBasics.Mesh)`; `load_mesh` falls back to FileIO for PLY, OFF, … |
| `GeoInterface` (e.g. via `GeoDataFrames`) | `gis_layer(df.geometry; frame = scene.frame)` for shapefiles, GeoPackage, any OGR source |

¹ KaosCleaner (a Python package with a Julia engine) is not in Julia's General
registry, so SpatialRender finds it at run time instead of declaring it as a
package extension: install it with
`Pkg.add(url = "https://github.com/rainerrodrigues/KaosCleaner")` and load it
with `using KaosCleaner`.

`GeometryBasics` also exports `Vec3f`; qualify it (`SpatialRender.Vec3f`)
when using both.
