# Format dispatch.

"""
    load_mesh(path; kwargs...) -> TriMesh

Load a triangle mesh by extension: `.stl` and `.obj` natively. Other mesh
formats (PLY, OFF, 3MF, …) load through `FileIO`/`MeshIO` once
`GeometryBasics` is imported (package extension).

STEP / IGES are B-rep CAD formats, not meshes: tessellate them first (for
example FreeCAD or OpenCASCADE `STEP → STL` export with a chosen tolerance).
"""
function load_mesh(path::AbstractString; kwargs...)
    ext = lowercase(splitext(path)[2])
    ext == ".stl" && return load_stl(path; kwargs...)
    ext == ".obj" && return load_obj(path; kwargs...)
    ext in (".step", ".stp", ".iges", ".igs") && error(
        "$ext is a B-rep CAD format; tessellate it to STL/OBJ first " *
        "(FreeCAD: File → Export → STL, or OpenCASCADE's StlAPI_Writer)")
    return _load_mesh_fallback(path, ext; kwargs...)
end
# Set by the GeometryBasics/FileIO extension: loads any format FileIO knows.
const MESH_FALLBACK = Ref{Any}(nothing)
function _load_mesh_fallback(path, ext; kwargs...)
    MESH_FALLBACK[] === nothing && error(
        "no native reader for $ext; `using GeometryBasics, FileIO, MeshIO` enables the FileIO mesh loaders")
    return MESH_FALLBACK[](path; kwargs...)
end

"""
    gis_layer(geometries; frame=nothing, color, width=2, height=0, name="gis") -> (LineLayer, PointCloud, frame)

Convert GeoInterface-compatible geometries (a `GeoDataFrames` geometry
column, Shapefile.jl / GeoJSON.jl / LibGEOS objects, …) into a line layer
and a point-marker layer, like [`load_geojson`](@ref).

Coordinates go through `frame`: with an `ENUFrame` (or `frame=nothing`,
which builds one at the data centre) they must be longitude/latitude, so
reproject projected layers to EPSG:4326 first; with an `OffsetFrame` they
are used as projected metres directly.

```julia
using GeoDataFrames
df = GeoDataFrames.read("roads.gpkg")
lines, marks, _ = gis_layer(df.geometry; frame = scene.frame)
add!(scene, lines)
```

Needs `using GeoInterface` (or any package that loads it, such as GeoDataFrames).
"""
function gis_layer end

"""
    load!(scene, path; kwargs...) -> scene

Load any supported file into `scene` in the scene's frame.

When the scene is still empty and uses an `IdentityFrame`, georeferenced
data sets the frame: a LAS file sets an `OffsetFrame` at its centre, a
GeoJSON file an `ENUFrame` at its centre. Later files reuse that frame.
A directory written by [`build_octree`](@ref) loads as an [`OctreeCloud`](@ref).
"""
function load!(scene::Scene, path::AbstractString; kwargs...)
    ispath(path) || throw(ArgumentError("no such file or directory: $path"))
    if isdir(path) && isfile(joinpath(path, "octree.json"))        # converted with build_octree
        o = OctreeCloud(path; kwargs...)
        if isempty(scene.layers) && scene.frame isa IdentityFrame
            scene.frame = o.frame
        elseif scene.frame isa OffsetFrame
            set_pose!(o, pose_matrix(o.frame.origin - scene.frame.origin))   # align origins
        end
        add!(scene, o)
        return scene
    end
    ext = lowercase(splitext(path)[2])
    adopt = isempty(scene.layers) && scene.frame isa IdentityFrame
    fr = adopt ? nothing : scene.frame
    if ext in (".las", ".laz")
        pc, f = load_las(path; frame=fr, kwargs...)
        adopt && (scene.frame = f)
        add!(scene, pc)
    elseif ext in (".geojson", ".json")
        ll, pts, f = load_geojson(path; frame=fr, kwargs...)
        adopt && (scene.frame = f)
        isempty(ll.polylines) || add!(scene, ll)
        length(pts) == 0 || add!(scene, pts)
    else
        add!(scene, load_mesh(path; frame=scene.frame, kwargs...))
    end
    return scene
end
