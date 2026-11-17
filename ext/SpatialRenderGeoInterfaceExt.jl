# Any GeoInterface geometry (GeoDataFrames / ArchGDAL, Shapefile.jl, GeoJSON.jl,
# LibGEOS, …) → line and point layers in a local frame.
module SpatialRenderGeoInterfaceExt

using SpatialRender
using SpatialRender: Polyline, LineLayer, PointCloud, PointVertex, Vec3f, Vec4f, LocalFrame,
                     ENUFrame, to_local32, _rgba8, DEFAULT_GIS_COLOR
import GeoInterface as GI

function SpatialRender.gis_layer(geoms; frame::Union{Nothing, LocalFrame}=nothing,
                                 color=DEFAULT_GIS_COLOR, width::Real=2.0, height::Real=0.0,
                                 name="gis")
    geoms = [g for g in geoms if g !== missing && g !== nothing]
    if frame === nothing
        lo, hi = [Inf, Inf], [-Inf, -Inf]
        for g in geoms
            _points(g) do p
                lo .= min.(lo, (GI.x(p), GI.y(p))); hi .= max.(hi, (GI.x(p), GI.y(p)))
            end
        end
        isfinite(lo[1]) || error("no coordinates in the given geometries")
        frame = ENUFrame((lo[2] + hi[2]) / 2, (lo[1] + hi[1]) / 2, 0.0)
    end
    col = Vec4f(color)
    tl(p) = to_local32(frame, (Float64(GI.x(p)), Float64(GI.y(p)),
                               (GI.is3d(p) ? Float64(GI.z(p)) : 0.0) + height))
    lines = Polyline[]; marks = Vec3f[]
    for g in geoms
        _collect!(GI.geomtrait(g), g, lines, marks, col, tl)
    end
    pts = PointCloud(PointVertex[PointVertex(p, 1f0, _rgba8(Tuple(col)), 0) for p in marks];
                     name="$name points", point_size=6.0, color_mode=SpatialRender.COLOR_RGB)
    return LineLayer(lines; name, width), pts, frame
end

_points(f, g) = _points(f, GI.geomtrait(g), g)
_points(f, ::GI.PointTrait, g) = f(g)
_points(f, ::GI.AbstractGeometryTrait, g) = foreach(s -> _points(f, s), GI.getgeom(g))

_line!(lines, g, col, tl) = GI.npoint(g) >= 2 && push!(lines, Polyline([tl(p) for p in GI.getpoint(g)], col))

_collect!(::GI.PointTrait, g, lines, marks, col, tl) = push!(marks, tl(g))
_collect!(::GI.MultiPointTrait, g, lines, marks, col, tl) = foreach(p -> push!(marks, tl(p)), GI.getpoint(g))
_collect!(::Union{GI.LineStringTrait, GI.LinearRingTrait, GI.LineTrait}, g, lines, marks, col, tl) =
    _line!(lines, g, col, tl)
_collect!(::GI.PolygonTrait, g, lines, marks, col, tl) = foreach(r -> _line!(lines, r, col, tl), GI.getring(g))
_collect!(::GI.AbstractGeometryTrait, g, lines, marks, col, tl) =     # Multi*, collections
    foreach(s -> _collect!(GI.geomtrait(s), s, lines, marks, col, tl), GI.getgeom(g))

end
