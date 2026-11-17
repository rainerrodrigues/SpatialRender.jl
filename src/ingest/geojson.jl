# GeoJSON vector layers (RFC 7946: WGS84 lon/lat[/height]).

const DEFAULT_GIS_COLOR = Vec4f(0.95, 0.75, 0.2, 1)

"""
    load_geojson(path; frame=nothing, color, width=2, height=0, name) -> (LineLayer, PointCloud, frame)

Read a GeoJSON file. Lines and polygon rings become a [`LineLayer`](@ref);
`Point`/`MultiPoint` features become a [`PointCloud`](@ref) of markers.

Coordinates are taken as WGS84 longitude/latitude and reprojected into
`frame`. With `frame=nothing` an [`ENUFrame`](@ref) at the centre of the
data's lon/lat bounds is used. `height` is added to 2-D coordinates (drape
boundaries slightly above terrain). A feature property `"stroke"` holding a
`#rrggbb` color overrides `color`.

For shapefiles, GeoPackage and other OGR formats load `GeoDataFrames` and use
`load_geodataframe` (a package extension).
"""
function load_geojson(path::AbstractString; frame::Union{Nothing, LocalFrame}=nothing,
                      color=DEFAULT_GIS_COLOR, width::Real=2.0, height::Real=0.0,
                      name=basename(path))
    doc = JSON3.read(read(path, String))
    feats = _features(doc)
    if frame === nothing
        lo, hi = [Inf, Inf], [-Inf, -Inf]
        for (geom, _) in feats
            _each_coord(geom) do c
                lo .= min.(lo, (c[1], c[2])); hi .= max.(hi, (c[1], c[2]))
            end
        end
        isfinite(lo[1]) || error("$path contains no coordinates")
        frame = ENUFrame((lo[2] + hi[2]) / 2, (lo[1] + hi[1]) / 2, 0.0)
    end
    lines = Polyline[]
    marks = Vec3f[]
    tolocal(c) = to_local32(frame, (Float64(c[1]), Float64(c[2]), (length(c) > 2 ? Float64(c[3]) : 0.0) + height))
    for (geom, props) in feats
        col = _stroke(props, Vec4f(color))
        _collect!(lines, marks, geom, col, tolocal)
    end
    ll = LineLayer(lines; name, width)
    pts = PointCloud(PointVertex[PointVertex(p, 1f0, _rgba8(Tuple(color)), 0) for p in marks];
                     name="$name points", point_size=6.0, color_mode=COLOR_RGB)
    return ll, pts, frame
end

function _features(doc)
    t = get(doc, :type, nothing)
    t == "FeatureCollection" && return [(f.geometry, get(f, :properties, nothing)) for f in doc.features if f.geometry !== nothing]
    t == "Feature" && return [(doc.geometry, get(doc, :properties, nothing))]
    return [(doc, nothing)]   # bare geometry
end

function _each_coord(f, geom)
    t = geom.type
    if t == "GeometryCollection"
        foreach(g -> _each_coord(f, g), geom.geometries)
    else
        _walk(f, geom.coordinates)
    end
end
_walk(f, c) = c[1] isa Number ? f(c) : foreach(x -> _walk(f, x), c)

function _collect!(lines, marks, geom, col, tl)
    t = geom.type
    C = t == "GeometryCollection" ? nothing : geom.coordinates
    line(cs) = length(cs) >= 2 && push!(lines, Polyline([tl(c) for c in cs], col))
    if t == "Point"
        push!(marks, tl(C))
    elseif t == "MultiPoint"
        foreach(c -> push!(marks, tl(c)), C)
    elseif t == "LineString"
        line(C)
    elseif t == "MultiLineString" || t == "Polygon"
        foreach(line, C)
    elseif t == "MultiPolygon"
        foreach(poly -> foreach(line, poly), C)
    elseif t == "GeometryCollection"
        foreach(g -> _collect!(lines, marks, g, col, tl), geom.geometries)
    else
        @warn "skipping unsupported GeoJSON geometry $t"
    end
end

function _stroke(props, default)
    props === nothing && return default
    s = get(props, :stroke, nothing)
    s isa AbstractString && length(s) == 7 && s[1] == '#' || return default
    v = parse(UInt32, s[2:end]; base=16)
    Vec4f(((v >> 16) & 0xff) / 255, ((v >> 8) & 0xff) / 255, (v & 0xff) / 255, 1)
end
