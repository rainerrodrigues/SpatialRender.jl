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
[`gis_layer`](@ref) (a package extension).
"""
function load_geojson(path::AbstractString; frame::Union{Nothing, LocalFrame}=nothing,
                      color=DEFAULT_GIS_COLOR, width::Real=2.0, height::Real=0.0,
                      name=basename(path))
    text = read(path, String)
    startswith(text, '\ufeff') && (text = text[nextind(text, 1):end])   # UTF-8 byte-order mark
    doc = try
        JSON3.read(text)
    catch e
        throw(ArgumentError("$path is not valid JSON: " * first(split(sprint(showerror, e), '\n'))))
    end
    feats = _features(doc)
    if frame === nothing
        lo, hi = [Inf, Inf], [-Inf, -Inf]
        for (geom, _) in feats
            _each_coord(geom) do c
                lo .= min.(lo, (c[1], c[2])); hi .= max.(hi, (c[1], c[2]))
            end
        end
        if isfinite(lo[1])
            frame = ENUFrame((lo[2] + hi[2]) / 2, (lo[1] + hi[1]) / 2, 0.0)
        else
            @warn "$(basename(path)) has no coordinates (only null or empty geometries)"
            frame = IdentityFrame()
        end
    end
    lines = Polyline[]
    marks = Vec3f[]
    # height only when a third coordinate is present *and* numeric ([x, y, null] occurs)
    tolocal(c) = to_local32(frame, (Float64(c[1]), Float64(c[2]),
                                    (length(c) > 2 && c[3] isa Number ? Float64(c[3]) : 0.0) + height))
    for (geom, props) in feats
        col = _stroke(props, Vec4f(color))
        _collect!(lines, marks, geom, col, tolocal)
    end
    ll = LineLayer(lines; name, width)
    pts = PointCloud(PointVertex[PointVertex(p, 1f0, _rgba8(Tuple(color)), 0) for p in marks];
                     name="$name points", point_size=6.0, color_mode=COLOR_RGB)
    return ll, pts, frame
end

# (geometry, properties) pairs; features without a geometry are skipped.
function _features(doc)
    doc isa JSON3.Object || return Tuple{Any, Any}[]
    t = get(doc, :type, nothing)
    if t == "FeatureCollection"
        fs = get(doc, :features, nothing)
        fs isa AbstractVector || return Tuple{Any, Any}[]
        return [(g, get(f, :properties, nothing)) for f in fs
                if f isa JSON3.Object && (g = get(f, :geometry, nothing)) isa JSON3.Object]
    elseif t == "Feature"
        g = get(doc, :geometry, nothing)
        return g isa JSON3.Object ? [(g, get(doc, :properties, nothing))] : Tuple{Any, Any}[]
    end
    return [(doc, nothing)]   # bare geometry
end

function _each_coord(f, geom)
    if get(geom, :type, nothing) == "GeometryCollection"
        gs = get(geom, :geometries, nothing)
        gs isa AbstractVector && foreach(g -> g isa JSON3.Object && _each_coord(f, g), gs)
    else
        _walk(f, get(geom, :coordinates, nothing))
    end
end
# a position is an array of ≥ 2 numbers; anything else (null, [], junk) is skipped
_walk(f, c) = c isa AbstractVector && !isempty(c) &&
    (c[1] isa Number ? (length(c) >= 2 && c[2] isa Number && f(c)) : foreach(x -> _walk(f, x), c))

_position(c) = c isa AbstractVector && length(c) >= 2 && c[1] isa Number && c[2] isa Number
_positions(cs) = cs isa AbstractVector ? [c for c in cs if _position(c)] : Any[]

function _collect!(lines, marks, geom, col, tl)
    t = get(geom, :type, nothing)
    if t == "GeometryCollection"
        gs = get(geom, :geometries, nothing)
        gs isa AbstractVector && foreach(g -> g isa JSON3.Object && _collect!(lines, marks, g, col, tl), gs)
        return
    end
    C = get(geom, :coordinates, nothing)
    C isa AbstractVector || return                     # null / missing coordinates
    line(cs) = (ps = _positions(cs); length(ps) >= 2 && push!(lines, Polyline([tl(c) for c in ps], col)))
    lists(x) = x isa AbstractVector ? x : Any[]
    if t == "Point"
        _position(C) && push!(marks, tl(C))
    elseif t == "MultiPoint"
        foreach(c -> push!(marks, tl(c)), _positions(C))
    elseif t == "LineString"
        line(C)
    elseif t == "MultiLineString" || t == "Polygon"
        foreach(line, lists(C))
    elseif t == "MultiPolygon"
        foreach(poly -> foreach(line, lists(poly)), lists(C))
    else
        @warn "skipping unsupported GeoJSON geometry $t" maxlog=1
    end
end

function _stroke(props, default)
    props === nothing && return default
    s = get(props, :stroke, nothing)
    s isa AbstractString && length(s) == 7 && s[1] == '#' || return default
    v = parse(UInt32, s[2:end]; base=16)
    Vec4f(((v >> 16) & 0xff) / 255, ((v >> 8) & 0xff) / 255, (v & 0xff) / 255, 1)
end
