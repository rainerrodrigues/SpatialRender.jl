# ASPRS LAS 1.0–1.4, point data record formats 0–10, read through Mmap.
#
# LAZ is the same format compressed with LASzip; it is decompressed with the
# `laszip` (LAStools / LASzip) or `pdal` command-line tool when one is
# installed, then read as LAS.

"""
Fields of a LAS public header block that the reader needs.
"""
struct LASHeader
    version::VersionNumber
    point_format::Int
    record_length::Int
    offset_to_points::Int
    npoints::Int
    scale::Vec3d
    offset::Vec3d
    lo::Vec3d
    hi::Vec3d
    compressed::Bool
end

@inline _rd(::Type{T}, data::Vector{UInt8}, off::Integer) where {T} =
    GC.@preserve data unsafe_load(Ptr{T}(pointer(data) + off))

"""
    read_las_header(path) -> LASHeader
"""
read_las_header(path::AbstractString) = open(io -> read_las_header(read(io, 375)), path)

function read_las_header(d::Vector{UInt8})
    length(d) >= 227 && d[1:4] == b"LASF" || error("not a LAS file (missing LASF signature)")
    major, minor = Int(d[25]), Int(d[26])
    rawfmt = Int(d[105])
    fmt = rawfmt & 0x3f
    n = Int(_rd(UInt32, d, 107))
    if (major, minor) >= (1, 4) && length(d) >= 255
        n64 = Int(_rd(UInt64, d, 247))
        n64 > 0 && (n = n64)
    end
    v3(o) = Vec3d(_rd(Float64, d, o), _rd(Float64, d, o + 8), _rd(Float64, d, o + 16))
    LASHeader(VersionNumber(major, minor), fmt, Int(_rd(UInt16, d, 105)),
              Int(_rd(UInt32, d, 96)), n, v3(131), v3(155),
              Vec3d(_rd(Float64, d, 187), _rd(Float64, d, 203), _rd(Float64, d, 219)),
              Vec3d(_rd(Float64, d, 179), _rd(Float64, d, 195), _rd(Float64, d, 211)),
              rawfmt & 0xc0 != 0)
end

"Centre of the header bounding box: a good local origin for this file."
las_center(h::LASHeader) = (h.lo + h.hi) / 2

# Byte offsets inside a point record: (classification, rgb, gps_time); -1 = absent.
function _las_layout(fmt::Int)
    0 <= fmt <= 10 || error("unsupported LAS point format $fmt")
    cls = fmt <= 5 ? 15 : 16
    gps = fmt in (1, 3, 4, 5) ? 20 : fmt >= 6 ? 22 : -1
    rgb = fmt == 2 ? 20 : fmt in (3, 5) ? 28 : fmt in (7, 8, 10) ? 30 : -1
    return cls, rgb, gps
end

"""
    load_las(path; frame=nothing, name=basename(path), gps_time=false, stride=1,
             classes=nothing) -> (PointCloud, frame)

Read a LAS file through a memory map.

- `frame`: local frame for the points. When `nothing`, an `OffsetFrame` at
  the centre of the file's bounding box is used. The frame is returned too,
  so other layers can be loaded into the same coordinates.
- `stride`: keep every `stride`-th point (quick previews of huge files).
- `classes`: keep only these ASPRS classes, e.g. `classes=[2, 6]`.
- `gps_time`: also read per-point GPS time (formats 1, 3–10).

Intensity is normalised to 0–1 by the largest value in the file; 16-bit
RGB is reduced to 8 bits.
"""
function load_las(path::AbstractString; frame::Union{Nothing, LocalFrame}=nothing,
                  name=basename(path), gps_time::Bool=false, stride::Integer=1,
                  classes=nothing, kwargs...)
    endswith(lowercase(path), ".laz") && return load_laz(path; frame, name, gps_time, stride, classes, kwargs...)
    data = Mmap.mmap(path)
    h = read_las_header(data)
    h.compressed && error("$path is LAZ-compressed; use load_laz")
    fr = something(frame, OffsetFrame(las_center(h)))
    cls_off, rgb_off, gps_off = _las_layout(h.point_format)
    gps_time && gps_off < 0 && @warn "LAS point format $(h.point_format) has no GPS time"
    rl = h.record_length
    base = h.offset_to_points
    length(data) >= base + rl * h.npoints || error("truncated LAS file: header promises $(h.npoints) points")

    sel = collect(1:stride:h.npoints)
    if classes !== nothing
        cset = Set{UInt8}(UInt8.(classes))
        mask = cls_off == 15 ?
            [(data[base+(i-1)*rl+cls_off+1] & 0x1f) in cset for i in sel] :
            [data[base+(i-1)*rl+cls_off+1] in cset for i in sel]
        sel = sel[mask]
    end
    n = length(sel)

    # Precision: integer record → Float64 world → local Float64 → Float32.
    # For an OffsetFrame this is exact up to the final Float32 rounding.
    imax = 1
    @inbounds for i in sel
        imax = max(imax, Int(_rd(UInt16, data, base + (i - 1) * rl + 12)))
    end
    rgbmax = 0
    if rgb_off >= 0
        @inbounds for i in sel, c in 0:2
            rgbmax = max(rgbmax, Int(_rd(UInt16, data, base + (i - 1) * rl + rgb_off + 2c)))
        end
    end
    rgbshift = rgbmax > 255 ? 8 : 0

    verts = Vector{PointVertex}(undef, n)
    times = gps_time && gps_off >= 0 ? Vector{Float64}(undef, n) : Float64[]
    _las_decode!(verts, times, data, sel, h, fr, base, rl, cls_off, rgb_off, gps_off, imax, rgbshift)
    mode = get(kwargs, :color_mode, rgb_off >= 0 && rgbmax > 0 ? COLOR_RGB : COLOR_HEIGHT)
    pc = PointCloud(verts; name, gps_time=times, color_mode=mode,
                    (k => v for (k, v) in kwargs if k != :color_mode)...)
    return pc, fr
end

# Function barrier: every argument is concretely typed inside the threaded loop.
function _las_decode!(verts, times, data, sel, h, frame, base, rl, cls_off, rgb_off, gps_off, imax, rgbshift)
    n = length(sel)
    Threads.@threads for j in 1:n
        @inbounds begin
            o = base + (sel[j] - 1) * rl
            X = _rd(Int32, data, o); Y = _rd(Int32, data, o + 4); Z = _rd(Int32, data, o + 8)
            w = Vec3d(X * h.scale[1] + h.offset[1], Y * h.scale[2] + h.offset[2], Z * h.scale[3] + h.offset[3])
            it = Float32(_rd(UInt16, data, o + 12) / imax)
            cl = cls_off == 15 ? data[o+cls_off+1] & 0x1f : data[o+cls_off+1]
            rgba = if rgb_off >= 0
                (UInt8(min(255, _rd(UInt16, data, o + rgb_off) >> rgbshift)),
                 UInt8(min(255, _rd(UInt16, data, o + rgb_off + 2) >> rgbshift)),
                 UInt8(min(255, _rd(UInt16, data, o + rgb_off + 4) >> rgbshift)), 0xff)
            else
                (0xff, 0xff, 0xff, 0xff)
            end
            verts[j] = PointVertex(to_local32(frame, w), it, rgba, cl)
            isempty(times) || (times[j] = _rd(Float64, data, o + gps_off))
        end
    end
    return verts
end

"""
    load_laz(path; kwargs...) -> (PointCloud, frame)

Decompress a LAZ file with `laszip` or `pdal` (whichever is on `PATH`) into
a temporary LAS file and read it with [`load_las`](@ref).
"""
function load_laz(path::AbstractString; kwargs...)
    tmp = tempname() * ".las"
    try
        if Sys.which("laszip") !== nothing
            run(pipeline(`laszip -i $path -o $tmp`; stdout=devnull, stderr=devnull))
        elseif Sys.which("laszip64") !== nothing
            run(pipeline(`laszip64 -i $path -o $tmp`; stdout=devnull, stderr=devnull))
        elseif Sys.which("pdal") !== nothing
            run(pipeline(`pdal translate $path $tmp`; stdout=devnull, stderr=devnull))
        else
            error("reading LAZ needs the `laszip` (https://github.com/LASzip/LASzip) or `pdal` " *
                  "command-line tool on PATH; or convert the file to LAS first")
        end
        kw = Dict{Symbol, Any}(kwargs)
        get!(kw, :name, basename(path))
        return load_las(tmp; kw...)
    finally
        rm(tmp; force=true)
    end
end

"""
    save_las(path, pc; frame=IdentityFrame(), scale=0.001)

Write a LAS 1.2 file (point format 3 when the cloud has GPS times, otherwise
2). Positions are mapped back to world coordinates through `frame` and
quantised to `scale` metres.
"""
function save_las(path::AbstractString, pc::PointCloud; frame::LocalFrame=IdentityFrame(),
                  scale::Real=0.001)
    fmt = isempty(pc.gps_time) ? 2 : 3
    rl = fmt == 2 ? 26 : 34
    n = length(pc)
    world = [to_world(frame, Vec3d(v.position)) for v in pc.vertices]
    lo = n == 0 ? Vec3d(0, 0, 0) : reduce((a, b) -> min.(a, b), world)
    hi = n == 0 ? Vec3d(0, 0, 0) : reduce((a, b) -> max.(a, b), world)
    off = round.(lo)
    s = Float64(scale)
    open(path, "w") do io
        write(io, b"LASF", UInt16(0), UInt16(0), zeros(UInt8, 16), UInt8(1), UInt8(2))
        write(io, rpad("SpatialRender.jl", 32)[1:32], rpad("SpatialRender.jl", 32)[1:32])
        write(io, UInt16(1), UInt16(2026), UInt16(227), UInt32(227), UInt32(0),
              UInt8(fmt), UInt16(rl), UInt32(n), UInt32(n), zeros(UInt32, 4))
        write(io, s, s, s, off[1], off[2], off[3])
        write(io, hi[1], lo[1], hi[2], lo[2], hi[3], lo[3])
        for (i, v) in enumerate(pc.vertices)
            q = round.(Int32, (world[i] - off) / s)
            write(io, q[1], q[2], q[3], round(UInt16, clamp(v.intensity, 0, 1) * 65535),
                  UInt8(0b00001001), v.classification & 0x1f, Int8(0), UInt8(0), UInt16(0))
            fmt == 3 && write(io, pc.gps_time[i])
            write(io, UInt16(v.rgba[1]) << 8, UInt16(v.rgba[2]) << 8, UInt16(v.rgba[3]) << 8)
        end
    end
    return path
end
