# ROS 2 message (de)serialisation: CDR little-endian ("XCDR1"), as written by
# rmw_fastrtps / rmw_cyclonedds into bags. Primitives are aligned to their own
# size, counted from the end of the 4-byte encapsulation header.

mutable struct CDRReader{V <: AbstractVector{UInt8}}
    d::V
    pos::Int          # 0-based, relative to the payload after the header
end

function CDRReader(d::AbstractVector{UInt8})
    length(d) >= 4 || error("CDR message shorter than its header")
    d[2] == 0x01 || error(d[2] == 0x00 ? "big-endian CDR is not supported" :
                          "unsupported CDR encapsulation $(d[1:2])")
    CDRReader(d, 0)
end

@inline function cdr(c::CDRReader, ::Type{T}) where {T}
    a = sizeof(T)
    c.pos = (c.pos + a - 1) & ~(a - 1)
    c.pos + a + 4 <= length(c.d) || error("CDR message truncated")
    v = GC.@preserve c unsafe_load(Ptr{T}(pointer(c.d) + 4 + c.pos))
    c.pos += a
    return v
end
function cdr_string(c::CDRReader)
    n = Int(cdr(c, UInt32))                      # includes the trailing NUL
    s = n == 0 ? "" : String(c.d[4+c.pos+1:4+c.pos+n-1])
    c.pos += n
    return s
end
cdr_vec3(c) = Vec3d(cdr(c, Float64), cdr(c, Float64), cdr(c, Float64))
function cdr_quat(c)                             # ROS order x, y, z, w
    x, y, z, w = cdr(c, Float64), cdr(c, Float64), cdr(c, Float64), cdr(c, Float64)
    q = Quat(w, x, y, z)
    n = norm(q)
    return n > 0 ? Quat(w / n, x / n, y / n, z / n) : Quat()
end
cdr_skip!(c, ::Type{T}, n) where {T} = (for _ in 1:n; cdr(c, T); end)

"std_msgs/Header → (stamp in ns, frame_id)."
function cdr_header(c)
    sec = cdr(c, Int32); nsec = cdr(c, UInt32)
    return Int64(sec) * 1_000_000_000 + Int64(nsec), cdr_string(c)
end

# --- decoded message types ---------------------------------------------------------

"A stamped rigid transform (tf2_msgs/TFMessage entry, or a pose)."
struct StampedTransform
    stamp::Int64
    parent::String
    child::String
    translation::Vec3d
    rotation::Quat
end

"geometry_msgs/TransformStamped[] from a tf2_msgs/TFMessage."
function decode_tf(d)
    c = CDRReader(d)
    n = Int(cdr(c, UInt32))
    out = Vector{StampedTransform}(undef, n)
    for i in 1:n
        st, parent = cdr_header(c)
        child = cdr_string(c)
        out[i] = StampedTransform(st, parent, child, cdr_vec3(c), cdr_quat(c))
    end
    return out
end

"geometry_msgs/PoseStamped → StampedTransform (child = \"\")."
function decode_pose_stamped(d)
    c = CDRReader(d)
    st, frame = cdr_header(c)
    return StampedTransform(st, frame, "", cdr_vec3(c), cdr_quat(c))
end

"nav_msgs/Odometry → StampedTransform (pose of child_frame_id in header.frame_id)."
function decode_odometry(d)
    c = CDRReader(d)
    st, frame = cdr_header(c)
    child = cdr_string(c)
    return StampedTransform(st, frame, child, cdr_vec3(c), cdr_quat(c))     # covariance/twist ignored
end

"nav_msgs/Path → (stamp, frame_id, poses::Vector{Vec3d})."
function decode_path(d)
    c = CDRReader(d)
    st, frame = cdr_header(c)
    n = Int(cdr(c, UInt32))
    pts = Vector{Vec3d}(undef, n)
    for i in 1:n
        cdr_header(c)
        pts[i] = cdr_vec3(c); cdr_quat(c)
    end
    return st, frame, pts
end

"sensor_msgs/NavSatFix → (stamp, frame_id, status, lat, lon, alt). status < 0 means no fix."
function decode_navsatfix(d)
    c = CDRReader(d)
    st, frame = cdr_header(c)
    status = cdr(c, Int8); cdr(c, UInt16)
    lat, lon, alt = cdr(c, Float64), cdr(c, Float64), cdr(c, Float64)
    return st, frame, Int(status), lat, lon, alt
end

const PF_DATATYPES = Dict(1 => Int8, 2 => UInt8, 3 => Int16, 4 => UInt16, 5 => Int32, 6 => UInt32,
                          7 => Float32, 8 => Float64)

"""
    decode_pointcloud2(d; intensity_range=nothing) -> (stamp, frame_id, Vector{PointVertex})

Decode a sensor_msgs/PointCloud2. Uses the `x`, `y`, `z` fields (float32 or
float64), `intensity` (any numeric type; normalised by `intensity_range`,
default 255 for floats and the type's maximum for integers), `rgb`/`rgba`
(packed) and `ring` (stored as the classification, so class colouring shows
LiDAR rings). Non-finite points are dropped.
"""
function decode_pointcloud2(d; intensity_range=nothing)
    c = CDRReader(d)
    st, frame = cdr_header(c)
    h = Int(cdr(c, UInt32)); w = Int(cdr(c, UInt32))
    nf = Int(cdr(c, UInt32))
    fields = Dict{String, Tuple{Int, Int}}()     # name => (offset, datatype)
    for _ in 1:nf
        name = cdr_string(c); off = Int(cdr(c, UInt32)); dt = Int(cdr(c, UInt8)); cdr(c, UInt32)
        fields[name] = (off, dt)
    end
    bigendian = cdr(c, UInt8) != 0
    bigendian && error("big-endian PointCloud2 data is not supported")
    step = Int(cdr(c, UInt32)); cdr(c, UInt32)
    nbytes = Int(cdr(c, UInt32))
    base = 4 + c.pos                             # data starts here (0-based in d)
    n = h * w
    nbytes >= n * step || error("PointCloud2 data shorter than width × height × point_step")
    all(haskey(fields, k) for k in ("x", "y", "z")) || error("PointCloud2 without x/y/z fields")
    (ox, tx), (oy, ty), (oz, tz) = fields["x"], fields["y"], fields["z"]
    T = PF_DATATYPES[tx]
    T in (Float32, Float64) && ty == tx && tz == tx || error("PointCloud2 x/y/z must share a float type")
    ii = get(fields, "intensity", nothing)
    rgbf = get(fields, "rgb", get(fields, "rgba", nothing))
    ring = get(fields, "ring", nothing)
    irange = Float32(something(intensity_range,
        ii === nothing ? 1 : (PF_DATATYPES[ii[2]] <: AbstractFloat ? 255 : typemax(PF_DATATYPES[ii[2]]))))
    out = Vector{PointVertex}(undef, n)
    k = 0
    GC.@preserve d begin
        p0 = pointer(d) + base
        @inbounds for i in 0:n-1
            p = p0 + i * step
            x = Float32(unsafe_load(Ptr{T}(p + ox))); y = Float32(unsafe_load(Ptr{T}(p + oy)))
            z = Float32(unsafe_load(Ptr{T}(p + oz)))
            (isfinite(x) & isfinite(y) & isfinite(z)) || continue
            it = ii === nothing ? 0f0 : clamp(Float32(_pf_load(p + ii[1], ii[2])) / irange, 0f0, 1f0)
            rgba = if rgbf === nothing
                (0xff, 0xff, 0xff, 0xff)
            else
                u = unsafe_load(Ptr{UInt32}(p + rgbf[1]))
                (UInt8((u >> 16) & 0xff), UInt8((u >> 8) & 0xff), UInt8(u & 0xff), 0xff)
            end
            cls = ring === nothing ? 0 : Int(_pf_load(p + ring[1], ring[2])) % 256
            out[k += 1] = PointVertex(Vec3f(x, y, z), it, rgba, cls)
        end
    end
    resize!(out, k)
    return st, frame, out
end

@inline function _pf_load(p, dt)
    dt == 7 && return Float64(unsafe_load(Ptr{Float32}(p)))
    dt == 4 && return Float64(unsafe_load(Ptr{UInt16}(p)))
    dt == 2 && return Float64(unsafe_load(Ptr{UInt8}(p)))
    dt == 8 && return unsafe_load(Ptr{Float64}(p))
    dt == 6 && return Float64(unsafe_load(Ptr{UInt32}(p)))
    dt == 1 && return Float64(unsafe_load(Ptr{Int8}(p)))
    dt == 3 && return Float64(unsafe_load(Ptr{Int16}(p)))
    dt == 5 && return Float64(unsafe_load(Ptr{Int32}(p)))
    return 0.0
end

# --- writer (test fixtures, recording from Julia) ----------------------------------

mutable struct CDRWriter
    io::IOBuffer
end
function CDRWriter()
    io = IOBuffer(); write(io, UInt8[0x00, 0x01, 0x00, 0x00])
    CDRWriter(io)
end
function cdr!(w::CDRWriter, v::T) where {T <: Union{Number, Bool}}
    a = sizeof(T)
    pad = (-(position(w.io) - 4)) & (a - 1)
    for _ in 1:pad; write(w.io, 0x00); end
    write(w.io, htol(v))
    return w
end
cdr_string!(w, s) = (b = codeunits(String(s)); cdr!(w, UInt32(length(b) + 1)); write(w.io, b, 0x00); w)
cdr_header!(w, stamp::Integer, frame) =
    (cdr!(w, Int32(fld(stamp, 1_000_000_000))); cdr!(w, UInt32(mod(stamp, 1_000_000_000))); cdr_string!(w, frame))
cdr_vec3!(w, v) = (foreach(x -> cdr!(w, Float64(x)), v[1:3]); w)
cdr_quat!(w, q::Quat) = (cdr!(w, q.x); cdr!(w, q.y); cdr!(w, q.z); cdr!(w, q.w); w)
cdr_bytes(w::CDRWriter) = take!(w.io)

"Encode a tf2_msgs/TFMessage."
function encode_tf(ts::AbstractVector{StampedTransform})
    w = CDRWriter(); cdr!(w, UInt32(length(ts)))
    for t in ts
        cdr_header!(w, t.stamp, t.parent); cdr_string!(w, t.child)
        cdr_vec3!(w, t.translation); cdr_quat!(w, t.rotation)
    end
    return cdr_bytes(w)
end

"Encode a nav_msgs/Odometry (zero covariance and twist)."
function encode_odometry(t::StampedTransform)
    w = CDRWriter(); cdr_header!(w, t.stamp, t.parent); cdr_string!(w, t.child)
    cdr_vec3!(w, t.translation); cdr_quat!(w, t.rotation)
    for _ in 1:36; cdr!(w, 0.0); end                 # pose covariance
    for _ in 1:6;  cdr!(w, 0.0); end                 # twist
    for _ in 1:36; cdr!(w, 0.0); end                 # twist covariance
    return cdr_bytes(w)
end

"Encode a geometry_msgs/PoseStamped."
encode_pose_stamped(t::StampedTransform) =
    (w = CDRWriter(); cdr_header!(w, t.stamp, t.parent); cdr_vec3!(w, t.translation); cdr_quat!(w, t.rotation); cdr_bytes(w))

"Encode a nav_msgs/Path."
function encode_path(stamp, frame, pts)
    w = CDRWriter(); cdr_header!(w, stamp, frame); cdr!(w, UInt32(length(pts)))
    for p in pts
        cdr_header!(w, stamp, frame); cdr_vec3!(w, p); cdr_quat!(w, Quat())
    end
    return cdr_bytes(w)
end

"Encode a sensor_msgs/NavSatFix."
function encode_navsatfix(stamp, frame, lat, lon, alt; status=0)
    w = CDRWriter(); cdr_header!(w, stamp, frame)
    cdr!(w, Int8(status)); cdr!(w, UInt16(1))
    cdr!(w, Float64(lat)); cdr!(w, Float64(lon)); cdr!(w, Float64(alt))
    for _ in 1:9; cdr!(w, 0.0); end
    cdr!(w, UInt8(0))
    return cdr_bytes(w)
end

"Encode a sensor_msgs/PointCloud2 with float32 x, y, z, intensity and uint16 ring (Velodyne-like layout)."
function encode_pointcloud2(stamp, frame, pts::AbstractVector; intensity=nothing, ring=nothing)
    w = CDRWriter(); cdr_header!(w, stamp, frame)
    n = length(pts)
    cdr!(w, UInt32(1)); cdr!(w, UInt32(n))
    fields = (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4))
    cdr!(w, UInt32(length(fields)))
    for (name, off, dt) in fields
        cdr_string!(w, name); cdr!(w, UInt32(off)); cdr!(w, UInt8(dt)); cdr!(w, UInt32(1))
    end
    step = 18
    cdr!(w, UInt8(0)); cdr!(w, UInt32(step)); cdr!(w, UInt32(step * n))
    cdr!(w, UInt32(step * n))
    for i in 1:n
        p = pts[i]
        write(w.io, htol(Float32(p[1])), htol(Float32(p[2])), htol(Float32(p[3])),
              htol(Float32(intensity === nothing ? 0 : intensity[i])),
              htol(UInt16(ring === nothing ? 0 : ring[i])))
    end
    cdr!(w, UInt8(1))                                  # is_dense
    return cdr_bytes(w)
end

"Minimal ros2msg schema text for the types the encoders above write."
const ROS2_SCHEMAS = Dict(
    "tf2_msgs/msg/TFMessage" => "geometry_msgs/TransformStamped[] transforms",
    "nav_msgs/msg/Odometry" => "std_msgs/Header header\nstring child_frame_id\ngeometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist",
    "geometry_msgs/msg/PoseStamped" => "std_msgs/Header header\ngeometry_msgs/Pose pose",
    "nav_msgs/msg/Path" => "std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses",
    "sensor_msgs/msg/NavSatFix" => "std_msgs/Header header\nNavSatStatus status\nfloat64 latitude\nfloat64 longitude\nfloat64 altitude\nfloat64[9] position_covariance\nuint8 position_covariance_type",
    "sensor_msgs/msg/PointCloud2" => "std_msgs/Header header\nuint32 height\nuint32 width\nPointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense")
