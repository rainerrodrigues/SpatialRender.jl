# Live point data: a fixed-capacity ring buffer of sensor scans.
#
# `push_scan!` (callable from any task or thread) writes a scan into the
# CPU ring and records the written range as dirty; the renderer uploads only
# dirty ranges with `glBufferSubData`, never the whole buffer, and never
# rebuilds an index. Old scans are overwritten in arrival order once the
# ring is full, so memory and per-frame cost stay bounded however long the
# feed runs.

struct ScanRecord
    first::Int        # global (monotonic) index of the scan's first point
    count::Int
    box::AABB
    seq::Int
end

"""
    StreamingPointCloud(capacity; name="live", point_size=2, color_mode=COLOR_AGE, max_age=50)

A point layer for live sensor feeds holding the most recent `capacity`
points. Fill it with [`push_scan!`](@ref). `COLOR_AGE` shades points by how
many scans ago they arrived (newest bright, `max_age` scans and older dark).
"""
mutable struct StreamingPointCloud <: Layer
    name::String
    buffer::Vector{PointVertex}
    written::Int                      # total points ever written
    seq::Int                          # scans pushed
    scans::Vector{ScanRecord}         # live scans, oldest first
    dirty::Vector{UnitRange{Int}}     # 1-based ranges of `buffer` awaiting upload
    point_size::Float32
    color_mode::Int32
    max_age::Float32
    visible::Bool
    model::Mat4f
    rev::Int
end

function StreamingPointCloud(capacity::Integer; name="live", point_size=2.0,
                             color_mode=COLOR_AGE, max_age::Real=50)
    capacity > 0 || throw(ArgumentError("capacity must be positive"))
    StreamingPointCloud(String(name), Vector{PointVertex}(undef, capacity), 0, 0, ScanRecord[],
                        UnitRange{Int}[], Float32(point_size), Int32(color_mode), Float32(max_age),
                        true, IDENTITY4, 0)
end

capacity(s::StreamingPointCloud) = length(s.buffer)
Base.length(s::StreamingPointCloud) = min(s.written, capacity(s))
bounds(s::StreamingPointCloud) = reduce(union, (r.box for r in s.scans); init=AABB())

"""
    push_scan!(layer, points; pose=nothing, intensity=nothing, classification=nothing) -> layer

Append one scan. `points` is a `Vector{PointVertex}` or a vector of
3-vectors in the **sensor** frame; `pose` (a 4×4 matrix, or a
`(position, Quat)` tuple) maps them into the layer frame, so each scan lands
where the vehicle was when it was taken. Thread-safe.

```julia
live = add!(scene, StreamingPointCloud(2_000_000))
@async for (pts, p, q) in lidar_feed
    push_scan!(live, pts; pose = (p, q))
end
open_viewer(scene)
```
"""
function push_scan!(s::StreamingPointCloud, pts::AbstractVector; pose=nothing,
                    intensity=nothing, classification=nothing)
    M = pose === nothing ? IDENTITY4 :
        pose isa Tuple ? pose_matrix(pose[1], pose[2]) : Mat4f(pose)
    n = length(pts)
    cap = capacity(s)
    skip = max(n - cap, 0)              # a scan larger than the ring keeps its tail
    lock(UPDATE_LOCK) do
        seq = s.seq + 1
        flag = UInt8(seq & 0xff)
        box = AABB()
        head = s.written % cap          # 0-based write position
        start = head
        @inbounds for k in skip+1:n
            v = _as_vertex(pts[k], k, intensity, classification)
            p = M === IDENTITY4 ? v.position : transform_point(M, v.position)
            box = union(box, p)
            s.buffer[head+1] = PointVertex(p, v.intensity, v.rgba, v.classification, flag, 0x0000)
            head = head + 1 == cap ? 0 : head + 1
        end
        m = n - skip
        # dirty ranges (split at the wrap-around)
        if m > 0
            if start + m <= cap
                push!(s.dirty, start+1:start+m)
            else
                push!(s.dirty, start+1:cap); push!(s.dirty, 1:start+m-cap)
            end
        end
        s.written += m
        s.seq = seq
        push!(s.scans, ScanRecord(s.written - m, m, box, seq))
        # forget scans that have been completely overwritten
        oldest_alive = s.written - cap
        while !isempty(s.scans) && s.scans[1].first + s.scans[1].count <= oldest_alive
            popfirst!(s.scans)
        end
        length(s.dirty) > 64 && _coalesce!(s.dirty, cap)
    end
    return s
end

_as_vertex(v::PointVertex, k, _, _) = v
_as_vertex(p, k, intensity, classification) =
    PointVertex(Vec3f(p[1], p[2], p[3]), intensity === nothing ? 0f0 : Float32(intensity[k]),
                (0xff, 0xff, 0xff, 0xff), classification === nothing ? 0 : classification[k])

# Many small pushes between frames: upload one covering range instead.
function _coalesce!(d::Vector{UnitRange{Int}}, cap)
    lo = minimum(first, d); hi = maximum(last, d)
    empty!(d); push!(d, lo:hi)
end

"Remove every point (the GPU buffer is kept)."
function Base.empty!(s::StreamingPointCloud)
    lock(UPDATE_LOCK) do
        s.written = 0; empty!(s.scans); empty!(s.dirty)
    end
    return s
end
