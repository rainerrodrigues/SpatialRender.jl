# Replaying ROS 2 bags: log time drives the scene.

const T_TF = "tf2_msgs/msg/TFMessage"
const T_CLOUD = "sensor_msgs/msg/PointCloud2"
const T_ODOM = "nav_msgs/msg/Odometry"
const T_POSE = "geometry_msgs/msg/PoseStamped"
const T_PATH = "nav_msgs/msg/Path"
const T_FIX = "sensor_msgs/msg/NavSatFix"

"A pose topic (odometry / PoseStamped) resolved into the fixed frame."
struct PoseTrack
    topic::String
    times::Vector{Int64}
    poses::Vector{Rigid}
    trail::LineLayer
    marker::LineLayer
end

"""
    BagPlayer

Replays an MCAP / rosbag2 file into a [`Scene`](@ref). Create with
[`open_bag`](@ref); drive with [`play!`](@ref), [`pause!`](@ref),
[`seek!`](@ref), [`tick!`](@ref) — or hand it to `open_viewer(scene;
player)`, which ticks it every frame and maps Space / ← → / [ ] / Home.
"""
mutable struct BagPlayer
    bag::MCAPFile
    scene::Scene
    tree::FrameTree
    fixed_frame::String
    t0::Int64
    t1::Int64
    t::Int64
    rate::Float64
    playing::Bool
    loop::Bool
    cursor::Int
    history::Int64
    clouds::Dict{UInt16, StreamingPointCloud}
    paths::Dict{UInt16, Tuple{LineLayer, Vector{Int}}}
    tracks::Vector{PoseTrack}
    intensity_range::Union{Nothing, Float64}
    scans::Int
    warned::Set{String}
end

function _warn_once(p::BagPlayer, key, msg)
    key in p.warned && return
    push!(p.warned, key)
    @warn msg
end

# fixed ← frame at stamp; identity (with one warning per frame) when unconnected
function _to_fixed(p::BagPlayer, frame::AbstractString, stamp::Int64)
    isempty(frame) && return Rigid()
    T = lookup_transform(p.tree, p.fixed_frame, frame, stamp)
    T === nothing || return T
    _warn_once(p, frame, "no transform from \"$frame\" to fixed frame \"$(p.fixed_frame)\"; drawing it untransformed")
    return Rigid()
end

"An RGB axes triad (x red, y green, z blue) as a line layer, `len` metres long."
function axes_marker(len::Real; name="axes", width=3)
    L = Float32(len)
    LineLayer([Polyline([Vec3f(0, 0, 0), Vec3f(L, 0, 0)], Vec4f(0.95, 0.25, 0.25, 1)),
               Polyline([Vec3f(0, 0, 0), Vec3f(0, L, 0)], Vec4f(0.3, 0.9, 0.3, 1)),
               Polyline([Vec3f(0, 0, 0), Vec3f(0, 0, L)], Vec4f(0.3, 0.5, 1.0, 1))]; name, width)
end

"""
    open_bag(path; scene=Scene(), fixed_frame=nothing, topics=nothing, cloud_capacity=2_000_000,
             history=3.0, intensity_range=nothing, marker_size=nothing) -> BagPlayer

Open a ROS 2 bag (an `.mcap` file or a rosbag2 directory) for replay into
`scene` and add a layer per supported topic:

| message type                     | shown as                                              |
|----------------------------------|-------------------------------------------------------|
| `sensor_msgs/PointCloud2`        | streaming cloud, each scan placed via TF at its stamp |
| `nav_msgs/Odometry`, `PoseStamped` | full trajectory + an axes marker at the current pose |
| `nav_msgs/Path`                  | the latest path at the current time                    |
| `sensor_msgs/NavSatFix`          | ENU track, first fix anchored at the robot's pose      |

`/tf` and `/tf_static` build the frame tree. Everything is drawn in
`fixed_frame` (default: `map`, `world`, `odom`, else the tree's root).
`history` seconds of scans stay visible. `topics` restricts which topics
are shown.
"""
function open_bag(path::AbstractString; scene::Scene=Scene(), fixed_frame=nothing, topics=nothing,
                  cloud_capacity::Integer=2_000_000, history::Real=3.0, intensity_range=nothing,
                  marker_size=nothing)
    bag = open_mcap(path)
    isempty(bag.messages) && error("$path contains no messages")
    tree = FrameTree()
    kind = Dict(id => schema_name(bag, id) for id in keys(bag.channels))
    for (id, c) in bag.channels
        c.message_encoding == "cdr" ||
            @warn "topic $(c.topic) uses $(c.message_encoding) encoding; only ROS 2 (cdr) messages are decoded"
    end
    for m in bag.messages
        kind[m.channel] == T_TF || continue
        static = occursin("static", bag.channels[m.channel].topic)
        for t in decode_tf(message_data(bag, m))
            add_transform!(tree, t; static)
        end
    end
    finalize!(tree)
    wanted(id) = bag.channels[id].message_encoding == "cdr" &&
                 (topics === nothing || bag.channels[id].topic in topics)
    t0, t1 = time_range(bag)
    # fixed frame: explicit, from TF, else the first cloud / pose frame
    fixed = something(fixed_frame, default_fixed_frame(tree), Some(nothing))
    if fixed === nothing
        for m in bag.messages
            kind[m.channel] in (T_CLOUD, T_ODOM, T_POSE) || continue
            d = message_data(bag, m)
            fixed = kind[m.channel] == T_CLOUD ? decode_pointcloud2(d)[2] :
                    kind[m.channel] == T_ODOM ? decode_odometry(d).parent : decode_pose_stamped(d).parent
            break
        end
        fixed = something(fixed, "map")
    end
    p = BagPlayer(bag, scene, tree, String(fixed), t0, t1, t0, 1.0, false, false, 1,
                  round(Int64, history * 1e9), Dict{UInt16, StreamingPointCloud}(),
                  Dict{UInt16, Tuple{LineLayer, Vector{Int}}}(), PoseTrack[],
                  intensity_range === nothing ? nothing : Float64(intensity_range), 0, Set{String}())
    duration = max((t1 - t0) / 1e9, 1e-3)
    counts = Dict{UInt16, Int}()
    for m in bag.messages
        counts[m.channel] = get(counts, m.channel, 0) + 1
    end
    for id in sort(collect(keys(bag.channels)); by = id -> bag.channels[id].topic)
        wanted(id) || continue
        topic = bag.channels[id].topic
        k = kind[id]
        if k == T_CLOUD
            hz = get(counts, id, 1) / duration
            p.clouds[id] = add!(scene, StreamingPointCloud(cloud_capacity; name=topic,
                                    max_age=clamp(round(history * hz), 2, 255)))
        elseif k == T_PATH
            idx = [i for (i, m) in enumerate(bag.messages) if m.channel == id]
            p.paths[id] = (add!(scene, LineLayer(Polyline[]; name=topic, width=3)), idx)
        elseif k in (T_ODOM, T_POSE)
            _add_track!(p, id, topic, k, marker_size)
        end
    end
    for id in keys(bag.channels)                 # GNSS last: anchored to a pose track if present
        wanted(id) && kind[id] == T_FIX && _add_gnss!(p, id, bag.channels[id].topic)
    end
    seek!(p, t0)
    return p
end

function _add_track!(p::BagPlayer, id, topic, k, marker_size)
    times = Int64[]; poses = Rigid[]
    for m in p.bag.messages
        m.channel == id || continue
        st = k == T_ODOM ? decode_odometry(message_data(p.bag, m)) : decode_pose_stamped(message_data(p.bag, m))
        push!(times, st.stamp)
        push!(poses, _to_fixed(p, st.parent, st.stamp) * Rigid(st.translation, st.rotation))
    end
    isempty(times) && return
    perm = sortperm(times); times = times[perm]; poses = poses[perm]
    pts = [q.t for q in poses]
    trail = length(pts) >= 2 ? trajectory(pts; times, width=2.5, name="$topic trail") :
                               LineLayer(Polyline[]; name="$topic trail")
    ext = maximum(norm(q - pts[1]) for q in pts)
    marker = axes_marker(something(marker_size, clamp(ext / 25, 0.3, 5.0)); name="$topic pose")
    add!(p.scene, trail); add!(p.scene, marker)
    push!(p.tracks, PoseTrack(topic, times, poses, trail, marker))
end

function _add_gnss!(p::BagPlayer, id, topic)
    fixes = Tuple{Int64, Float64, Float64, Float64}[]
    for m in p.bag.messages
        m.channel == id || continue
        st, _, status, lat, lon, alt = decode_navsatfix(message_data(p.bag, m))
        status >= 0 && isfinite(lat) && isfinite(lon) && push!(fixes, (st, lat, lon, alt))
    end
    length(fixes) >= 2 || return
    sort!(fixes)
    f = p.scene.frame isa ENUFrame ? p.scene.frame : ENUFrame(fixes[1][2], fixes[1][3], fixes[1][4])
    pts = [to_local(f, (lon, lat, alt)) for (_, lat, lon, alt) in fixes]
    # Anchor the track to the robot: shift it so the first fix sits at the pose
    # track's position at that time (assumes the fixed frame's x/y point east/north).
    if !isempty(p.tracks) && !(p.scene.frame isa ENUFrame)
        tr = p.tracks[1]
        k = clamp(searchsortedlast(tr.times, fixes[1][1]), 1, length(tr.times))
        shift = tr.poses[k].t - pts[1]
        pts = [q + Vec3d(shift[1], shift[2], shift[3]) for q in pts]
    end
    add!(p.scene, trajectory(pts; times=[t for (t, _, _, _) in fixes], colormap=:solid,
                             color=Vec4f(1.0, 0.45, 0.85, 0.9), width=2, name="$topic (ENU)"))
end

# --- time control --------------------------------------------------------------------

"""
    play!(player) -> player

Start (or resume) playback; restarts from the beginning when at the end.
Playback advances with [`tick!`](@ref), which the viewer calls every frame.
"""
play!(p::BagPlayer) = (p.t >= p.t1 && seek!(p, p.t0); p.playing = true; p)

"""
    pause!(player) -> player

Stop advancing; the scene keeps showing the current moment.
"""
pause!(p::BagPlayer) = (p.playing = false; p)

"Toggle between playing and paused."
toggle!(p::BagPlayer) = p.playing ? pause!(p) : play!(p)
"Current playback position in seconds from the start of the bag."
current_time(p::BagPlayer) = (p.t - p.t0) / 1e9
duration(p::BagPlayer) = (p.t1 - p.t0) / 1e9

"""
    seek!(player, t) -> player

Jump to log time `t` (nanoseconds; or a `Float64` in seconds from the
start). Short forward jumps continue playback; anything else rebuilds the
streaming layers from the last `history` seconds before `t`.
"""
seek!(p::BagPlayer, s::AbstractFloat) = seek!(p, p.t0 + round(Int64, s * 1e9))
function seek!(p::BagPlayer, t::Integer)
    t = clamp(Int64(t), p.t0, p.t1)
    if t >= p.t && t - p.t <= p.history && p.cursor > 1
        return advance_to!(p, t)
    end
    for c in values(p.clouds)
        empty!(c)
    end
    for (id, (layer, idx)) in p.paths            # latest path at or before t, however old
        j = findlast(i -> p.bag.messages[i].log_time <= t, idx)
        if j === nothing
            with_update_lock(() -> (empty!(layer.polylines); layer.rev += 1))
        else
            _process!(p, p.bag.messages[idx[j]])
        end
    end
    start = t - p.history
    p.cursor = searchsortedfirst(p.bag.messages, start; by = m -> m isa MessageRef ? m.log_time : m)
    p.t = start
    return advance_to!(p, t)
end

"""
    advance_to!(player, t) -> player

Process every message up to log time `t` and update poses.
"""
function advance_to!(p::BagPlayer, t::Int64)
    msgs = p.bag.messages
    while p.cursor <= length(msgs) && msgs[p.cursor].log_time <= t
        _process!(p, msgs[p.cursor])
        p.cursor += 1
    end
    p.t = t
    _update_markers!(p)
    return p
end

function _process!(p::BagPlayer, m::MessageRef)
    if (layer = get(p.clouds, m.channel, nothing)) !== nothing
        st, frame, pts = decode_pointcloud2(message_data(p.bag, m); p.intensity_range)
        push_scan!(layer, pts; pose=pose_matrix(_to_fixed(p, frame, st)))
        p.scans += 1
    elseif (pl = get(p.paths, m.channel, nothing)) !== nothing
        st, frame, pts = decode_path(message_data(p.bag, m))
        T = _to_fixed(p, frame, st)
        line = Polyline([Vec3f(T(q)) for q in pts], Vec4f(0.3, 0.95, 0.6, 1))
        with_update_lock() do
            empty!(pl[1].polylines); length(line.points) >= 2 && push!(pl[1].polylines, line)
            pl[1].rev += 1
        end
    end
end

function _update_markers!(p::BagPlayer)
    for tr in p.tracks
        k = searchsortedlast(tr.times, p.t)
        T = if k == 0
            tr.poses[1]
        elseif k == length(tr.times)
            tr.poses[end]
        else
            a = (p.t - tr.times[k]) / max(tr.times[k+1] - tr.times[k], 1)
            A, B = tr.poses[k], tr.poses[k+1]
            Rigid(rotation_matrix(slerp(Quat(A.R), Quat(B.R), a)), A.t + a * (B.t - A.t))
        end
        set_pose!(tr.marker, pose_matrix(T))
    end
end

"""
    tick!(player, dt) -> player

Advance playback by `dt` wall-clock seconds × `rate` (no-op while paused).
Stops at the end, or wraps when `player.loop` is set.
"""
function tick!(p::BagPlayer, dt::Real)
    p.playing || return p
    t = p.t + round(Int64, dt * p.rate * 1e9)
    if t >= p.t1
        if p.loop
            return seek!(p, p.t0)
        end
        p.playing = false
        t = p.t1
    end
    return advance_to!(p, t)
end

"One-line status for the HUD."
status(p::BagPlayer) = @sprintf("%s %.2f / %.2f s   x%g   frame %s   %d scans",
                                p.playing ? "PLAY " : "PAUSE", current_time(p), duration(p), p.rate,
                                p.fixed_frame, p.scans)

"""
    replay(path; kwargs...) -> Viewer

Open a ROS 2 bag and replay it in a viewer window (playing from the start).
Keyword arguments go to [`open_bag`](@ref); `title`, `width`, `height` to the viewer.
"""
function replay(path::AbstractString; title="SpatialRender replay", width=1280, height=800, block=true, kwargs...)
    p = open_bag(path; kwargs...)
    display(p.bag)
    play!(p)
    return open_viewer(p.scene; title, width, height, player=p, block)
end
