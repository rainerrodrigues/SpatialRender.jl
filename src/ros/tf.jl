# A tf2-style frame tree: every frame has one parent and a time series of
# transforms (parent ← child); static transforms hold for all time.

"Rigid transform `p ↦ R p + t` (Float64; composed on the CPU, converted for the GPU)."
struct Rigid
    R::SMatrix{3, 3, Float64, 9}
    t::Vec3d
end
Rigid() = Rigid(SMatrix{3, 3, Float64, 9}(I), Vec3d(0, 0, 0))
Rigid(t::Vec3d, q::Quat) = Rigid(rotation_matrix(q), t)
Base.:*(a::Rigid, b::Rigid) = Rigid(a.R * b.R, a.R * b.t + a.t)
Base.inv(a::Rigid) = Rigid(a.R', -(a.R' * a.t))
(a::Rigid)(p) = a.R * Vec3d(p) + a.t
pose_matrix(a::Rigid) = Mat4f(a.R[1, 1], a.R[2, 1], a.R[3, 1], 0, a.R[1, 2], a.R[2, 2], a.R[3, 2], 0,
                              a.R[1, 3], a.R[2, 3], a.R[3, 3], 0, a.t[1], a.t[2], a.t[3], 1)

struct TFSeries
    parent::String
    times::Vector{Int64}
    trans::Vector{Vec3d}
    rots::Vector{Quat}
    static::Bool
end

"""
    FrameTree(; max_gap=2.0)

Frames and their transforms over time, filled from `/tf` and `/tf_static`
(see [`add_transform!`](@ref)) and queried with [`lookup_transform`](@ref).
Between two samples more than `max_gap` seconds apart (a recording gap), the
nearest sample is used instead of interpolating motion that wasn't recorded.
"""
struct FrameTree
    edges::Dict{String, TFSeries}        # child => series
    max_gap::Int64                       # ns: interpolate only between samples this close
end
FrameTree(; max_gap::Real=2.0) = FrameTree(Dict{String, TFSeries}(), round(Int64, max_gap * 1e9))

frames(tree::FrameTree) = union(Set(keys(tree.edges)), Set(e.parent for e in values(tree.edges)))
parent_frame(tree::FrameTree, f::AbstractString) = (e = get(tree.edges, f, nothing); e === nothing ? nothing : e.parent)

"""
    add_transform!(tree, t::StampedTransform; static=false)

Record `parent ← child` at `t.stamp`. Samples may arrive in any order; call
`SpatialRender.finalize!` once after loading to sort them. A frame that changes
parent keeps the first one (tf2 forbids reparenting) and a warning is shown.
"""
function add_transform!(tree::FrameTree, t::StampedTransform; static::Bool=false)
    e = get(tree.edges, t.child, nothing)
    if e === nothing
        e = tree.edges[t.child] = TFSeries(t.parent, Int64[], Vec3d[], Quat[], static)
    elseif e.parent != t.parent
        @warn "frame $(t.child) published with parents $(e.parent) and $(t.parent); keeping $(e.parent)" maxlog=1
        return tree
    end
    push!(e.times, static ? 0 : t.stamp); push!(e.trans, t.translation); push!(e.rots, t.rotation)
    return tree
end

"Sort every series by time (after loading out-of-order samples)."
function finalize!(tree::FrameTree)
    for e in values(tree.edges)
        p = sortperm(e.times)
        permute!(e.times, p); permute!(e.trans, p); permute!(e.rots, p)
    end
    return tree
end

# parent ← child at time t: interpolate between bracketing samples, hold at the ends
function _edge(e::TFSeries, t::Int64, max_gap::Int64)
    n = length(e.times)
    (e.static || n == 1) && return Rigid(e.trans[end], e.rots[end])
    k = searchsortedlast(e.times, t)
    k == 0 && return Rigid(e.trans[1], e.rots[1])
    k == n && return Rigid(e.trans[n], e.rots[n])
    t0, t1 = e.times[k], e.times[k+1]
    if t1 - t0 > max_gap                 # recording gap: use the nearest real sample, don't invent motion
        j = t - t0 <= t1 - t ? k : k + 1
        return Rigid(e.trans[j], e.rots[j])
    end
    a = t1 > t0 ? (t - t0) / (t1 - t0) : 0.0
    return Rigid(e.trans[k] + a * (e.trans[k+1] - e.trans[k]), slerp(e.rots[k], e.rots[k+1], a))
end

# root ← frame at time t, and the root's name
function _to_root(tree::FrameTree, f::String, t::Int64)
    T = Rigid(); cur = f; hops = 0
    while (e = get(tree.edges, cur, nothing)) !== nothing
        T = _edge(e, t, tree.max_gap) * T
        cur = e.parent
        (hops += 1) > 64 && error("frame tree has a cycle through $f")
    end
    return T, cur
end

"""
    lookup_transform(tree, target, source, t) -> Union{Rigid, Nothing}

Transform taking points in frame `source` to frame `target` at time `t`
(nanoseconds), or `nothing` when the frames are not connected.
"""
function lookup_transform(tree::FrameTree, target::AbstractString, source::AbstractString, t::Integer)
    target == source && return Rigid()
    Ts, rs = _to_root(tree, String(source), Int64(t))
    Tt, rt = _to_root(tree, String(target), Int64(t))
    rs == rt || return nothing
    return inv(Tt) * Ts
end

"Choose a fixed frame: `map`, else `odom`, else `world`, else the root of the largest tree."
function default_fixed_frame(tree::FrameTree)
    fs = frames(tree)
    for f in ("map", "world", "odom")
        f in fs && return f
    end
    isempty(fs) && return nothing
    roots = Dict{String, Int}()
    for f in fs
        _, r = _to_root(tree, f, Int64(0))
        roots[r] = get(roots, r, 0) + 1
    end
    return first(sort(collect(roots); by = p -> -p.second)).first
end
