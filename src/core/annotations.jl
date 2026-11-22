# Text labels and measurements.

"""
    Label(position, text; color=white, size=13, marker=true)

Screen-aligned text anchored at a 3-D `position`; `size` is the cap height
in pixels. `marker` draws a small × at the anchor.
"""
struct Label
    position::Vec3f
    text::String
    color::Vec4f
    size::Float32
    marker::Bool
end
Label(p, text; color=Vec4f(1, 1, 1, 1), size=13, marker=true) =
    Label(Vec3f(p), String(text), Vec4f(color), Float32(size), marker)

"""
    Annotations(; name="annotations", width=2)

A layer of labels and measurement lines, drawn on top of the scene (never
hidden by geometry). Add to it with [`annotate!`](@ref) and [`measure!`](@ref);
[`undo!`](@ref) removes the last addition.
"""
mutable struct Annotations <: Layer
    name::String
    lines::Vector{Polyline}
    labels::Vector{Label}
    groups::Vector{NTuple{2, Int}}     # (#lines, #labels) added per call, for undo!
    width::Float32
    visible::Bool
    model::Mat4f
    rev::Int
end
Annotations(; name="annotations", width::Real=2.0) =
    Annotations(String(name), Polyline[], Label[], NTuple{2, Int}[], Float32(width), true, IDENTITY4, 0)

function bounds(a::Annotations)
    b = AABB()
    for p in a.lines, q in p.points
        b = union(b, q)
    end
    for l in a.labels
        b = union(b, l.position)
    end
    return b
end

function _record!(a::Annotations, nl, nlab)
    push!(a.groups, (nl, nlab))
    a.rev += 1
    return a
end

"""
    annotate!(ann, position, text; color, size=13, marker=true) -> ann

Add a text label at `position` (layer frame).
"""
function annotate!(a::Annotations, p, text::AbstractString; kwargs...)
    lock(UPDATE_LOCK) do
        push!(a.labels, Label(p, text; kwargs...))
        _record!(a, 0, 1)
    end
    return a
end

"""
    measure!(ann, p, q; color=yellow) -> (distance, dz, slope_degrees)

Add a measurement from `p` to `q`: the segment, its 3-D length, the height
difference and the slope, labelled at the midpoint.
"""
function measure!(a::Annotations, p, q; color=Vec4f(1, 0.85, 0.2, 1))
    P, Q = Vec3f(p), Vec3f(q)
    d = norm(Q - P)
    dz = Q[3] - P[3]
    horiz = hypot(Q[1] - P[1], Q[2] - P[2])
    slope = rad2deg(atan(dz, horiz))
    text = @sprintf("%.3f m\nΔz %+.3f m  %.1f°", d, dz, slope)
    lock(UPDATE_LOCK) do
        push!(a.lines, Polyline([P, Q], Vec4f(color)))
        push!(a.labels, Label(P, ""; color), Label(Q, ""; color))      # markers under the text
        push!(a.labels, Label((P + Q) / 2, text; color, marker=false))
        _record!(a, 1, 3)
    end
    return (d, dz, slope)
end

"Remove the most recent `annotate!` / `measure!` addition."
function undo!(a::Annotations)
    lock(UPDATE_LOCK) do
        isempty(a.groups) && return
        nl, nlab = pop!(a.groups)
        resize!(a.lines, length(a.lines) - nl)
        resize!(a.labels, length(a.labels) - nlab)
        a.rev += 1
    end
    return a
end

function Base.empty!(a::Annotations)
    lock(UPDATE_LOCK) do
        empty!(a.lines); empty!(a.labels); empty!(a.groups); a.rev += 1
    end
    return a
end

layer_summary(a::Annotations) = "$(length(a.labels)) labels, $(length(a.lines)) lines"
