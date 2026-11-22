# Vehicle / sensor trajectories.

"""
    trajectory(positions; frame=IdentityFrame(), times=nothing, colormap=:time,
               color=Vec4f(0.2, 0.8, 1, 1), width=3, name="trajectory") -> LineLayer

Turn a sequence of positions (a vector of 3-vectors, or a 3×N / N×3 matrix)
into a line layer. With `colormap=:time` (default) the path is shaded from
blue (start) to red (end) along `times` (or the sample index), which makes
loops and revisits easy to read. Use `colormap=:solid` for one `color`.
`max_gap` (in the units of `times`) breaks the line wherever consecutive
samples are further apart, so recording gaps don't show as straight jumps.

Positions go through `frame`, so GNSS fixes can be given as `(lon, lat, h)`
with an [`ENUFrame`](@ref).
"""
function trajectory(positions::AbstractVector; frame::LocalFrame=IdentityFrame(), times=nothing,
                    colormap::Symbol=:time, color=Vec4f(0.2, 0.8, 1, 1), width::Real=3.0,
                    name="trajectory", max_gap::Real=Inf)
    pts = [to_local32(frame, p) for p in positions]
    n = length(pts)
    n >= 2 || throw(ArgumentError("a trajectory needs at least two positions"))
    t = times === nothing ? collect(1.0:n) : Float64.(times)
    gap(i) = t[i+1] - t[i] > max_gap          # recording gap: don't draw a line across it
    if colormap == :solid
        lines = Polyline[]; start = 1
        for i in 1:n-1
            if gap(i)
                i > start && push!(lines, Polyline(pts[start:i], Vec4f(color)))
                start = i + 1
            end
        end
        n > start && push!(lines, Polyline(pts[start:n], Vec4f(color)))
        return LineLayer(lines; name, width)
    end
    t0, t1 = extrema(t)
    span = t1 > t0 ? t1 - t0 : 1.0
    # one polyline per segment so each segment carries its own color
    lines = [Polyline([pts[i], pts[i+1]], turbo(Float32((t[i] - t0) / span))) for i in 1:n-1 if !gap(i)]
    return LineLayer(lines; name, width)
end
function trajectory(m::AbstractMatrix{<:Real}; kwargs...)
    pts = size(m, 1) == 3 ? [Vec3d(m[:, i]) for i in axes(m, 2)] : [Vec3d(m[i, :]) for i in axes(m, 1)]
    trajectory(pts; kwargs...)
end

"""
    turbo(x) -> Vec4f

Google's Turbo colormap (polynomial approximation), `x ∈ [0, 1]`. The same
polynomial is used in the point shader for height / intensity coloring.
"""
function turbo(x::Real)
    x = clamp(Float32(x), 0f0, 1f0)
    r = 0.13572138f0 + x * (4.61539260f0 + x * (-42.66032258f0 + x * (132.13108234f0 + x * (-152.94239396f0 + x * 59.28637943f0))))
    g = 0.09140261f0 + x * (2.19418839f0 + x * (4.84296658f0 + x * (-14.18503333f0 + x * (4.27729857f0 + x * 2.82956604f0))))
    b = 0.10667330f0 + x * (12.64194608f0 + x * (-60.58204836f0 + x * (110.36276771f0 + x * (-89.90310912f0 + x * 27.34824973f0))))
    Vec4f(clamp(r, 0, 1), clamp(g, 0, 1), clamp(b, 0, 1), 1)
end
