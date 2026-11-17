# View-frustum culling.

"""
    Frustum

Six planes `(n, d)` with `n ⋅ p + d >= 0` inside, extracted from a
view-projection matrix (Gribb–Hartmann).
"""
struct Frustum
    planes::NTuple{6, Vec4f}
end

"""
    frustum_from_matrix(vp) -> Frustum

Planes of the clip volume of the OpenGL-convention view-projection matrix
`vp` (clip z in [-w, w]).
"""
function frustum_from_matrix(m::AbstractMatrix)
    r(i) = Vec4f(m[i, 1], m[i, 2], m[i, 3], m[i, 4])
    r1, r2, r3, r4 = r(1), r(2), r(3), r(4)
    nrm(p) = p / norm(Vec3f(p[1], p[2], p[3]))
    Frustum((nrm(r4 + r1), nrm(r4 - r1), nrm(r4 + r2), nrm(r4 - r2), nrm(r4 + r3), nrm(r4 - r3)))
end

"""
    intersects(frustum, box) -> Bool

Conservative test: false only when `box` is entirely outside one plane.
"""
@inline function intersects(f::Frustum, b::AABB)
    @inbounds for p in f.planes
        n = Vec3f(p[1], p[2], p[3])
        # the box corner furthest along the plane normal
        v = Vec3f(ifelse(n[1] >= 0, b.hi[1], b.lo[1]),
                  ifelse(n[2] >= 0, b.hi[2], b.lo[2]),
                  ifelse(n[3] >= 0, b.hi[3], b.lo[3]))
        dot(n, v) + p[4] < 0 && return false
    end
    return true
end
intersects(f::Frustum, p::Vec3f) = all(dot(Vec3f(q[1], q[2], q[3]), p) + q[4] >= 0 for q in f.planes)

"""
    cull!(index, frustum; eye, pixels_per_radian=Inf, density=1.0, min_points=256) -> VisibleRanges

Fill `index.visible` with the draw ranges of chunks inside `frustum`.

Level of detail: a chunk whose bounding sphere covers `A` pixels on screen
(computed from the distance to `eye` and `pixels_per_radian`) draws at most
`density × A` of its points, never fewer than `min_points`. Because chunks
are shuffled, the drawn prefix is a uniform subsample. Pass
`pixels_per_radian=Inf` to always draw every point.

Adjacent full chunks are merged into one range. Allocation-free.
"""
function cull!(ix::PointIndex, f::Frustum; eye::Vec3f=Vec3f(0, 0, 0),
               pixels_per_radian::Real=Inf, density::Real=1.0, min_points::Integer=256)
    vis = ix.visible
    firsts = vis.ranges.first; counts = vis.ranges.count
    n = 0; total = 0
    ch = ix.chunks
    ppr = Float32(pixels_per_radian); dens = Float32(density)
    @inbounds for i in eachindex(ch)
        box = ch.box[i]
        intersects(f, box) || continue
        c = ch.count[i]
        if isfinite(ppr)
            r = radius(box)
            d = max(norm(center(box) - eye) - r, 1f-3)
            rpx = ppr * r / d
            want = dens * Float32(π) * rpx * rpx
            want < c && (c = Int32(clamp(ceil(want), min(min_points, c), c)))
        end
        first = ch.first[i]
        # Contiguous with the previous range only if that one ended on a full
        # chunk, so merging never skips points.
        if n > 0 && firsts[n] + counts[n] == first
            counts[n] += c
        else
            n += 1
            firsts[n] = first; counts[n] = c
        end
        total += c
    end
    vis.n = n; vis.npoints = total
    return vis
end
