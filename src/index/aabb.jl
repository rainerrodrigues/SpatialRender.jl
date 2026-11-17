# Ray / box primitives shared by the BVH, the point index and picking.

"""
    Ray(origin, direction)

A ray in the local frame. `direction` is normalised; `inv` caches its
reciprocal for slab tests.
"""
struct Ray
    origin::Vec3f
    dir::Vec3f
    inv::Vec3f
end
function Ray(o, d)
    d = normalize(Vec3f(d))
    Ray(Vec3f(o), d, Vec3f(1 / d[1], 1 / d[2], 1 / d[3]))
end
(r::Ray)(t) = r.origin + t * r.dir

"""
    ray_box(ray, box, tmax=Inf32) -> Float32

Entry distance of `ray` into `box` (0 when the origin is inside), or `Inf32`
when it misses or enters beyond `tmax`.
"""
@inline function ray_box(r::Ray, b::AABB, tmax::Float32=Inf32)
    t1 = (b.lo - r.origin) .* r.inv
    t2 = (b.hi - r.origin) .* r.inv
    tn = maximum(min.(t1, t2))
    tf = minimum(max.(t1, t2))
    # NaN (0 * Inf on a slab boundary) compares false and is treated as a miss
    return (tf >= max(tn, 0f0)) & (tn <= tmax) ? max(tn, 0f0) : Inf32
end

expand(b::AABB, e::Real) = AABB(b.lo .- Float32(e), b.hi .+ Float32(e))
