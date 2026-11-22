# Picking: rays against meshes (BVH) and point clouds (chunk index).

"""
    RayHit

Result of a pick: distance `t` along the ray, hit `position`, the `layer`
hit, and the 1-based `index` of the triangle or point.
"""
struct RayHit
    t::Float32
    position::Vec3f
    layer::Union{Nothing, Layer}
    index::Int
end
const NO_HIT = RayHit(Inf32, Vec3f(NaN32, NaN32, NaN32), nothing, 0)
ishit(h::RayHit) = h.layer !== nothing

"""
    pick_point(pc, index, ray; angle=0.003) -> (t, point_index)

Closest point (along the ray) lying within the cone of half-angle `angle`
radians around `ray`, so the tolerance is a constant number of pixels at any
distance; pass `angle = pixels × fov / viewport_height`. Only chunks whose
boxes the cone can touch are scanned. `(Inf32, 0)` when nothing is found.
"""
function pick_point(pc::PointCloud, ix::PointIndex, r::Ray; angle::Real=0.003)
    tanα = Float32(tan(angle))
    best = Inf32; besti = 0
    ch = ix.chunks
    @inbounds for c in eachindex(ch)
        box = ch.box[c]
        # grow the box by the cone radius at its far side
        tfar = norm(center(box) - r.origin) + radius(box)
        ray_box(r, expand(box, tanα * tfar), best) == Inf32 && continue
        for k in ch.first[c]+1:ch.first[c]+ch.count[c]
            p = pc.vertices[k].position - r.origin
            t = dot(p, r.dir)
            (t <= 0 || t >= best) && continue
            dperp2 = dot(p, p) - t * t
            if dperp2 <= (tanα * t)^2
                best = t; besti = k
            end
        end
    end
    return best, besti
end

"""
    pick_layer(layer, accel, ray_local; tmax, angle) -> (t, index, position)

Closest hit of a ray already in the layer's frame: distance along that ray,
the 1-based triangle / point index, and the hit position (layer frame).
`(Inf32, 0, …)` on a miss. Each layer type adds a method; `accel` is its
acceleration structure (BVH, PointIndex) or whatever the renderer holds.
"""
function pick_layer end

function pick_layer(m::TriMesh, bvh::BVH, r::Ray; tmax=Inf32, angle=0.003)
    t, i = raycast(bvh, r; tmax)
    return t, i, r(t)
end

function pick_layer(pc::PointCloud, ix::PointIndex, r::Ray; tmax=Inf32, angle=0.003)
    t, i = pick_point(pc, ix, r; angle)
    return t < tmax ? (t, i, pc.vertices[i].position) : (Inf32, 0, r.origin)
end

"Closest point of `pts[range]` inside the pick cone (brute force; used at click time only)."
function pick_points_brute(pts::AbstractVector{PointVertex}, r::Ray, tanα::Float32, tmax::Float32)
    best = tmax; besti = 0
    @inbounds for k in eachindex(pts)
        p = pts[k].position - r.origin
        t = dot(p, r.dir)
        (t <= 0 || t >= best) && continue
        if dot(p, p) - t * t <= (tanα * t)^2
            best = t; besti = k
        end
    end
    return best, besti
end

# streaming layers: scan the live part of the ring (bounded, a few million points)
function pick_layer(sp::StreamingPointCloud, ::Any, r::Ray; tmax=Inf32, angle=0.003)
    n = length(sp)
    n == 0 && return (Inf32, 0, r.origin)
    ray_box(r, expand(bounds(sp), 0.5), Float32(tmax)) == Inf32 && return (Inf32, 0, r.origin)
    t, i = pick_points_brute(view(sp.buffer, 1:n), r, Float32(tan(angle)), Float32(tmax))
    return i == 0 ? (Inf32, 0, r.origin) : (t, i, sp.buffer[i].position)
end

"""
    raycast(items, ray; angle=0.003) -> RayHit

Closest hit among `(layer, accel)` pairs (see `SpatialRender.pick_layer`); layer
poses are applied, so the ray is given in the scene frame.
"""
function raycast(items::AbstractVector, r::Ray; angle::Real=0.003)
    hit = NO_HIT
    for (layer, accel) in items
        layer.visible || continue
        # ray in the layer's frame; `s` converts layer distances back to scene distances
        M = layer.model
        lr, s = M === IDENTITY4 ? (r, 1f0) : _local_ray(M, r)
        t, i, p = pick_layer(layer, accel, lr; tmax=hit.t * s, angle)
        if i != 0 && t / s < hit.t
            hit = RayHit(t / s, M === IDENTITY4 ? p : transform_point(M, p), layer, i)
        end
    end
    return hit
end

function _local_ray(M::Mat4f, r::Ray)
    Mi = inverse_pose(M)
    o = transform_point(Mi, r.origin)
    d = SMatrix{3, 3, Float32, 9}(Mi[1, 1], Mi[2, 1], Mi[3, 1], Mi[1, 2], Mi[2, 2], Mi[3, 2],
                                  Mi[1, 3], Mi[2, 3], Mi[3, 3]) * r.dir
    return Ray(o, d), norm(d)
end
