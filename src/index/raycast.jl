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
    raycast(scene_items, ray) -> RayHit

Closest hit among `(layer, accel)` pairs where `accel` is a [`BVH`](@ref)
for meshes or a [`PointIndex`](@ref) for clouds.
"""
function raycast(items::AbstractVector, r::Ray; angle::Real=0.003)
    hit = NO_HIT
    for (layer, accel) in items
        layer.visible || continue
        if accel isa BVH
            t, i = raycast(accel, r; tmax=hit.t)
            t < hit.t && (hit = RayHit(t, r(t), layer, i))
        elseif accel isa PointIndex
            t, i = pick_point(layer, accel, r; angle)
            t < hit.t && (hit = RayHit(t, layer.vertices[i].position, layer, i))
        end
    end
    return hit
end
