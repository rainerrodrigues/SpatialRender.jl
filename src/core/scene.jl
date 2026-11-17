"""
    Scene(; frame=IdentityFrame())

A collection of layers sharing one local frame. Loaders accept `frame=` so
data from different sources lands in the same coordinates; `scene.frame`
is the natural value to pass.
"""
mutable struct Scene
    frame::LocalFrame
    layers::Vector{Layer}
    version::Int          # bumped on every change so renderers re-upload
end
Scene(; frame::LocalFrame=IdentityFrame()) = Scene(frame, Layer[], 0)

layers(s::Scene) = s.layers
layers(s::Scene, ::Type{T}) where {T <: Layer} = (l for l in s.layers if l isa T)

"""
    add!(scene, layer) -> layer

Add a layer to the scene. Returns the layer so it can be tweaked afterwards.
"""
function add!(s::Scene, l::Layer)
    push!(s.layers, l)
    s.version += 1
    return l
end
add!(s::Scene, ls::Layer...) = (foreach(l -> add!(s, l), ls); s)

function Base.delete!(s::Scene, l::Layer)
    filter!(x -> x !== l, s.layers)
    s.version += 1
    return s
end

bounds(s::Scene) = reduce(union, (bounds(l) for l in s.layers if l.visible); init=AABB())

function Base.show(io::IO, ::MIME"text/plain", s::Scene)
    println(io, "Scene with ", length(s.layers), " layer", length(s.layers) == 1 ? "" : "s",
            " (frame: ", nameof(typeof(s.frame)), ")")
    for l in s.layers
        print(io, "  ", rpad(nameof(typeof(l)), 11), rpad(l.name, 24))
        l isa PointCloud && print(io, length(l), " points")
        l isa TriMesh && print(io, ntriangles(l), " triangles")
        l isa LineLayer && print(io, length(l.polylines), " polylines")
        println(io)
    end
end
