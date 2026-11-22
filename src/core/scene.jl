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
    lock(UPDATE_LOCK) do
        push!(s.layers, l)
        s.version += 1
    end
    return l
end
add!(s::Scene, ls::Layer...) = (foreach(l -> add!(s, l), ls); s)

function Base.delete!(s::Scene, l::Layer)
    lock(UPDATE_LOCK) do
        filter!(x -> x !== l, s.layers)
        s.version += 1
    end
    return s
end

"Bounds of all visible layers in the scene frame (poses applied)."
bounds(s::Scene) = reduce(union, (world_bounds(l) for l in s.layers if l.visible); init=AABB())

# One-line summaries: printing a layer must not dump its vertices.
"Short description of a layer's contents (each layer type adds a method)."
layer_summary(l::Layer) = ""
layer_summary(l::PointCloud) = "$(length(l)) points"
layer_summary(l::TriMesh) = "$(ntriangles(l)) triangles"
layer_summary(l::LineLayer) = "$(length(l.polylines)) polylines"

Base.show(io::IO, l::Layer) = print(io, nameof(typeof(l)), " \"", l.name, "\" (", layer_summary(l), ")")

function Base.show(io::IO, ::MIME"text/plain", s::Scene)
    println(io, "Scene with ", length(s.layers), " layer", length(s.layers) == 1 ? "" : "s",
            " (frame: ", nameof(typeof(s.frame)), ")")
    for l in s.layers
        println(io, "  ", rpad(nameof(typeof(l)), 20), " ", rpad(l.name, 34), " ", layer_summary(l))
    end
end
