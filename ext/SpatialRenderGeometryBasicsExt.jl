# GeometryBasics meshes → TriMesh, and FileIO/MeshIO as the fallback loader for
# mesh formats without a native reader (PLY, OFF, 2DM, MSH, …).
module SpatialRenderGeometryBasicsExt

using SpatialRender
using SpatialRender: MeshVertex, TriMesh, IdentityFrame, LocalFrame, Vec3d, Vec3f, Vec2f,
                     to_local32, compute_normals!, MESH_FALLBACK
import GeometryBasics
import FileIO

"""
    TriMesh(mesh::GeometryBasics.Mesh; frame=IdentityFrame(), scale=1, kwargs...)

Convert a GeometryBasics mesh (any polygon faces, optional per-vertex
normals and uvs) into a GPU-ready `TriMesh`.
"""
function SpatialRender.TriMesh(m::GeometryBasics.Mesh; frame::LocalFrame=IdentityFrame(),
                               scale::Real=1.0, kwargs...)
    m = GeometryBasics.expand_faceviews(m)
    pos = GeometryBasics.coordinates(m)
    attrs = GeometryBasics.vertex_attributes(m)
    nrm = get(attrs, :normal, nothing)
    uv = get(attrs, :uv, nothing)
    verts = Vector{MeshVertex}(undef, length(pos))
    for i in eachindex(pos)
        p = pos[i]
        P = to_local32(frame, scale .* Vec3d(p[1], p[2], length(p) > 2 ? p[3] : 0.0))
        N = nrm === nothing ? Vec3f(0, 0, 1) : Vec3f(nrm[i]...)
        T = uv === nothing ? Vec2f(0, 0) : Vec2f(uv[i][1], uv[i][2])
        verts[i] = MeshVertex(P, N, T)
    end
    tris = GeometryBasics.decompose(GeometryBasics.GLTriangleFace, m)
    idx = Vector{UInt32}(undef, 3length(tris))
    for (t, f) in enumerate(tris), k in 1:3
        idx[3(t-1)+k] = UInt32(GeometryBasics.value(f[k]) - 1)   # 1-based face → 0-based GL index
    end
    out = TriMesh(verts, idx; kwargs...)
    nrm === nothing && compute_normals!(out)
    return out
end

function __init__()
    MESH_FALLBACK[] = (path; kwargs...) -> TriMesh(FileIO.load(path); name=basename(path), kwargs...)
end

end
