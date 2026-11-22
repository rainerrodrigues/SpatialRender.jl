# STL (binary and ASCII).

"""
    load_stl(path; smooth=false, frame=IdentityFrame(), name=basename(path), scale=1) -> TriMesh

Read a binary or ASCII STL file.

With `smooth=false` (the default, right for CAD parts) every facet keeps its
own three vertices and the facet normal, so hard edges stay hard. With
`smooth=true` coincident vertices are welded and normals averaged.
`scale` converts units (e.g. `scale=1e-3` for millimetre CAD into metres).
`up=:y` converts a Y-up model (common from 3D modelling tools) to the
scene's Z-up convention.
"""
function load_stl(path::AbstractString; smooth::Bool=false, frame::LocalFrame=IdentityFrame(),
                  name=basename(path), scale::Real=1.0, up::Symbol=:z, kwargs...)
    tris = open(path) do io
        is_binary_stl(path) ? read_binary_stl(io) : read_ascii_stl(io)
    end
    if _up_check(up) == :y
        tris = [map(y_up_to_z_up, t) for t in tris]
    end
    return build_trimesh(tris, frame, Float64(scale), smooth; name, kwargs...)
end

function is_binary_stl(path)
    sz = filesize(path)
    sz < 84 && return false
    open(path) do io
        head = read(io, 80)
        n = read(io, UInt32)
        sz == 84 + 50 * Int(n) && return true
        # ASCII files start with "solid"; some binary exporters also write
        # "solid" in the header, which the size check above already caught.
        return !startswith(String(copy(head)), "solid")
    end
end

# A raw facet in source coordinates.
const RawTri = NTuple{4, Vec3d}   # normal, a, b, c

function read_binary_stl(io::IO)
    skip(io, 80)
    n = Int(read(io, UInt32))
    buf = read(io, 50 * n)
    length(buf) == 50 * n || error("truncated binary STL: expected $n facets")
    tris = Vector{RawTri}(undef, n)
    GC.@preserve buf begin
        p = pointer(buf)
        @inbounds for i in 0:n-1
            f = Ptr{Float32}(p + 50i)
            v(k) = Vec3d(unsafe_load(f, 3k + 1), unsafe_load(f, 3k + 2), unsafe_load(f, 3k + 3))
            tris[i+1] = (v(0), v(1), v(2), v(3))
        end
    end
    return tris
end

function read_ascii_stl(io::IO)
    tris = RawTri[]
    normal = Vec3d(0, 0, 0)
    verts = Vec3d[]
    for line in eachline(io)
        t = split(line)
        isempty(t) && continue
        if t[1] == "facet" && length(t) >= 5
            normal = Vec3d(parse.(Float64, t[3:5])...)
            empty!(verts)
        elseif t[1] == "vertex"
            push!(verts, Vec3d(parse.(Float64, t[2:4])...))
        elseif t[1] == "endfacet"
            length(verts) == 3 || error("ASCII STL facet with $(length(verts)) vertices")
            push!(tris, (normal, verts[1], verts[2], verts[3]))
        end
    end
    return tris
end

function build_trimesh(tris::Vector{RawTri}, frame, scale, smooth; kwargs...)
    n = length(tris)
    if !smooth
        verts = Vector{MeshVertex}(undef, 3n)
        idx = Vector{UInt32}(undef, 3n)
        @inbounds for (t, (nrm, a, b, c)) in enumerate(tris)
            pa, pb, pc = (to_local32(frame, scale * x) for x in (a, b, c))
            fn = cross(pb - pa, pc - pa)
            l = norm(fn)
            fnn = l > 0 ? fn / l : (norm(nrm) > 0 ? Vec3f(normalize(nrm)) : Vec3f(0, 0, 1))
            for (k, p) in enumerate((pa, pb, pc))
                verts[3(t-1)+k] = MeshVertex(p, fnn)
                idx[3(t-1)+k] = UInt32(3(t-1) + k - 1)
            end
        end
        return TriMesh(verts, idx; kwargs...)
    end
    # weld identical positions
    lut = Dict{Vec3f, UInt32}()
    verts = MeshVertex[]
    idx = Vector{UInt32}(undef, 3n)
    for (t, (_, a, b, c)) in enumerate(tris), (k, x) in enumerate((a, b, c))
        p = to_local32(frame, scale * x)
        i = get!(lut, p) do
            push!(verts, MeshVertex(p, Vec3f(0, 0, 1)))
            UInt32(length(verts) - 1)
        end
        idx[3(t-1)+k] = i
    end
    return compute_normals!(TriMesh(verts, idx; kwargs...))
end

"""
    save_stl(path, mesh)

Write a binary STL (used by tests and for exporting cleaned meshes).
"""
function save_stl(path::AbstractString, m::TriMesh)
    open(path, "w") do io
        write(io, rpad("binary STL written by SpatialRender.jl", 80)[1:80])
        write(io, UInt32(ntriangles(m)))
        for t in 1:ntriangles(m)
            a, b, c = triangle(m, t)
            n = cross(b - a, c - a)
            n = norm(n) > 0 ? normalize(n) : n
            for v in (n, a, b, c)
                write(io, Float32(v[1]), Float32(v[2]), Float32(v[3]))
            end
            write(io, UInt16(0))
        end
    end
    return path
end
