# Wavefront OBJ (positions, normals, texture coordinates, polygon faces).

"""
    load_obj(path; frame=IdentityFrame(), scale=1, name=basename(path)) -> TriMesh

Read a Wavefront OBJ. Polygons are fan-triangulated; each distinct
`v/vt/vn` combination becomes one GPU vertex. Missing normals are computed.
"""
function load_obj(path::AbstractString; frame::LocalFrame=IdentityFrame(), scale::Real=1.0,
                  name=basename(path), kwargs...)
    P = Vec3f[]; N = Vec3f[]; T = Vec2f[]
    lut = Dict{NTuple{3, Int}, UInt32}()
    verts = MeshVertex[]
    idx = UInt32[]
    has_normals = true
    resolve(i, n) = i < 0 ? n + 1 + i : i
    for line in eachline(path)
        s = strip(line)
        (isempty(s) || s[1] == '#') && continue
        t = split(s)
        if t[1] == "v"
            push!(P, to_local32(frame, scale .* Vec3d(parse.(Float64, t[2:4])...)))
        elseif t[1] == "vn"
            push!(N, Vec3f(parse.(Float32, t[2:4])...))
        elseif t[1] == "vt"
            push!(T, Vec2f(parse(Float32, t[2]), length(t) > 2 ? parse(Float32, t[3]) : 0f0))
        elseif t[1] == "f"
            corner = map(t[2:end]) do c
                parts = split(c, '/')
                vi = resolve(parse(Int, parts[1]), length(P))
                ti = length(parts) > 1 && !isempty(parts[2]) ? resolve(parse(Int, parts[2]), length(T)) : 0
                ni = length(parts) > 2 && !isempty(parts[3]) ? resolve(parse(Int, parts[3]), length(N)) : 0
                ni == 0 && (has_normals = false)
                key = (vi, ti, ni)
                get!(lut, key) do
                    push!(verts, MeshVertex(P[vi], ni > 0 ? N[ni] : Vec3f(0, 0, 1),
                                            ti > 0 ? T[ti] : Vec2f(0, 0)))
                    UInt32(length(verts) - 1)
                end
            end
            for k in 2:length(corner)-1
                push!(idx, corner[1], corner[k], corner[k+1])
            end
        end
    end
    m = TriMesh(verts, idx; name, kwargs...)
    has_normals || compute_normals!(m)
    return m
end
