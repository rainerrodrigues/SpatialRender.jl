# Out-of-core octree conversion (Potree-style, simplified).
#
# Peak memory is one bucket, not the whole survey:
# Nodes split only along axes at least half as long as their longest axis, so
# flat survey data (km wide, tens of metres tall) becomes a quadtree until
# cells are as small as the height range: no split planes through terrain.
#
#   pass 1  count points per cell of a coarse bucket grid (streaming, mmap)
#   pass 2  distribute points into one temporary file per non-empty cell
#   pass 3  load one bucket at a time and build its subtree in memory; every
#           node keeps a grid-subsampled set of its points (at most one per
#           cell of a `spacing_div`³ grid) and passes the rest to its children
#   pass 4  build the coarse levels above the buckets by subsampling their
#           children (these few nodes duplicate some points; nothing is lost)
# Output: `nodes.bin` (PointVertex records, node after node) + `octree.json`.

const OCTREE_VERSION = 2          # 2: 20-byte PointVertex

struct BuildNode
    name::String
    depth::Int
    box::AABB
    offset::Int          # in points, into nodes.bin
    count::Int
    children::Vector{String}
end

"""
    build_octree(source, out_dir; node_capacity=50_000, bucket_points=4_000_000,
                 spacing_div=128, chunk=1_000_000, seed=1) -> out_dir

Convert a point cloud for out-of-core rendering with [`OctreeCloud`](@ref).
`source` is a LAS file path or a [`PointCloud`](@ref) (whose coordinates
are taken as already local).

- `node_capacity`: maximum points stored in one node.
- `bucket_points`: target points per on-disk bucket. Peak memory is about
  `bucket_points × 24` bytes plus write buffers.
- `spacing_div`: subsampling grid per node edge; larger keeps more points in
  coarse nodes.

The LAS path is read in `chunk`-point slices through a memory map, so files
larger than RAM convert. Coordinates are stored relative to the cloud's
centre (an `OffsetFrame` recorded in `octree.json`).
"""
function build_octree(source, out_dir::AbstractString; node_capacity::Int=50_000,
                      bucket_points::Int=4_000_000, spacing_div::Int=128,
                      chunk::Int=1_000_000, seed::Int=1)
    src = _octree_source(source, chunk)
    mkpath(out_dir)
    n = src.n
    n > 0 || error("source has no points")
    c = (src.lo + src.hi) / 2
    frame = OffsetFrame(c)
    pad = Vec3f(1e-3, 1e-3, 1e-3) .+ Float32(1e-4) .* Vec3f(src.hi - src.lo)
    root = AABB(Vec3f(src.lo - c) - pad, Vec3f(src.hi - c) + pad)
    # split table: which axes split at each level (identical for all nodes of a level)
    levels = NTuple{3, Bool}[]
    e = extent(root); cells = 1
    while cells < n / bucket_points && length(levels) < 8 && cells < 4096
        sp = split_axes(e)
        push!(levels, sp)
        cells *= 2^count(sp)
        e = e ./ Vec3f(sp[1] ? 2 : 1, sp[2] ? 2 : 1, sp[3] ? 2 : 1)
    end
    d = length(levels)
    bits = (count(l -> l[1], levels), count(l -> l[2], levels), count(l -> l[3], levels))
    G = (1 << bits[1], 1 << bits[2], 1 << bits[3])
    csize = extent(root) ./ Vec3f(G...)
    cellq(p) = clamp.(floor.(Int, (p - root.lo) ./ csize), 0, SVector(G...) .- 1)
    cellof(p) = (q = cellq(p); q[1] + G[1] * (q[2] + G[2] * q[3]) + 1)
    # pass 1: counts
    counts = zeros(Int, prod(G))
    src.foreach_chunk(frame) do pts
        for v in pts
            counts[cellof(v.position)] += 1
        end
    end
    # pass 2: distribute into bucket files with small write buffers
    tmp = mkpath(joinpath(out_dir, "tmp"))
    bufs = Dict{Int, Vector{PointVertex}}()
    flush_bucket(i) = (b = bufs[i]; isempty(b) || open(io -> write(io, b), joinpath(tmp, "b$i.bin"), "a"); empty!(b))
    src.foreach_chunk(frame) do pts
        for v in pts
            i = cellof(v.position)
            b = get!(() -> sizehint!(PointVertex[], 2048), bufs, i)
            push!(b, v)
            length(b) >= 2048 && flush_bucket(i)
        end
    end
    foreach(flush_bucket, keys(bufs))
    # pass 3: subtrees, written straight to nodes.bin
    rng = Xoshiro(seed)
    nodes = Dict{String, BuildNode}()
    data = open(joinpath(out_dir, "nodes.bin"), "w")
    written = Ref(0)
    occupied = falses(spacing_div^3)
    emit!(name, depth, box, pts, children) = begin
        write(data, pts)
        nodes[name] = BuildNode(name, depth, box, written[], length(pts), children)
        written[] += length(pts)
    end
    # one point buffer + one scratch buffer, reused for every bucket
    buf = PointVertex[]; scratch = PointVertex[]
    for i in findall(>(0), counts)
        path = joinpath(tmp, "b$i.bin")
        resize!(buf, counts[i]); resize!(scratch, counts[i])
        open(io -> read!(io, buf), path)
        rm(path)
        ci = i - 1
        q = (ci % G[1], (ci ÷ G[1]) % G[2], ci ÷ (G[1] * G[2]))
        name = "r" * _bucket_digits(q, levels, bits)
        lo = root.lo .+ Vec3f(q...) .* csize
        _build_subtree!(emit!, buf, scratch, 1, length(buf), name, d, AABB(lo, lo .+ csize),
                        node_capacity, spacing_div, occupied, rng)
    end
    empty!(buf); empty!(scratch); sizehint!(buf, 0); sizehint!(scratch, 0)
    # pass 4: coarse levels above the buckets, deepest first
    for depth in d-1:-1:0
        names = unique(nm[1:depth+1] for nm in keys(nodes) if length(nm) == depth + 2)
        for name in names
            kids = sort([nm for nm in keys(nodes) if length(nm) == depth + 2 && startswith(nm, name)])
            box = _parent_box(nodes[kids[1]].box, nodes[kids[1]].name, levels[depth+1])
            pool = PointVertex[]
            for k in kids
                append!(pool, _read_node(out_dir, data, nodes[k]))
            end
            shuffle!(rng, pool)
            k = _subsample!(pool, nothing, 1, length(pool), box, node_capacity, spacing_div, occupied)
            emit!(name, depth, box, view(pool, 1:k), kids)
        end
    end
    close(data)
    rm(tmp; recursive=true, force=true)
    _write_index(out_dir, nodes, n, frame, root, src, node_capacity, d, spacing_div)
    return out_dir
end

"Axes worth splitting: at least half as long as the longest one."
function split_axes(e)
    m = maximum(e)
    return (e[1] >= 0.5f0 * m, e[2] >= 0.5f0 * m, e[3] >= 0.5f0 * m)
end

# Octant digit of a child: one bit per axis, set only for split axes.
_octant(p, mid, sp) = (sp[1] & (p[1] >= mid[1])) | ((sp[2] & (p[2] >= mid[2])) << 1) |
                      ((sp[3] & (p[3] >= mid[3])) << 2)

function _child_box(b::AABB, o::Integer, sp)
    mid = center(b)
    lo = Vec3f(sp[1] ? (o & 1 == 1 ? mid[1] : b.lo[1]) : b.lo[1],
               sp[2] ? ((o >> 1) & 1 == 1 ? mid[2] : b.lo[2]) : b.lo[2],
               sp[3] ? ((o >> 2) & 1 == 1 ? mid[3] : b.lo[3]) : b.lo[3])
    e = extent(b) ./ Vec3f(sp[1] ? 2 : 1, sp[2] ? 2 : 1, sp[3] ? 2 : 1)
    return AABB(lo, lo + e)
end

function _parent_box(b::AABB, name, sp)
    o = parse(Int, name[end])
    e = extent(b)
    lo = b.lo - Vec3f(sp[1] ? (o & 1) * e[1] : 0, sp[2] ? ((o >> 1) & 1) * e[2] : 0,
                      sp[3] ? ((o >> 2) & 1) * e[3] : 0)
    return AABB(lo, lo + e .* Vec3f(sp[1] ? 2 : 1, sp[2] ? 2 : 1, sp[3] ? 2 : 1))
end

# Name digits of a bucket cell: walk the split table from the root.
function _bucket_digits(q, levels, bits)
    used = [0, 0, 0]
    io = IOBuffer()
    for sp in levels
        o = 0
        for a in 1:3
            sp[a] || continue
            used[a] += 1
            o |= ((q[a] >> (bits[a] - used[a])) & 1) << (a - 1)
        end
        print(io, o)
    end
    return String(take!(io))
end

# Points of an already-written node (pass 4 needs them while nodes.bin is open).
function _read_node(dir, io, nd::BuildNode)
    flush(io)
    v = Vector{PointVertex}(undef, nd.count)
    open(joinpath(dir, "nodes.bin")) do r
        seek(r, nd.offset * sizeof(PointVertex)); read!(r, v)
    end
    return v
end

# Keep the first point per occupied grid cell (points arrive shuffled, so the
# kept set is a uniform-ish sample), at most `cap`, compacted in place to the
# front of pts[lo:hi]. The rest go to scratch[1:m] (or are dropped when
# scratch === nothing). Returns the number kept.
function _subsample!(pts, scratch, lo, hi, box::AABB, cap, div, occupied)
    fill!(occupied, false)
    e = max.(extent(box), 1f-9)
    # cubic cells: `div` along the longest axis, proportionally fewer elsewhere
    nc = clamp.(round.(Int, div .* e ./ maximum(e)), 1, div)
    inv = Vec3f(nc) ./ e
    w = lo - 1; m = 0
    @inbounds for j in lo:hi
        v = pts[j]
        q = clamp.(floor.(Int, (v.position - box.lo) .* inv), 0, nc .- 1)
        k = q[1] + nc[1] * (q[2] + nc[2] * q[3]) + 1
        if !occupied[k] && w - lo + 1 < cap
            occupied[k] = true
            pts[w += 1] = v                     # w ≤ j: never overwrites an unread point
        elseif scratch !== nothing
            scratch[m += 1] = v
        end
    end
    return w - lo + 1
end

# Build the subtree of the points in pts[lo:hi] in place: subsample this node
# to the front of the range, counting-sort the rest by child octant behind it,
# recurse on each child range. Memory: the two buffers only.
function _build_subtree!(emit!, pts, scratch, lo, hi, name, depth, box, cap, div, occupied, rng)
    n = hi - lo + 1
    shuffle!(rng, view(pts, lo:hi))
    if n <= cap || depth >= 24
        emit!(name, depth, box, view(pts, lo:hi), String[])
        return
    end
    k = _subsample!(pts, scratch, lo, hi, box, cap, div, occupied)
    m = n - k                                   # rest, now in scratch[1:m]
    mid = center(box)
    sp = split_axes(extent(box))
    counts = zeros(Int, 8)
    @inbounds for j in 1:m
        counts[_octant(scratch[j].position, mid, sp)+1] += 1
    end
    starts = cumsum(counts) .- counts .+ (lo + k)
    next = copy(starts)
    @inbounds for j in 1:m
        o = _octant(scratch[j].position, mid, sp) + 1
        pts[next[o]] = scratch[j]; next[o] += 1
    end
    kids = [name * string(o - 1) for o in 1:8 if counts[o] > 0]
    emit!(name, depth, box, view(pts, lo:lo+k-1), kids)
    for o in 1:8
        counts[o] == 0 && continue
        _build_subtree!(emit!, pts, scratch, starts[o], starts[o] + counts[o] - 1, name * string(o - 1),
                        depth + 1, _child_box(box, o - 1, sp), cap, div, occupied, rng)
    end
end

function _write_index(dir, nodes, n, frame, root, src, cap, d, div)
    v3(x) = [Float64(x[1]), Float64(x[2]), Float64(x[3])]
    idx = Dict(
        "version" => OCTREE_VERSION, "points" => n, "node_capacity" => cap, "bucket_depth" => d,
        "origin" => v3(frame.origin), "root" => Dict("lo" => v3(root.lo), "hi" => v3(root.hi)),
        "bounds" => Dict("lo" => v3(src.lo - frame.origin), "hi" => v3(src.hi - frame.origin)),
        "has_rgb" => src.has_rgb, "spacing_div" => div,
        "nodes" => [Dict("name" => nd.name, "depth" => nd.depth, "offset" => nd.offset,
                         "count" => nd.count, "lo" => v3(nd.box.lo), "hi" => v3(nd.box.hi),
                         "children" => nd.children)
                    for nd in sort(collect(values(nodes)); by = x -> (length(x.name), x.name))])
    open(io -> JSON3.write(io, idx), joinpath(dir, "octree.json"), "w")
end

# --- sources ------------------------------------------------------------------

function _octree_source(path::AbstractString, chunk)
    data = Mmap.mmap(path)
    h = read_las_header(data)
    h.compressed && error("LAZ input: decompress to LAS first (laszip / pdal)")
    cls_off, rgb_off, gps_off = _las_layout(h.point_format)
    rl = h.record_length; base = h.offset_to_points
    imax = 1; rgbmax = 0
    for s in 1:chunk:h.npoints, i in s:min(s + chunk - 1, h.npoints)
        o = base + (i - 1) * rl
        imax = max(imax, Int(_rd(UInt16, data, o + 12)))
        if rgb_off >= 0
            rgbmax = max(rgbmax, Int(_rd(UInt16, data, o + rgb_off)))
        end
    end
    function foreach_chunk(f, frame)
        buf = Vector{PointVertex}(undef, chunk)
        for s in 1:chunk:h.npoints
            sel = s:min(s + chunk - 1, h.npoints)
            v = view(buf, 1:length(sel))
            _las_decode!(v, Float64[], data, sel, h, frame, base, rl, cls_off, rgb_off, -1, imax,
                         rgbmax > 255 ? 8 : 0)
            f(v)
        end
    end
    return (n=h.npoints, lo=h.lo, hi=h.hi, foreach_chunk=foreach_chunk, has_rgb=rgb_off >= 0 && rgbmax > 0)
end

function _octree_source(pc::PointCloud, chunk)
    b = bounds(pc)
    lo, hi = Vec3d(b.lo), Vec3d(b.hi)
    function foreach_chunk(f, frame)
        o = Vec3f(frame.origin)
        for s in 1:chunk:length(pc)
            f([with_position(v, v.position - o)
               for v in view(pc.vertices, s:min(s + chunk - 1, length(pc)))])
        end
    end
    return (n=length(pc), lo=lo, hi=hi, foreach_chunk=foreach_chunk,
            has_rgb=pc.color_mode == Int32(COLOR_RGB))
end
