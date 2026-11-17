# Point cloud preprocessing before GPU upload.
#
# Every filter computes a keep-mask (`keep_mask(filter, pc) -> BitVector`),
# so filters compose cheaply and the original cloud is never modified
# unless asked. `apply_filter(f, pc)` returns a new cloud with the kept points.

"Supertype of point cloud filters. Implement `keep_mask(f, pc)::BitVector`."
abstract type PointFilter end

function keep_mask end

"""
    apply_filter(f, pc) -> PointCloud

A new cloud holding the points `f` keeps.
"""
apply_filter(f::PointFilter, pc::PointCloud) = subset(pc, keep_mask(f, pc))
(f::PointFilter)(pc::PointCloud) = apply_filter(f, pc)

"""
    subset(pc, mask_or_indices) -> PointCloud
"""
function subset(pc::PointCloud, sel)
    PointCloud(pc.name, pc.vertices[sel], pc.point_size, pc.color_mode, pc.visible,
               isempty(pc.gps_time) ? Float64[] : pc.gps_time[sel])
end

"""
    FilterPipeline(filters...)

Run filters in order; each sees the output of the previous one.
"""
struct FilterPipeline <: PointFilter
    filters::Vector{PointFilter}
end
FilterPipeline(fs::PointFilter...) = FilterPipeline(collect(PointFilter, fs))
Base.:(|>)(a::PointFilter, b::PointFilter) = FilterPipeline(a, b)
function apply_filter(p::FilterPipeline, pc::PointCloud)
    for f in p.filters
        pc = apply_filter(f, pc)
    end
    return pc
end
function keep_mask(p::FilterPipeline, pc::PointCloud)
    idx = collect(1:length(pc))
    for f in p.filters
        m = keep_mask(f, subset(pc, idx))
        idx = idx[m]
    end
    mask = falses(length(pc)); mask[idx] .= true
    return mask
end

_kdtree(pc::PointCloud) = KDTree([v.position for v in pc.vertices]; leafsize=16)

# ---------------------------------------------------------------------------

"""
    StatisticalOutlierFilter(; k=8, nsigma=2.0)

Statistical outlier removal (as in PCL's `StatisticalOutlierRemoval`): for
each point, the mean distance to its `k` nearest neighbours; points whose
mean distance exceeds the global mean by more than `nsigma` standard
deviations are dropped. Removes isolated returns (dust, rain, multipath).
"""
Base.@kwdef struct StatisticalOutlierFilter <: PointFilter
    k::Int = 8
    nsigma::Float64 = 2.0
end
keep_mask(f::StatisticalOutlierFilter, pc::PointCloud) = statistical_outlier_mask(pc; f.k, f.nsigma)

function statistical_outlier_mask(pc::PointCloud; k::Int=8, nsigma::Real=2.0)
    n = length(pc)
    n <= k && return trues(n)
    pts = [v.position for v in pc.vertices]
    tree = KDTree(pts; leafsize=16)
    idxs, dists = knn(tree, pts, k + 1)   # +1: the point itself
    md = [sum(d) / k for d in dists]      # self-distance is 0
    μ, σ = mean(md), std(md)
    return md .<= μ + nsigma * σ
end

"""
    RadiusOutlierFilter(; radius, min_neighbors=4)

Drop points with fewer than `min_neighbors` other points within `radius`.
"""
Base.@kwdef struct RadiusOutlierFilter <: PointFilter
    radius::Float64
    min_neighbors::Int = 4
end
keep_mask(f::RadiusOutlierFilter, pc::PointCloud) = radius_outlier_mask(pc; f.radius, f.min_neighbors)

function radius_outlier_mask(pc::PointCloud; radius::Real, min_neighbors::Int=4)
    pts = [v.position for v in pc.vertices]
    tree = KDTree(pts; leafsize=16)
    mask = falses(length(pts))
    Threads.@threads for i in eachindex(pts)
        mask[i] = length(inrange(tree, pts[i], radius)) - 1 >= min_neighbors
    end
    return mask
end

"""
    VoxelDownsample(voxel)

Keep one point per `voxel`-sized cube (the point nearest the voxel centre),
so attributes stay real sensor values rather than averages.
"""
struct VoxelDownsample <: PointFilter
    voxel::Float64
end
keep_mask(f::VoxelDownsample, pc::PointCloud) = voxel_downsample(pc, f.voxel)

function voxel_downsample(pc::PointCloud, voxel::Real)
    voxel > 0 || throw(ArgumentError("voxel size must be positive"))
    inv = 1 / voxel
    best = Dict{NTuple{3, Int32}, Tuple{Int, Float32}}()
    for (i, v) in enumerate(pc.vertices)
        q = v.position .* Float32(inv)
        key = (floor(Int32, q[1]), floor(Int32, q[2]), floor(Int32, q[3]))
        d = sum(abs2, q .- (Vec3f(key...) .+ 0.5f0))
        cur = get(best, key, (0, Inf32))
        d < cur[2] && (best[key] = (i, d))
    end
    mask = falses(length(pc))
    for (i, _) in values(best)
        mask[i] = true
    end
    return mask
end

"""
    GroundPlaneFilter(; threshold=0.15, iterations=200, max_tilt=15, mode=:remove, seed=1)

Fit the dominant near-horizontal plane with RANSAC (normal within `max_tilt`
degrees of +Z) and treat points within `threshold` metres of it as ground.

- `mode=:remove` drops ground points (obstacle / structure views);
- `mode=:keep` keeps only ground points;
- `mode=:classify` keeps everything and sets ASPRS class 2 (ground) on the
  inliers, so the classification color mode shows them.
"""
Base.@kwdef struct GroundPlaneFilter <: PointFilter
    threshold::Float64 = 0.15
    iterations::Int = 200
    max_tilt::Float64 = 15.0
    mode::Symbol = :remove
    seed::Int = 1
end

function keep_mask(f::GroundPlaneFilter, pc::PointCloud)
    g = ground_mask(pc; f.threshold, f.iterations, f.max_tilt, f.seed)
    f.mode == :remove && return .!g
    f.mode == :keep && return g
    f.mode == :classify && return trues(length(pc))
    throw(ArgumentError("GroundPlaneFilter mode must be :remove, :keep or :classify"))
end
function apply_filter(f::GroundPlaneFilter, pc::PointCloud)
    f.mode == :classify || return subset(pc, keep_mask(f, pc))
    g = ground_mask(pc; f.threshold, f.iterations, f.max_tilt, f.seed)
    out = subset(pc, trues(length(pc)))
    for i in findall(g)
        v = out.vertices[i]
        out.vertices[i] = PointVertex(v.position, v.intensity, v.rgba, 0x02, v.flags, v._pad)
    end
    return out
end

"""
    fit_ground_plane(pc; threshold=0.15, iterations=200, max_tilt=15, seed=1) -> (n, d)

RANSAC plane `n ⋅ p + d = 0` with unit normal `n` pointing up, refined by
least squares on the inliers. Returns `nothing` when no plane is found.
"""
function fit_ground_plane(pc::PointCloud; threshold::Real=0.15, iterations::Int=200,
                          max_tilt::Real=15.0, seed::Int=1)
    pts = [Vec3d(v.position) for v in pc.vertices]
    n = length(pts)
    n < 3 && return nothing
    rng = Xoshiro(seed)
    # score on a sample for speed on large clouds
    sample = n > 50_000 ? pts[rand(rng, 1:n, 50_000)] : pts
    cosmax = cosd(max_tilt)
    best, bestscore = nothing, -1
    for _ in 1:iterations
        a, b, c = pts[rand(rng, 1:n)], pts[rand(rng, 1:n)], pts[rand(rng, 1:n)]
        nn = cross(b - a, c - a)
        l = norm(nn)
        l < 1e-9 && continue
        nn = nn / l
        nn[3] < 0 && (nn = -nn)
        nn[3] < cosmax && continue
        d = -dot(nn, a)
        score = count(p -> abs(dot(nn, p) + d) <= threshold, sample)
        score > bestscore && ((best, bestscore) = ((nn, d), score))
    end
    best === nothing && return nothing
    # least-squares refinement: z = αx + βy + γ on inliers
    nn, d = best
    inl = [p for p in pts if abs(dot(nn, p) + d) <= threshold]
    length(inl) < 3 && return best
    A = [hcat([p[1] for p in inl], [p[2] for p in inl]) ones(length(inl))]
    α, β, γ = A \ [p[3] for p in inl]
    nr = normalize(Vec3d(-α, -β, 1))
    return (nr, -γ * nr[3])
end

"""
    ground_mask(pc; kwargs...) -> BitVector

Points within `threshold` of the RANSAC ground plane (see [`fit_ground_plane`](@ref)).
"""
function ground_mask(pc::PointCloud; threshold::Real=0.15, kwargs...)
    pl = fit_ground_plane(pc; threshold, kwargs...)
    pl === nothing && return falses(length(pc))
    nn, d = pl
    return BitVector([abs(dot(nn, Vec3d(v.position)) + d) <= threshold for v in pc.vertices])
end

"""
    CropBox(lo, hi; invert=false)

Keep points inside (or with `invert=true`, outside) an axis-aligned box in
the local frame, e.g. to cut the ego-vehicle out of a scan.
"""
struct CropBox <: PointFilter
    box::AABB
    invert::Bool
end
CropBox(lo::AbstractVector, hi::AbstractVector; invert::Bool=false) = CropBox(AABB(Vec3f(lo), Vec3f(hi)), invert)
keep_mask(f::CropBox, pc::PointCloud) =
    BitVector([contains_point(f.box, v.position) ⊻ f.invert for v in pc.vertices])

"""
    AttributeOutlierFilter(; attributes=(:z, :intensity), method="mad", threshold=nothing, window=7)

One-dimensional outlier rejection on per-point attributes (`:x`, `:y`, `:z`,
`:intensity`); a point is dropped when any listed attribute is an outlier.
Methods: `"zscore"`, `"iqr"`, `"mad"` (default, robust) and `"hampel"`
(a moving-window MAD along acquisition order, GPS time when present, which
follows a drifting signal: intensity spikes along a scan line).

The statistics are computed by the `KaosCleaner` engine: `using KaosCleaner`
enables this filter (package extension).
"""
Base.@kwdef struct AttributeOutlierFilter <: PointFilter
    attributes::Vector{Symbol} = [:z, :intensity]
    method::String = "mad"
    threshold::Union{Nothing, Float64} = nothing
    window::Int = 7
end

function keep_mask(f::AttributeOutlierFilter, pc::PointCloud)
    ext = Base.get_extension(@__MODULE__, :SpatialRenderKaosCleanerExt)
    ext === nothing && error("AttributeOutlierFilter needs the KaosCleaner engine: run `using KaosCleaner` first")
    return ext.attribute_keep_mask(f, pc)
end

"Column of one point attribute as Float64."
function attribute_column(pc::PointCloud, a::Symbol)
    a === :x && return Float64[v.position[1] for v in pc.vertices]
    a === :y && return Float64[v.position[2] for v in pc.vertices]
    a === :z && return Float64[v.position[3] for v in pc.vertices]
    a === :intensity && return Float64[v.intensity for v in pc.vertices]
    throw(ArgumentError("unknown point attribute :$a (use :x, :y, :z or :intensity)"))
end
