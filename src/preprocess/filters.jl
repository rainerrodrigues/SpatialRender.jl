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
    k >= 1 || throw(ArgumentError("k must be at least 1"))
    n = length(pc)
    n <= k && return trues(n)
    g = GridNeighbors(Positions(pc.vertices))
    md = _mean_knn_distance(g, pc.vertices, k)        # 4 bytes per point
    μ = mean(md); σ = std(md; mean=μ)
    return md .<= μ + nsigma * σ
end

function _mean_knn_distance(g::GridNeighbors, verts::Vector{PointVertex}, k::Int)
    md = Vector{Float32}(undef, length(verts))
    threaded_chunks(_mean_knn_kernel!, length(verts), md, g, verts, k)
    return md
end

function _mean_knn_kernel!(range, md, g, verts, k)
    h = KHeap(k)                                     # one small heap per chunk
    # knn! is @inline: as a separate call, Julia 1.12 boxed its arguments
    # (~130 B per point) whenever this kernel ran in spawned tasks
    for i in range
        knn!(h, g, verts[i].position, i)
        acc = 0f0
        for j in 1:h.n
            acc += sqrt(h.d[j])
        end
        md[i] = acc / k
    end
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
    radius > 0 || throw(ArgumentError("radius must be positive"))
    min_neighbors >= 0 || throw(ArgumentError("min_neighbors must be non-negative"))
    g = GridNeighbors(Positions(pc.vertices), Float32(radius))
    return BitVector(_radius_keep(g, pc.vertices, Float32(radius), min_neighbors))
end

# Vector{Bool}, not BitVector: threads writing neighbouring bits of one word would race.
function _radius_keep(g::GridNeighbors, verts::Vector{PointVertex}, r::Float32, m::Int)
    keep = Vector{Bool}(undef, length(verts))
    threaded_chunks(_radius_kernel!, length(verts), keep, g, verts, r, m)
    return keep
end

function _radius_kernel!(range, keep, g, verts, r, m)
    @inbounds for i in range
        keep[i] = count_within(g, verts[i].position, r, i, m) >= m
    end
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
    inv = Float32(1 / voxel)
    # one packed key per occupied voxel → index of the point nearest its centre
    best = Dict{UInt64, Int32}()
    dist(i, q) = sum(abs2, pc.vertices[i].position .* inv .- (Vec3f(q) .+ 0.5f0))
    for (i, v) in enumerate(pc.vertices)
        q = floor.(Int32, v.position .* inv)
        key = (UInt64(reinterpret(UInt32, q[1]) & 0x1fffff) << 42) |
              (UInt64(reinterpret(UInt32, q[2]) & 0x1fffff) << 21) | UInt64(reinterpret(UInt32, q[3]) & 0x1fffff)
        j = get(best, key, Int32(0))
        (j == 0 || dist(i, q) < dist(j, q)) && (best[key] = Int32(i))
    end
    mask = falses(length(pc))
    for i in values(best)
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
        out.vertices[i] = with_class(v, 2)
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
    threshold > 0 || throw(ArgumentError("threshold must be positive"))
    iterations >= 1 || throw(ArgumentError("iterations must be at least 1"))
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
enables this filter. KaosCleaner is not registered, so it is found at run time
instead of being a declared dependency; install it with
`Pkg.add(url = "https://github.com/rainerrodrigues/KaosCleaner", subdir = "julia/KaosCleaner.jl")`.
"""
Base.@kwdef struct AttributeOutlierFilter <: PointFilter
    attributes::Vector{Symbol} = [:z, :intensity]
    method::String = "mad"
    threshold::Union{Nothing, Float64} = nothing
    window::Int = 7
end

# KaosCleaner (unregistered, so not in Project.toml: General refuses unregistered
# dependencies, weak ones included) is used once the user has loaded it.
const KAOSCLEANER = Base.PkgId(Base.UUID("33cd8d56-0019-45b6-a585-d49e6d89a734"), "KaosCleaner")
const OUTLIER_METHODS = ("zscore", "iqr", "mad", "hampel")

function _kaoscleaner()
    m = get(Base.loaded_modules, KAOSCLEANER, nothing)
    m === nothing && error("AttributeOutlierFilter needs the KaosCleaner engine: run `using KaosCleaner` first ",
                           "(install with `Pkg.add(url = \"https://github.com/rainerrodrigues/KaosCleaner\", subdir = \"julia/KaosCleaner.jl\")`)")
    return m
end

# A point survives when none of the selected attribute columns flags it. Hampel
# windows follow GPS time when the cloud has it (scan order otherwise).
function keep_mask(f::AttributeOutlierFilter, pc::PointCloud)
    f.method in OUTLIER_METHODS ||
        throw(ArgumentError("unknown outlier method $(repr(f.method)); use one of $(join(OUTLIER_METHODS, ", "))"))
    K = _kaoscleaner()
    # invokelatest: KaosCleaner's methods may be newer than the calling code
    k = something(f.threshold, Base.invokelatest(getproperty, K, :OUTLIER_DEFAULTS)[f.method])
    outlier_mask = Base.invokelatest(getproperty, K, :outlier_mask)
    times = isempty(pc.gps_time) ? nothing : pc.gps_time
    keep = trues(length(pc))
    for a in f.attributes
        flagged = Base.invokelatest(outlier_mask, attribute_column(pc, a), f.method, k, f.window, times)
        keep .&= .!flagged
    end
    return keep
end

"Column of one point attribute as Float64."
function attribute_column(pc::PointCloud, a::Symbol)
    a === :x && return Float64[v.position[1] for v in pc.vertices]
    a === :y && return Float64[v.position[2] for v in pc.vertices]
    a === :z && return Float64[v.position[3] for v in pc.vertices]
    a === :intensity && return Float64[intensity(v) for v in pc.vertices]
    throw(ArgumentError("unknown point attribute :$a (use :x, :y, :z or :intensity)"))
end
