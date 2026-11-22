using Documenter
using SpatialRender

makedocs(
    sitename = "SpatialRender.jl",
    modules = [SpatialRender],
    authors = "Rainer Rodrigues",
    # DOCS_DRAFT=true skips running the @example blocks (quick text-only builds)
    draft = get(ENV, "DOCS_DRAFT", "false") == "true",
    format = Documenter.HTML(prettyurls = get(ENV, "CI", nothing) == "true",
                             assets = String[], size_threshold = 400 * 1024, size_threshold_warn = 300 * 1024),
    checkdocs = :exports,
    pages = [
        "Home" => "index.md",
        "Tutorials" => [
            "tutorials/getting_started.md",
            "tutorials/point_clouds.md",
            "tutorials/maps_and_frames.md",
            "tutorials/live_robot.md",
            "tutorials/replay_bags.md",
        ],
        "Guides" => [
            "guide/loading.md",
            "guide/viewer.md",
            "guide/live.md",
            "guide/replay.md",
            "guide/octree.md",
            "guide/performance.md",
        ],
        "API" => "api.md",
    ],
)

deploydocs(repo = "github.com/rainerrodrigues/SpatialRender.jl.git", devbranch = "main")
