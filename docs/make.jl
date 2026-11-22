using Documenter
using SpatialRender

makedocs(
    sitename = "SpatialRender.jl",
    modules = [SpatialRender],
    authors = "Rainer Rodrigues",
    format = Documenter.HTML(prettyurls = get(ENV, "CI", nothing) == "true",
                             assets = String[], size_threshold = 400 * 1024, size_threshold_warn = 300 * 1024),
    checkdocs = :exports,
    pages = [
        "Home" => "index.md",
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
