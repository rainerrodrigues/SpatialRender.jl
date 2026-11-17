# Out-of-core viewing of a LAS file larger than RAM / GPU memory.
#
#   julia -t auto --project=. examples/octree.jl survey.las            # convert (once) and view
#   julia -t auto --project=. examples/octree.jl survey.las shot.png   # offscreen screenshot

using SpatialRender

las = ARGS[1]
dir = splitext(las)[1] * "_octree"
if !isfile(joinpath(dir, "octree.json"))
    @info "converting $las → $dir (one-off)"
    @time build_octree(las, dir)
end
scene = Scene()
load!(scene, dir; point_budget = 4_000_000)       # sets the scene frame from the data
if length(ARGS) > 1
    screenshot(ARGS[2], scene; width = 1600, height = 900)
else
    open_viewer(scene; title = "SpatialRender: $(basename(las))")
end
