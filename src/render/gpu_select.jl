# GPU selection on WSL2.
#
# WSL exposes Windows GPUs through D3D12 (/dev/dxg); Mesa's `d3d12` Gallium
# driver translates OpenGL onto it. Two things commonly go wrong:
#
#  1. Mesa does not pick `d3d12` by itself and falls back to llvmpipe (CPU).
#  2. On hybrid laptops DXCore lists the integrated GPU first; if its WSL
#     user-mode driver fails (seen with AMD `amdxc64.so`), Mesa gives up
#     instead of trying the discrete GPU, and context creation fails outright.
#
# `configure_gpu!` fixes both: it sets GALLIUM_DRIVER=d3d12 and
# MESA_D3D12_DEFAULT_ADAPTER_NAME, re-initialises GLFW (GLX reads these when
# the display is opened, which GLFW does at init), and verifies each
# candidate with a real hidden context, keeping the first hardware renderer.
# If none works the environment is restored, so the CPU fallback still runs.

const GPU_CONFIGURED = Ref(false)
const GPU_SELECTION = Ref("not configured")

"Running under WSL2 with GPU paravirtualisation?"
is_wsl_gpu() = Sys.islinux() && ispath("/dev/dxg")

# Renderer string of a throwaway hidden context, or nothing if none can be made.
function _probe_renderer()
    GLFW.DefaultWindowHints()
    GLFW.WindowHint(GLFW.CONTEXT_VERSION_MAJOR, 4)
    GLFW.WindowHint(GLFW.CONTEXT_VERSION_MINOR, 1)
    GLFW.WindowHint(GLFW.OPENGL_PROFILE, GLFW.OPENGL_CORE_PROFILE)
    GLFW.WindowHint(GLFW.OPENGL_FORWARD_COMPAT, true)
    GLFW.WindowHint(GLFW.VISIBLE, false)
    win = try
        GLFW.CreateWindow(16, 16, "SpatialRender GPU probe")
    catch
        return nothing
    end
    GLFW.MakeContextCurrent(win)
    p = glGetString(GL_RENDERER)
    r = p == C_NULL ? "" : unsafe_string(p)
    GLFW.DestroyWindow(win)
    CURRENT_CONTEXT[] = 0
    return r
end

_is_software(r) = occursin(r"llvmpipe|softpipe|swrast|SwiftShader|Basic Render"i, r)

function _reinit_glfw()
    GLFW.Terminate()
    GLFW.Init()
end

"""
    configure_gpu!(; candidates=nothing, force=false) -> String

Make OpenGL use a hardware GPU on WSL2 (see the comment at the top of
`src/render/gpu_select.jl`). Called automatically before the first window;
returns a description of what was selected.

Does nothing when not on WSL, when `GALLIUM_DRIVER` is already set (your
choice wins), or when `SPATIALRENDER_GPU_AUTOCONFIG=0`. `candidates` are
adapter-name substrings to try in order (default: an explicitly set
`MESA_D3D12_DEFAULT_ADAPTER_NAME`, else NVIDIA, AMD, Intel, then Mesa's
default adapter).

`probe` (returns the renderer string of a fresh context, or `nothing`),
`reinit` and `wsl` exist so tests can simulate drivers; leave them alone.
"""
function configure_gpu!(; candidates=nothing, force::Bool=false,
                        probe=_probe_renderer, reinit=_reinit_glfw, wsl::Bool=is_wsl_gpu())
    GPU_CONFIGURED[] && !force && return GPU_SELECTION[]
    GPU_CONFIGURED[] = true
    if get(ENV, "SPATIALRENDER_GPU_AUTOCONFIG", "1") == "0"
        return GPU_SELECTION[] = "auto-configuration disabled (SPATIALRENDER_GPU_AUTOCONFIG=0)"
    elseif !wsl
        return GPU_SELECTION[] = "not WSL: using the system OpenGL driver"
    elseif haskey(ENV, "GALLIUM_DRIVER") && !force
        return GPU_SELECTION[] = "GALLIUM_DRIVER=$(ENV["GALLIUM_DRIVER"]) set by the user"
    end
    saved = Dict(k => get(ENV, k, nothing) for k in ("GALLIUM_DRIVER", "MESA_D3D12_DEFAULT_ADAPTER_NAME"))
    restore!() = for (k, v) in saved
        v === nothing ? delete!(ENV, k) : (ENV[k] = v)
    end
    if candidates === nothing
        user = saved["MESA_D3D12_DEFAULT_ADAPTER_NAME"]
        candidates = user === nothing ? ["NVIDIA", "AMD", "Intel", ""] : [user]
    end
    for name in candidates
        ENV["GALLIUM_DRIVER"] = "d3d12"
        isempty(name) ? delete!(ENV, "MESA_D3D12_DEFAULT_ADAPTER_NAME") :
                        (ENV["MESA_D3D12_DEFAULT_ADAPTER_NAME"] = name)
        reinit()
        r = probe()
        if r !== nothing && !_is_software(r)
            return GPU_SELECTION[] = "WSL: $r (GALLIUM_DRIVER=d3d12" *
                (isempty(name) ? ")" : ", MESA_D3D12_DEFAULT_ADAPTER_NAME=$name)")
        end
    end
    restore!()
    reinit()
    return GPU_SELECTION[] = "WSL: no D3D12 adapter worked (tried $(join(repr.(candidates), ", "))); using the CPU"
end
