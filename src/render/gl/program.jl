# Shader compilation with a per-context cache and offline validation.
#
# Following the build-and-cache pattern of VeriJul.jl (compile external
# sources once, key the artefact by a content hash, check the toolchain in
# tests): programs are cached by the hash of their sources, so recreating a
# renderer or hot-reloading identical sources never recompiles, and
# `validate_shaders` checks every stage with `glslangValidator` from
# glslang_jll (no GPU needed, usable in CI), or with the live GL driver.

struct GLProgram
    id::GLuint
    name::Symbol
    uniforms::Dict{Symbol, GLint}
end

const PROGRAM_CACHE = Dict{Tuple{UInt, UInt}, GLProgram}()   # (context, source hash)
const CACHE_STATS = Ref((hits = 0, misses = 0))

"""
    compile_cache_stats() -> (hits, misses)

How often a shader program was served from the cache versus compiled.
"""
compile_cache_stats() = CACHE_STATS[]

function _shader_log(id, getiv, getlog)
    len = Ref{GLint}(0)
    getiv(id, GL_INFO_LOG_LENGTH, len)
    len[] <= 1 && return ""
    buf = Vector{UInt8}(undef, len[])
    getlog(id, len[], C_NULL, buf)
    return String(buf[1:end-1])
end

function _numbered(src)
    join((@sprintf("%4d  %s", i, l) for (i, l) in enumerate(split(src, '\n'))), '\n')
end

function compile_stage(src::String, kind::GLenum, name)
    id = glCreateShader(kind)
    ptrs = Ptr{GLchar}[pointer(src)]
    GC.@preserve src glShaderSource(id, 1, ptrs, C_NULL)
    glCompileShader(id)
    ok = Ref{GLint}(0)
    glGetShaderiv(id, GL_COMPILE_STATUS, ok)
    if ok[] != GL_TRUE
        log = _shader_log(id, glGetShaderiv, glGetShaderInfoLog)
        glDeleteShader(id)
        stage = kind == GL_VERTEX_SHADER ? "vertex" : "fragment"
        error("$stage shader `$name` failed to compile:\n$log\n$(_numbered(src))")
    end
    return id
end

"""
    GLProgram(src::ShaderSource; context=current context id)

Compile and link (or fetch from the cache) a program in the current GL
context, and resolve all active uniform locations once.
"""
function GLProgram(src::ShaderSource; context::UInt=current_context_id())
    key = (context, hash((src.vertex, src.fragment)))
    if haskey(PROGRAM_CACHE, key)
        s = CACHE_STATS[]; CACHE_STATS[] = (hits = s.hits + 1, misses = s.misses)
        return PROGRAM_CACHE[key]
    end
    s = CACHE_STATS[]; CACHE_STATS[] = (hits = s.hits, misses = s.misses + 1)
    vs = compile_stage(src.vertex, GL_VERTEX_SHADER, src.name)
    fs = compile_stage(src.fragment, GL_FRAGMENT_SHADER, src.name)
    id = glCreateProgram()
    glAttachShader(id, vs); glAttachShader(id, fs)
    glLinkProgram(id)
    glDeleteShader(vs); glDeleteShader(fs)
    ok = Ref{GLint}(0)
    glGetProgramiv(id, GL_LINK_STATUS, ok)
    ok[] == GL_TRUE || error("shader program `$(src.name)` failed to link:\n" *
                             _shader_log(id, glGetProgramiv, glGetProgramInfoLog))
    p = GLProgram(id, src.name, _active_uniforms(id))
    PROGRAM_CACHE[key] = p
    return p
end

function _active_uniforms(id)
    n = Ref{GLint}(0)
    glGetProgramiv(id, GL_ACTIVE_UNIFORMS, n)
    out = Dict{Symbol, GLint}()
    buf = Vector{UInt8}(undef, 256)
    len = Ref{GLsizei}(0); sz = Ref{GLint}(0); ty = Ref{GLenum}(0)
    for i in 0:n[]-1
        glGetActiveUniform(id, i, length(buf), len, sz, ty, buf)
        name = String(buf[1:len[]])
        name = replace(name, "[0]" => "")    # arrays: location of element 0
        out[Symbol(name)] = glGetUniformLocation(id, name)
    end
    return out
end

"Forget cached programs of a context that is being destroyed."
function drop_context_programs!(ctx::UInt)
    for k in collect(keys(PROGRAM_CACHE))
        k[1] == ctx && delete!(PROGRAM_CACHE, k)
    end
end

const CURRENT_CONTEXT = Ref{UInt}(0)
current_context_id() = CURRENT_CONTEXT[]

# --- uniform setters (no allocation: isbits values through Ref) -----------

@inline loc(p::GLProgram, s::Symbol) = get(p.uniforms, s, GLint(-1))
setuniform(p::GLProgram, s::Symbol, v::Integer) = glUniform1i(loc(p, s), GLint(v))
setuniform(p::GLProgram, s::Symbol, v::AbstractFloat) = glUniform1f(loc(p, s), Float32(v))
setuniform(p::GLProgram, s::Symbol, v::Vec2f) = glUniform2f(loc(p, s), v[1], v[2])
setuniform(p::GLProgram, s::Symbol, v::Vec3f) = glUniform3f(loc(p, s), v[1], v[2], v[3])
setuniform(p::GLProgram, s::Symbol, v::Vec4f) = glUniform4f(loc(p, s), v[1], v[2], v[3], v[4])
setuniform(p::GLProgram, s::Symbol, m::Mat4f) = glUniformMatrix4fv(loc(p, s), 1, GL_FALSE, Ref(m))
setuniform(p::GLProgram, s::Symbol, m::Mat3f) = glUniformMatrix3fv(loc(p, s), 1, GL_FALSE, Ref(m))
setuniform(p::GLProgram, s::Symbol, v::NTuple{N, Float32}) where {N} =
    glUniform1fv(loc(p, s), N, Ref(v))
setuniform(p::GLProgram, s::Symbol, v::Vector{Vec4f}) =
    glUniform4fv(loc(p, s), length(v), Ptr{Float32}(pointer(v)))

# --- offline validation ------------------------------------------------------

"""
    validate_shaders(sources=BUILTIN_SHADERS; backend=:auto) -> Vector{NamedTuple}

Check that every shader stage compiles. `backend=:glslang` runs
`glslangValidator`, the Khronos reference compiler shipped by glslang_jll (no
GPU or display needed), `backend=:gl` compiles in a hidden GLFW window with the
real driver, `:auto` prefers glslang. Returns
one `(name, stage, ok, log)` record per stage; never throws on a shader
error, so it suits CI reports.
"""
function validate_shaders(sources=ALL_SHADERS; backend::Symbol=:auto)
    tool = _glslang_command()
    if backend == :glslang || (backend == :auto && tool !== nothing)
        tool === nothing && error("glslangValidator is unavailable: glslang_jll has no build for this platform ",
                                  "and it is not on PATH")
        return _validate_glslang(sources, tool)
    end
    return _validate_gl(sources)
end

function _glslang_command()
    glslang_jll.is_available() && return glslang_jll.glslangValidator()
    exe = Sys.which("glslangValidator")
    return exe === nothing ? nothing : `$exe`
end

function _validate_glslang(sources, tool::Cmd)
    out = NamedTuple[]
    for s in sources, (stage, src, ext) in ((:vertex, s.vertex, "vert"), (:fragment, s.fragment, "frag"))
        path = tempname() * "." * ext
        write(path, src)
        buf = IOBuffer()
        ok = success(pipeline(ignorestatus(`$tool $path`); stdout=buf, stderr=buf))
        rm(path; force=true)
        push!(out, (name=s.name, stage=stage, ok=ok, log=String(take!(buf))))
    end
    return out
end

function _validate_gl(sources)
    out = NamedTuple[]
    with_hidden_context() do
        for s in sources, (stage, src, kind) in ((:vertex, s.vertex, GL_VERTEX_SHADER),
                                                 (:fragment, s.fragment, GL_FRAGMENT_SHADER))
            try
                glDeleteShader(compile_stage(src, kind, s.name))
                push!(out, (name=s.name, stage=stage, ok=true, log=""))
            catch e
                push!(out, (name=s.name, stage=stage, ok=false, log=sprint(showerror, e)))
            end
        end
    end
    return out
end
