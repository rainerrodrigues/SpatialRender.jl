# Offscreen rendering: images without a window (CI snapshots, batch reports,
# headless servers).

"Read the current framebuffer as an `h×w` matrix of RGB tuples (top row first)."
function read_framebuffer(w::Integer, h::Integer)
    buf = Vector{UInt8}(undef, 3w * h)
    glPixelStorei(GL_PACK_ALIGNMENT, 1)
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf)
    img = Matrix{NTuple{3, UInt8}}(undef, h, w)
    for y in 1:h, x in 1:w
        k = 3 * ((h - y) * w + (x - 1))
        img[y, x] = (buf[k+1], buf[k+2], buf[k+3])
    end
    return img
end

"""
    render_offscreen(scene; width=1024, height=768, camera=nothing, settings) -> Matrix{NTuple{3,UInt8}}

Render one frame into a hidden framebuffer and return it as an image
(rows top to bottom). With `camera=nothing` the camera frames the scene
from the south-west, 30° above the horizon. Useful for CI snapshots and
batch reports.
"""
function render_offscreen(scene::Scene; width::Integer=1024, height::Integer=768,
                          camera::Union{Nothing, Camera}=nothing,
                          settings::RenderSettings=RenderSettings())
    with_hidden_context() do
        fbo = Ref{GLuint}(0); rb = Ref{GLuint}(0); db = Ref{GLuint}(0)
        glGenFramebuffers(1, fbo); glBindFramebuffer(GL_FRAMEBUFFER, fbo[])
        glGenRenderbuffers(1, rb); glBindRenderbuffer(GL_RENDERBUFFER, rb[])
        glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[])
        glGenRenderbuffers(1, db); glBindRenderbuffer(GL_RENDERBUFFER, db[])
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, db[])
        glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE ||
            error("offscreen framebuffer incomplete")
        r = Renderer(scene; settings)
        cam = camera
        if cam === nothing
            b = r.box
            c = isempty(b) ? Vec3d(0, 0, 0) : Vec3d(center(b))
            cam = Camera(position = c + Vec3d(-1, -1, 0.82), target = c, aspect = width / height)
            frame!(cam, b)
        end
        render_frame!(r, cam, width, height)
        # out-of-core layers: load what the view asks for, redraw, until stable
        # (one extra frame per octree level, not per node)
        for _ in 1:64
            streaming(r) || break
            drain!(r)
            render_frame!(r, cam, width, height)
        end
        glFinish()
        img = read_framebuffer(width, height)
        release!(r)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glDeleteRenderbuffers(1, rb); glDeleteRenderbuffers(1, db); glDeleteFramebuffers(1, fbo)
        return img
    end
end

"""
    screenshot(path, scene; kwargs...) -> path

[`render_offscreen`](@ref) and save the image as PNG.
"""
screenshot(path::AbstractString, scene::Scene; kwargs...) = write_png(path, render_offscreen(scene; kwargs...))

