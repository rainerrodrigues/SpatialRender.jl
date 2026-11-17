# GLSL 4.10 core sources (GL 4.1 is the newest version macOS provides).

"""
    ShaderSource(name, vertex, fragment)

A named vertex + fragment shader pair. Programs are compiled once per GL
context and cached by a hash of their source (see [`compile_cache_stats`](@ref));
[`validate_shaders`](@ref) checks every built-in pair offline.
"""
struct ShaderSource
    name::Symbol
    vertex::String
    fragment::String
end

const GLSL_HEADER = "#version 410 core\n"

# Turbo colormap polynomial, shared by the point shader.
const GLSL_TURBO = """
vec3 turbo(float x) {
    x = clamp(x, 0.0, 1.0);
    const vec4 kr4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec4 kg4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec4 kb4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kr2 = vec2(-152.94239396, 59.28637943);
    const vec2 kg2 = vec2(4.27729857, 2.82956604);
    const vec2 kb2 = vec2(-89.90310912, 27.34824973);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z;
    return clamp(vec3(dot(v4, kr4) + dot(v2, kr2),
                      dot(v4, kg4) + dot(v2, kg2),
                      dot(v4, kb4) + dot(v2, kb2)), 0.0, 1.0);
}
"""

const POINT_SHADER = ShaderSource(:points, GLSL_HEADER * """
layout(location = 0) in vec3 a_pos;
layout(location = 1) in float a_intensity;
layout(location = 2) in vec4 a_rgba;
layout(location = 3) in uint a_class;
layout(location = 4) in uint a_flags;    // streaming layers: scan sequence number (mod 256)

uniform mat4 u_vp;
uniform mat4 u_model;
uniform int u_size_mode;        // 0: constant pixels, 1: world size (perspective)
uniform float u_point_size;     // pixels (mode 0) or metres (mode 1)
uniform float u_proj_scale;     // viewport height / (2 tan(fov / 2))
uniform int u_color_mode;       // 0 height, 1 intensity, 2 classification, 3 rgb, 4 scan age
uniform int u_seq;              // newest scan sequence number (age mode)
uniform float u_max_age;        // age (in scans) mapped to the end of the colormap
uniform vec2 u_zrange;
uniform vec4 u_class_colors[32];
uniform int u_highlight;        // 0-based vertex id to highlight, -1 none

out vec4 v_color;

$GLSL_TURBO

void main() {
    vec4 world = u_model * vec4(a_pos, 1.0);
    vec4 clip = u_vp * world;
    gl_Position = clip;
    float px = u_size_mode == 0 ? u_point_size
                                : u_point_size * u_proj_scale / max(clip.w, 1e-4);
    vec3 c;
    if (u_color_mode == 0)      c = turbo((world.z - u_zrange.x) / max(u_zrange.y - u_zrange.x, 1e-6));
    else if (u_color_mode == 1) c = vec3(pow(a_intensity, 0.6));
    else if (u_color_mode == 2) c = u_class_colors[min(a_class, 31u)].rgb;
    else if (u_color_mode == 3) c = a_rgba.rgb;
    else {
        float age = float((uint(u_seq) - a_flags) & 255u) / max(u_max_age, 1.0);
        c = turbo(1.0 - clamp(age, 0.0, 1.0)) * (1.0 - 0.55 * clamp(age, 0.0, 1.0));
    }
    if (gl_VertexID == u_highlight) { c = vec3(1.0, 0.1, 0.9); px = max(px * 3.0, 9.0); }
    gl_PointSize = clamp(px, 1.0, 64.0);
    v_color = vec4(c, 1.0);
}
""", GLSL_HEADER * """
in vec4 v_color;
uniform int u_round;
out vec4 frag;
void main() {
    vec2 d = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(d, d);
    if (u_round == 1 && r2 > 1.0) discard;
    // paraboloid shading: splats read as small spheres, which helps depth
    frag = vec4(v_color.rgb * (u_round == 1 ? 1.0 - 0.35 * r2 : 1.0), 1.0);
}
""")

const MESH_SHADER = ShaderSource(:mesh, GLSL_HEADER * """
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;

uniform mat4 u_vp;
uniform mat4 u_model;
uniform mat3 u_normal;          // inverse-transpose of the model's 3×3 part

out vec3 v_pos;
out vec3 v_normal;

void main() {
    vec4 world = u_model * vec4(a_pos, 1.0);
    v_pos = world.xyz;
    v_normal = u_normal * a_normal;
    gl_Position = u_vp * world;
}
""", GLSL_HEADER * """
in vec3 v_pos;
in vec3 v_normal;
uniform vec3 u_eye;
uniform vec4 u_color;
uniform int u_flat;             // 1: solid u_color (wireframe overlay pass)
out vec4 frag;

void main() {
    if (u_flat == 1) { frag = u_color; return; }
    vec3 n = normalize(v_normal);
    if (!gl_FrontFacing) n = -n;    // CAD exports often mix winding
    vec3 v = normalize(u_eye - v_pos);
    vec3 l = v;                      // headlight
    vec3 h = normalize(l + v);
    float diff = max(dot(n, l), 0.0);
    float spec = pow(max(dot(n, h), 0.0), 48.0);
    // hemisphere ambient: sky from +Z, ground from -Z
    vec3 amb = mix(vec3(0.20, 0.19, 0.18), vec3(0.32, 0.34, 0.38), 0.5 + 0.5 * n.z);
    vec3 c = u_color.rgb * (amb + 0.75 * diff) + vec3(0.25) * spec;
    frag = vec4(c, u_color.a);
}
""")

const LINE_SHADER = ShaderSource(:lines, GLSL_HEADER * """
layout(location = 0) in vec3 a_a;        // per instance: segment start
layout(location = 1) in vec3 a_b;        // per instance: segment end
layout(location = 2) in vec4 a_color;    // per instance
layout(location = 3) in vec2 a_corner;   // per vertex: (t ∈ {0,1}, side ∈ {-1,1})

uniform mat4 u_vp;
uniform mat4 u_model;
uniform vec2 u_viewport;
uniform float u_width;                   // pixels

out vec4 v_color;
out float v_dist;                        // signed pixels from the centre line
out float v_half;

void main() {
    vec4 ca = u_vp * (u_model * vec4(a_a, 1.0));
    vec4 cb = u_vp * (u_model * vec4(a_b, 1.0));
    // clip the segment against the near plane (w > eps) before dividing
    const float eps = 1e-5;
    if (ca.w < eps && cb.w < eps) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); return; }
    if (ca.w < eps) ca = mix(ca, cb, (eps - ca.w) / (cb.w - ca.w));
    if (cb.w < eps) cb = mix(cb, ca, (eps - cb.w) / (ca.w - cb.w));

    vec2 half_vp = 0.5 * u_viewport;
    vec2 sa = ca.xy / ca.w * half_vp;
    vec2 sb = cb.xy / cb.w * half_vp;
    vec2 d = sb - sa;
    float len = length(d);
    vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
    vec2 nrm = vec2(-dir.y, dir.x);

    float t = a_corner.x;
    float side = a_corner.y;
    v_half = 0.5 * u_width + 1.0;        // +1 px for the anti-aliased edge
    vec4 c = t < 0.5 ? ca : cb;
    vec2 off = nrm * side * v_half + dir * (t < 0.5 ? -1.0 : 1.0) * 0.5 * u_width;
    c.xy += off / half_vp * c.w;
    gl_Position = c;
    v_dist = side * v_half;
    v_color = a_color;
}
""", GLSL_HEADER * """
in vec4 v_color;
in float v_dist;
in float v_half;
out vec4 frag;
void main() {
    float a = clamp(v_half - abs(v_dist), 0.0, 1.0);
    if (a <= 0.0) discard;
    frag = vec4(v_color.rgb, v_color.a * a);
}
""")

const BUILTIN_SHADERS = (POINT_SHADER, MESH_SHADER, LINE_SHADER)

"ASPRS LAS 1.4 standard classes 0–18 plus a palette for user classes."
const CLASS_COLORS = let c = fill(Vec4f(0.8, 0.8, 0.8, 1), 32)
    c[1] = Vec4f(0.60, 0.60, 0.60, 1)    # 0 never classified
    c[2] = Vec4f(0.75, 0.75, 0.75, 1)    # 1 unclassified
    c[3] = Vec4f(0.63, 0.45, 0.28, 1)    # 2 ground
    c[4] = Vec4f(0.55, 0.85, 0.35, 1)    # 3 low vegetation
    c[5] = Vec4f(0.25, 0.70, 0.25, 1)    # 4 medium vegetation
    c[6] = Vec4f(0.10, 0.50, 0.15, 1)    # 5 high vegetation
    c[7] = Vec4f(0.90, 0.35, 0.25, 1)    # 6 building
    c[8] = Vec4f(1.00, 0.00, 1.00, 1)    # 7 low point (noise)
    c[10] = Vec4f(0.20, 0.45, 0.95, 1)   # 9 water
    c[11] = Vec4f(0.95, 0.85, 0.20, 1)   # 10 rail
    c[12] = Vec4f(0.35, 0.35, 0.40, 1)   # 11 road surface
    c[14] = Vec4f(0.95, 0.60, 0.10, 1)   # 13 wire guard
    c[15] = Vec4f(1.00, 0.80, 0.30, 1)   # 14 wire conductor
    c[16] = Vec4f(0.70, 0.30, 0.90, 1)   # 15 transmission tower
    c[18] = Vec4f(0.40, 0.75, 0.85, 1)   # 17 bridge deck
    c[19] = Vec4f(1.00, 0.20, 0.60, 1)   # 18 high noise
    for k in 20:32                         # user-defined: spread on turbo
        c[k] = turbo((k - 20) / 12)
    end
    c
end

# Eye-dome lighting (Boucheny 2009, as in CloudCompare / Potree): a
# screen-space pass that shades each pixel by how much nearer its neighbours
# are in log depth. Gives unlit point clouds readable shape and silhouettes.
const EDL_SHADER = ShaderSource(:edl, GLSL_HEADER * """
void main() {
    // one triangle covering the screen, no vertex buffer needed
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
""", GLSL_HEADER * """
uniform sampler2D u_color;
uniform sampler2D u_depth;
uniform vec2 u_near_far;
uniform int u_ortho;
uniform float u_strength;
uniform float u_radius;
out vec4 frag;

float log_depth(float d) {
    float n = u_near_far.x, f = u_near_far.y;
    float z = u_ortho == 1 ? n + d * (f - n) : n * f / (f - d * (f - n));
    return log2(max(z, 1e-6));
}

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    float d = texelFetch(u_depth, p, 0).r;
    vec4 c = texelFetch(u_color, p, 0);
    if (d >= 1.0) { frag = c; gl_FragDepth = 1.0; return; }   // background untouched
    float L = log_depth(d);
    float sum = 0.0;
    ivec2 size = textureSize(u_depth, 0);
    for (int i = 0; i < 8; i++) {
        float a = 0.78539816 * float(i);
        ivec2 q = clamp(p + ivec2(round(u_radius * vec2(cos(a), sin(a)))), ivec2(0), size - 1);
        float dn = texelFetch(u_depth, q, 0).r;
        // a background neighbour (silhouette) counts as much nearer
        sum += dn >= 1.0 ? 0.004 : max(0.0, L - log_depth(dn));
    }
    float shade = exp(-sum / 8.0 * 300.0 * u_strength);
    frag = vec4(c.rgb * shade, c.a);
    gl_FragDepth = d;
}
""")

const ALL_SHADERS = (BUILTIN_SHADERS..., EDL_SHADER)
