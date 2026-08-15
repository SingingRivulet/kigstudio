
$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_albedo, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);

// 全局 SSAO 参数：x=采样半径(世界单位) y=强度 z=屏幕采样半径(像素) w=开关(>0.5 启用)
uniform vec4 u_aoParams;

// Interleaved Gradient Noise：比 hash 更均匀的逐像素抖动，噪点更细更散
float ign(vec2 p)
{
    return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

void main()
{
    // OpenGL 下全屏 quad 写 RT 时 y 翻转（texel = F(varying)），而 G-buffer
    // 纹理是屏幕对齐的（同 fs_deferred_combine 对 s_collision 的处理）：
    // 采样 G-buffer 前必须先翻转 y，否则烘焙出的 AO 值上下颠倒。
    vec2 uv0 = vec2(v_texcoord0.x, 1.0 - v_texcoord0.y);

    if (texture2D(s_albedo, uv0).a < 0.001) {
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);  // 背景无遮蔽
        return;
    }

    vec3 normal = normalize(texture2D(s_normal, uv0).rgb * 2.0 - 1.0);
    vec3 world_pos = texture2D(s_worldPos, uv0).xyz;

    float radius = u_aoParams.x;
    float strength = u_aoParams.y;
    float px_radius = u_aoParams.z;
    // 每像素随机旋转采样核，减少条带
    float angle0 = ign(gl_FragCoord.xy) * 6.2831853;
    float ca = cos(angle0);
    float sa = sin(angle0);
    float occ = 0.0;
    const int kSamples = 24;
    for (int i = 0; i < kSamples; ++i) {
        float fi = float(i) + 0.5;
        float r = px_radius * fi / float(kSamples);
        float a = fi * 2.3999632;  // 黄金角螺旋分布
        vec2 off = vec2(cos(a), sin(a));
        off = vec2(off.x * ca - off.y * sa, off.x * sa + off.y * ca);
        vec2 uv = uv0 + off * r * u_viewTexel.xy;
        if (texture2D(s_albedo, uv).a < 0.001) {
            continue;  // 背景像素不参与遮挡
        }
        vec3 d = texture2D(s_worldPos, uv).xyz - world_pos;
        float dist = length(d);
        if (dist >= radius || dist < 1e-4) {
            continue;
        }
        float h = dot(d, normal);
        // 法向半球平滑过渡（无硬阈值），避免样本进出跳动产生噪点
        occ += (1.0 - dist / radius) * smoothstep(0.0, radius * 0.25, h);
    }
    float ao = clamp(occ / float(kSamples) * 3.0, 0.0, 1.0);
    float brightness = 1.0 - ao * strength;
    gl_FragColor = vec4(brightness, brightness, brightness, 1.0);
}
