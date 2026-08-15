$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_albedo, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);
SAMPLER2D(s_collision, 3);
SAMPLER2D(s_volume, 4);
SAMPLER2D(s_meshStencil, 5);
SAMPLER2D(s_worldPosPick, 6);

uniform vec4 u_lightDir;
uniform vec4 u_space_div;
uniform vec4 u_space_div_mix;

uniform vec4 u_mousePos;
uniform vec4 u_mouseHighlight;

uniform vec4 u_pos_hightlight_counts;
uniform vec4 u_pos_hightlight[16];
uniform vec4 u_pos_hightlight_color[16];

// 全局 SSAO 参数：x=采样半径(世界单位) y=强度 z=屏幕采样半径(像素) w=开关(>0.5 启用)
uniform vec4 u_aoParams;

// Interleaved Gradient Noise：比 hash 更均匀的逐像素抖动，噪点更细更散
float ign(vec2 p)
{
    return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

void main()
{   
    vec4 albedo_sample = texture2D(s_albedo, v_texcoord0);
    if (albedo_sample.a < 0.001) {
        gl_FragColor = vec4(0.188, 0.188, 0.188, 1.0);
        return;
    }

    vec3 albedo = albedo_sample.rgb;
    vec4 normal_sample = texture2D(s_normal, v_texcoord0);
    vec3 normal = normal_sample.rgb * 2.0 - 1.0;
    vec4 world_pos_sample = texture2D(s_worldPos, v_texcoord0);
    vec3 world_pos = world_pos_sample.xyz;
    bool exclude_from_tint = world_pos_sample.a < 0.5;
    // 判断在哪一面
    float face_side = world_pos.x * u_space_div.x + world_pos.y * u_space_div.y + world_pos.z * u_space_div.z + u_space_div.w;
    if (face_side < 0.0) {
        albedo = mix(albedo, vec3(0.20, 1.00, 0.45), u_space_div_mix.x);
    } else {
        albedo = mix(albedo, vec3(1.00, 0.45, 0.20), u_space_div_mix.x);
    }

    normal = normalize(normal);

    vec3 light_dir = normalize(u_lightDir.xyz);
    float diffuse = max(dot(normal, light_dir), 0.0);
    float lighting = 0.25 + diffuse * 0.75;

    // 全局 SSAO（接触阴影）：所有表面互相遮蔽，靠近的面变暗。
    // 在屏幕空间螺旋采样邻近像素的 world_pos，统计法向半球内的近距遮挡。
    if (u_aoParams.w > 0.5) {
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
            vec2 uv = v_texcoord0 + off * r * u_viewTexel.xy;
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
        lighting *= 1.0 - ao * strength;
    }

    vec3 color = albedo * lighting;

    // 必须反转y轴，bgfx中rt的y轴是反的
    if (texture2D(s_collision, vec2(v_texcoord0.x, 1.0-v_texcoord0.y)).r > 0.5) {
        color = mix(color, vec3(0.20, 0.45, 1.00), 0.65);
    }

    if (texture2D(s_volume, vec2(v_texcoord0.x, v_texcoord0.y)).r > 0.5) {
        color = mix(color, vec3(0.20, 1.00, 0.45), 0.65);
    }

    // 必须反转y轴，bgfx中rt的y轴是反的
    if (!exclude_from_tint && texture2D(s_meshStencil, vec2(v_texcoord0.x, v_texcoord0.y)).r > 0.5) {
        color = mix(color, vec3(1.00, 0.80, 0.80), 0.45);
    }

    if (u_mouseHighlight.x > 0.5) {
        vec3 mouse_world_pos = vec3(u_mousePos.x, u_mousePos.y, u_mousePos.z);
        float range = u_mouseHighlight.y;
        float dist = max(range - length(mouse_world_pos - world_pos), 0.0);
        float t = smoothstep(0.0, range, dist);
        color = mix(color, vec3(1.00, 0.0, 0.0), t);
    }

    for (int i = 0; i < 16; ++i) {
        if (i < int(u_pos_hightlight_counts.x)) {
            vec3 pos = u_pos_hightlight[i].xyz;
            float dist = max(1.0 - length(pos - world_pos), 0.0);
            float t = smoothstep(0.0, 1.0, dist);
            color = mix(color, u_pos_hightlight_color[i].xyz, t);
        }
    }

    gl_FragColor = vec4(color, 1.0);
}
