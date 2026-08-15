
$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_aoRaw, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);

// 复用 AO 参数：x=世界采样半径（用作距离权重基准）
uniform vec4 u_aoParams;

// 法线+世界位置感知的双边模糊：抹平 SSAO 噪点，同时阻止遮蔽跨
//  silhouette/缝隙边缘渗透（钻孔与底模之间的间隙保持清晰）。
void main()
{
    vec3 ns0 = texture2D(s_normal, v_texcoord0).rgb;
    if (dot(ns0, ns0) < 0.0001) {
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);  // 背景
        return;
    }

    float center = texture2D(s_aoRaw, v_texcoord0).r;
    vec3 n0 = normalize(ns0 * 2.0 - 1.0);
    vec3 p0 = texture2D(s_worldPos, v_texcoord0).xyz;

    float sigma_p = u_aoParams.x * 0.5;
    float inv2sp2 = 1.0 / (2.0 * sigma_p * sigma_p);

    float sum = 0.0;
    float wsum = 0.0;
    for (int dy = -2; dy <= 2; ++dy)
    {
        for (int dx = -2; dx <= 2; ++dx)
        {
            vec2 uv = v_texcoord0 + vec2(float(dx), float(dy)) * u_viewTexel.xy;
            vec3 ns = texture2D(s_normal, uv).rgb;
            if (dot(ns, ns) < 0.0001) {
                continue;  // 背景样本不参与，防止暗部渗出物体边缘
            }
            vec3 n = normalize(ns * 2.0 - 1.0);
            vec3 p = texture2D(s_worldPos, uv).xyz;
            float dp = length(p - p0);
            float w_spatial = exp(-float(dx * dx + dy * dy) * 0.25);
            float w_normal = pow(max(dot(n, n0), 0.0), 8.0);
            float w_pos = exp(-dp * dp * inv2sp2);
            float w = w_spatial * w_normal * w_pos;
            sum += texture2D(s_aoRaw, uv).r * w;
            wsum += w;
        }
    }
    float ao = wsum > 1e-5 ? sum / wsum : center;
    gl_FragColor = vec4(ao, ao, ao, 1.0);
}
