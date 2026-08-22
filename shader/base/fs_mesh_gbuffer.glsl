$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;
uniform vec4 u_excludeFromTint;
uniform vec4 u_lightingMode;
uniform vec4 u_pickId;

void main()
{
    vec4 albedo = u_baseColor;

    // Apply diffuse lighting when enabled (x > 0.5)
    if (u_lightingMode.x > 0.5)
    {
        vec3 light_dir = normalize(vec3(0.4, 0.6, 0.7));
        float diff = max(dot(normalize(v_normal), light_dir), 0.0);
        float ambient = 0.4;
        float lighting = ambient + diff * (1.0 - ambient);
        albedo.rgb *= lighting;
    }

    vec3 n = normalize(v_normal) * 0.5 + 0.5;
    gl_FragData[0] = albedo;
    // normal alpha=0 表示"非钻孔像素"（钻孔用 fs_mesh_gbuffer_drill 写 1.0），
    // 供 fs_deferred_combine 的钻孔 SSAO 区分。
    gl_FragData[1] = vec4(n, 0.0);
    float exclude = u_excludeFromTint.x > 0.5 ? 0.0 : 1.0;
    gl_FragData[2] = vec4(v_pos, exclude);
    gl_FragData[3] = vec4(v_pos, exclude);
    gl_FragData[4] = u_pickId;
}
