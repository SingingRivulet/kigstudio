$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;
uniform vec4 u_excludeFromTint;
uniform vec4 u_lightingMode;

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
    gl_FragData[1] = vec4(n, 1.0);
    float exclude = u_excludeFromTint.x > 0.5 ? 0.0 : 1.0;
    gl_FragData[2] = vec4(v_pos, exclude);
    gl_FragData[3] = vec4(v_pos, exclude);
}
