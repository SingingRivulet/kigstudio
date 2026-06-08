$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;
uniform vec4 u_excludeFromTint;

void main()
{
    vec3 n = normalize(v_normal) * 0.5 + 0.5;
    gl_FragData[0] = u_baseColor;
    gl_FragData[1] = vec4(n, 1.0);
    float exclude = u_excludeFromTint.x > 0.5 ? 0.0 : 1.0;
    gl_FragData[2] = vec4(v_pos, exclude);
}
