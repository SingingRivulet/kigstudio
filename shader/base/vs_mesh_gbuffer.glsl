$input a_position, a_normal
$output v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_depthBias;

void main()
{
    vec3 offset_pos = a_position + a_normal * u_depthBias.x;
    gl_Position = mul(u_modelViewProj, vec4(offset_pos, 1.0));
    v_normal = normalize(mul(u_model[0], vec4(a_normal, 0.0)).xyz);
    v_pos = a_position;
}
