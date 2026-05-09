$input a_position, a_normal
$output v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_depthBias;

void main()
{
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    gl_Position.z -= u_depthBias.x * gl_Position.w;
    v_normal = normalize(mul(u_model[0], vec4(a_normal, 0.0)).xyz);
    v_pos = a_position;
}
