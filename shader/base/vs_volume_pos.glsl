$input a_position

#include <bgfx_shader.sh>

void main()
{
    vec4 sc_pos = mul(u_modelViewProj, vec4(a_position, 1.0));
    gl_Position = sc_pos;
}
