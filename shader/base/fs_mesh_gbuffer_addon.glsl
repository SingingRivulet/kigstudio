
$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;

void main()
{
    vec3 n = normalize(v_normal) * 0.5 + 0.5;
    gl_FragData[0] = u_baseColor;
    gl_FragData[1] = vec4(n, 1.0);
    gl_FragData[2] = vec4(v_pos, 1.0);
    // 不写 gl_FragData[3]：保留 mouse pick world_pos 通道中已有的下层表面数据，
    // 让 addon 模型在视觉上遮挡，但对鼠标拾取保持透明。
}
