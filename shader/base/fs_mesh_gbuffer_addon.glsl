
$input v_normal

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;

void main()
{
    vec3 n = normalize(v_normal) * 0.5 + 0.5;
    gl_FragData[0] = u_baseColor;
    gl_FragData[1] = vec4(n, 1.0);
    // 故意不写 gl_FragData[2]（world_pos 通道）：该像素保留下层表面的
    // 世界坐标，鼠标拾取可以穿透本网格拾取到下层模型
}
