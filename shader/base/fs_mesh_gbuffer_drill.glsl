
$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;

void main()
{
    vec3 n = normalize(v_normal) * 0.5 + 0.5;
    gl_FragData[0] = u_baseColor;
    // normal RT 的 alpha=1.0 作为"钻孔像素"标记，供 fs_deferred_combine
    // 做钻孔专用 SSAO（接触阴影）；其余 gbuffer shader 该通道写 0。
    gl_FragData[1] = vec4(n, 1.0);
    gl_FragData[2] = vec4(v_pos, 1.0);
    // 不写 gl_FragData[3]：与 addon 版一致，钻孔对鼠标拾取保持透明
    // （拾取模式下改用 fs_mesh_gbuffer，写完整 world_pos）。
}
