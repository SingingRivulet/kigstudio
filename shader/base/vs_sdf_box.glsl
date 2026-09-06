$input a_position
$output v_pos

#include <bgfx_shader.sh>

uniform vec4 u_sdfOrigin;   // xyz: SDF 全局原点, w: SDF 体素边长
uniform vec4 u_chunkMin;    // xyz: chunk map 原点（chunk 坐标）
uniform vec4 u_chunkDim;    // xyz: chunk map 尺寸（chunk 数）

void main()
{
    // 单位盒顶点 → chunk map AABB（模型局部坐标），v_pos 供 fs 重建视线方向
    vec3 local = u_sdfOrigin.xyz +
                 (u_chunkMin.xyz + a_position * u_chunkDim.xyz) * 32.0 *
                     u_sdfOrigin.w;
    v_pos = local;
    gl_Position = mul(u_modelViewProj, vec4(local, 1.0));
}
