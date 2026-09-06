$input v_pos

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;
uniform vec4 u_excludeFromTint;
uniform vec4 u_lightingMode;
uniform vec4 u_pickId;

uniform vec4 u_sdfOrigin;   // xyz: SDF 全局原点, w: SDF 体素边长
uniform vec4 u_sdfCamPos;   // xyz: 相机位置（模型局部坐标）
uniform vec4 u_chunkMin;    // xyz: chunk map 原点（chunk 坐标）
uniform vec4 u_chunkDim;    // xyz: chunk map 尺寸（chunk 数）
uniform vec4 u_poolInfo;    // x: brick pool 每轴 brick 数

SAMPLER3D(s_chunkMap, 14);   // R32F point: 0=远场, >0=brick 索引+1, <0=uniform 值
SAMPLER3D(s_brickPool, 15);  // R32F linear: 34^3 brick 图集（32^3 数据 + 1 体素 ghost layer）

#define SDF_FAR 1.0e6

// vox: SDF 体素连续坐标（采样点在体素中心，vox = (p - origin)/vs - 0.5）
float sampleSDF(vec3 vox)
{
    vec3 vv = vox + 0.5;  // 体素角点坐标
    vec3 chunk_f = floor(vv / 32.0);
    vec3 cm = chunk_f - u_chunkMin.xyz;
    if (any(lessThan(cm, vec3(0.0, 0.0, 0.0))) ||
        any(greaterThan(cm, u_chunkDim.xyz - 1.0)))
    {
        return SDF_FAR;
    }
    float code = texelFetch(s_chunkMap, ivec3(cm), 0).r;
    if (code == 0.0)
    {
        return SDF_FAR;
    }
    if (code < 0.0)
    {
        return code;  // uniform chunk（值 <= -1）
    }
    // dense brick（brick 含 1 体素 ghost layer：texel j 存体素 j-1，
    // local+1.0 使体素中心恰好落在 texel 中心，边界混入的是邻 chunk
    // 真实值而非图集中相邻 brick 的无关 texel）
    int brick = int(code) - 1;
    int bpa = int(u_poolInfo.x);
    ivec3 bxyz = ivec3(brick % bpa, (brick / bpa) % bpa, brick / (bpa * bpa));
    vec3 local = vv - chunk_f * 32.0;  // [0,32) chunk 内连续坐标
    float pool_texels = u_poolInfo.x * 34.0;
    vec3 uvw = (vec3(bxyz) * 34.0 + local + 1.0) / pool_texels;
    return texture(s_brickPool, uvw).r;
}

void main()
{
    float vs = u_sdfOrigin.w;
    vec3 ro = u_sdfCamPos.xyz;
    vec3 rd = v_pos - ro;
    float rd_len = max(length(rd), 1e-9);
    rd = rd / rd_len + vec3(1e-9, 1e-9, 1e-9);  // 防除零

    // 视线与 chunk map 覆盖的 AABB（世界/局部坐标）求交
    vec3 bmin = u_sdfOrigin.xyz + u_chunkMin.xyz * 32.0 * vs;
    vec3 bmax = u_sdfOrigin.xyz + (u_chunkMin.xyz + u_chunkDim.xyz) * 32.0 * vs;
    vec3 tA = (bmin - ro) / rd;
    vec3 tB = (bmax - ro) / rd;
    vec3 tmin = min(tA, tB);
    vec3 tmax = max(tA, tB);
    float tEnter = max(max(tmin.x, tmin.y), tmin.z);
    float tExit = min(min(tmax.x, tmax.y), tmax.z);
    if (tExit <= max(tEnter, 0.0))
    {
        discard;
    }

    // 球体追踪：远场按 chunk 边界精确步进（DDA，不跳 chunk），
    // 窄带内按 SDF 值自适应；命中条件为真正穿过符号（d<0），
    // 保证后续二分细化的 hi 端在表面内侧，收敛到 d=0 等值面
    float t = max(tEnter, 0.0);
    float hitT = -1.0;
    float prevT = t;
    // 掠射角视线可能始终不穿符号：记录最近点兜底，防止迭代耗尽出洞
    float bestT = -1.0;
    float bestD = 1.0e30;
    for (int i = 0; i < 256; ++i)
    {
        vec3 p = ro + rd * t;
        vec3 vox = (p - u_sdfOrigin.xyz) / vs - 0.5;
        float d = sampleSDF(vox);
        if (d < 0.0)
        {
            hitT = t;
            break;
        }
        if (d < bestD)
        {
            bestD = d;
            bestT = t;
        }
        float step_len;
        if (d >= 1.0e5)
        {
            // 远场：走到当前 chunk 的下一个边界面（沿视线方向）
            vec3 vv = vox + 0.5;
            vec3 cf = floor(vv / 32.0);
            vec3 nb = (cf + vec3(rd.x > 0.0 ? 1.0 : 0.0,
                                 rd.y > 0.0 ? 1.0 : 0.0,
                                 rd.z > 0.0 ? 1.0 : 0.0)) * 32.0;
            vec3 tb = (nb - vv) * vs / rd;
            step_len = min(min(tb.x, tb.y), tb.z) + vs * 1e-3;
        }
        else
        {
            // d*0.9 略保守防止过冲；d 小于 0.25vs 时最小步长必穿越符号
            step_len = max(d * 0.9, vs * 0.25);
        }
        prevT = t;
        t += step_len;
        if (t > tExit)
        {
            discard;
        }
    }
    bool crossed = hitT >= 0.0;
    if (!crossed)
    {
        // 未穿越符号但曾足够接近表面（掠射）：取最近点作为近似命中
        if (bestT >= 0.0 && bestD < vs * 0.75)
        {
            hitT = bestT;
        }
        else
        {
            discard;
        }
    }

    vec3 hit;
    if (crossed)
    {
        // 二分细化命中点（lo 在外 d>0，hi 在内 d<0）
        float lo = prevT;
        float hi = hitT;
        for (int i = 0; i < 6; ++i)
        {
            float mid = (lo + hi) * 0.5;
            vec3 p = ro + rd * mid;
            vec3 vox = (p - u_sdfOrigin.xyz) / vs - 0.5;
            float d = sampleSDF(vox);
            if (d < 0.0) { hi = mid; } else { lo = mid; }
        }
        hit = ro + rd * ((lo + hi) * 0.5);
    }
    else
    {
        hit = ro + rd * hitT;
    }
    vec3 hit_vox = (hit - u_sdfOrigin.xyz) / vs - 0.5;

    // 法线：中心差分梯度
    float eps = vs;
    vec3 n_local = normalize(vec3(
        sampleSDF(hit_vox + vec3(eps / vs, 0.0, 0.0)) - sampleSDF(hit_vox - vec3(eps / vs, 0.0, 0.0)),
        sampleSDF(hit_vox + vec3(0.0, eps / vs, 0.0)) - sampleSDF(hit_vox - vec3(0.0, eps / vs, 0.0)),
        sampleSDF(hit_vox + vec3(0.0, 0.0, eps / vs)) - sampleSDF(hit_vox - vec3(0.0, 0.0, eps / vs))));

    // 命中点深度（供与 mesh 正确遮挡 + 鼠标拾取 readback）
    vec4 clip = mul(u_modelViewProj, vec4(hit, 1.0));
    gl_FragDepth = clip.z / clip.w;

    // 与 fs_mesh_gbuffer 相同的 G-buffer 布局
    vec4 albedo = u_baseColor;
    if (u_lightingMode.x > 0.5)
    {
        vec3 light_dir = normalize(vec3(0.4, 0.6, 0.7));
        vec3 n_world = normalize(mul(u_model[0], vec4(n_local, 0.0)).xyz);
        float diff = max(dot(n_world, light_dir), 0.0);
        float ambient = 0.4;
        float lighting = ambient + diff * (1.0 - ambient);
        albedo.rgb *= lighting;
    }
    vec3 n_out = normalize(mul(u_model[0], vec4(n_local, 0.0)).xyz) * 0.5 + 0.5;
    gl_FragData[0] = albedo;
    gl_FragData[1] = vec4(n_out, 0.0);
    float exclude = u_excludeFromTint.x > 0.5 ? 0.0 : 1.0;
    gl_FragData[2] = vec4(hit, exclude);
    gl_FragData[3] = vec4(hit, exclude);
    gl_FragData[4] = u_pickId;
}
