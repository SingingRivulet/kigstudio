$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_albedo, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);
SAMPLER2D(s_collision, 3);

uniform vec4 u_lightDir;
uniform vec4 u_collisionCounts;
uniform vec4 u_collisionSpheres[16];
uniform vec4 u_collisionCylindersA[16];
uniform vec4 u_collisionCylindersB[16];
uniform vec4 u_collisionCapsulesA[16];
uniform vec4 u_collisionCapsulesB[16];
uniform vec4 u_collisionObbCenter[16];
uniform vec4 u_collisionObbAxisX[16];
uniform vec4 u_collisionObbAxisY[16];
uniform vec4 u_collisionObbAxisZ[16];
uniform vec4 u_space_div;
uniform vec4 u_space_div_mix;

bool insideSphere(vec3 point, vec4 sphere)
{
    vec3 delta = point - sphere.xyz;
    return dot(delta, delta) <= sphere.w * sphere.w;
}

bool insideCylinder(vec3 point, vec4 start_radius, vec4 end_point)
{
    vec3 axis = end_point.xyz - start_radius.xyz;
    float axis_len_sq = dot(axis, axis);
    if (axis_len_sq <= 1e-8) {
        vec3 delta = point - start_radius.xyz;
        return dot(delta, delta) <= start_radius.w * start_radius.w;
    }

    float t = dot(point - start_radius.xyz, axis) / axis_len_sq;
    if (t < 0.0 || t > 1.0) {
        return false;
    }

    vec3 closest = start_radius.xyz + axis * t;
    vec3 delta = point - closest;
    return dot(delta, delta) <= start_radius.w * start_radius.w;
}

bool insideCapsule(vec3 point, vec4 start_radius, vec4 end_point)
{
    vec3 axis = end_point.xyz - start_radius.xyz;
    float axis_len_sq = dot(axis, axis);
    float t = 0.0;
    if (axis_len_sq > 1e-8) {
        t = clamp(dot(point - start_radius.xyz, axis) / axis_len_sq, 0.0, 1.0);
    }

    vec3 closest = start_radius.xyz + axis * t;
    vec3 delta = point - closest;
    return dot(delta, delta) <= start_radius.w * start_radius.w;
}

bool insideObb(vec3 point, vec4 center, vec4 axis_x, vec4 axis_y, vec4 axis_z)
{
    vec3 local = point - center.xyz;

    float hx = length(axis_x.xyz);
    float hy = length(axis_y.xyz);
    float hz = length(axis_z.xyz);
    if (hx <= 1e-8 || hy <= 1e-8 || hz <= 1e-8) {
        return false;
    }

    vec3 dx = axis_x.xyz / hx;
    vec3 dy = axis_y.xyz / hy;
    vec3 dz = axis_z.xyz / hz;

    return abs(dot(local, dx)) <= hx &&
           abs(dot(local, dy)) <= hy &&
           abs(dot(local, dz)) <= hz;
}

bool insideAnyCollision(vec3 point)
{
    for (int i = 0; i < 16; ++i) {
        if (float(i) < u_collisionCounts.x && insideSphere(point, u_collisionSpheres[i])) {
            return true;
        }
        if (float(i) < u_collisionCounts.y &&
            insideCylinder(point, u_collisionCylindersA[i], u_collisionCylindersB[i])) {
            return true;
        }
        if (float(i) < u_collisionCounts.z &&
            insideCapsule(point, u_collisionCapsulesA[i], u_collisionCapsulesB[i])) {
            return true;
        }
        if (float(i) < u_collisionCounts.w &&
            insideObb(point,
                      u_collisionObbCenter[i],
                      u_collisionObbAxisX[i],
                      u_collisionObbAxisY[i],
                      u_collisionObbAxisZ[i])) {
            return true;
        }
    }

    return false;
}

void main()
{   
    vec4 albedo_sample = texture2D(s_albedo, v_texcoord0);
    if (albedo_sample.a < 0.001) {
        gl_FragColor = vec4(0.188, 0.188, 0.188, 1.0);
        return;
    }

    vec3 albedo = albedo_sample.rgb;
    vec3 normal = texture2D(s_normal, v_texcoord0).rgb * 2.0 - 1.0;
    vec3 world_pos = texture2D(s_worldPos, v_texcoord0).xyz;
    // 判断在哪一面
    float face_side = world_pos.x * u_space_div.x + world_pos.y * u_space_div.y + world_pos.z * u_space_div.z + u_space_div.w;
    if (face_side < 0.0) {
        albedo = mix(albedo, vec3(0.20, 1.00, 0.45), u_space_div_mix.x);
    } else {
        albedo = mix(albedo, vec3(1.00, 0.45, 0.20), u_space_div_mix.x);
    }

    normal = normalize(normal);

    vec3 light_dir = normalize(u_lightDir.xyz);
    float diffuse = max(dot(normal, light_dir), 0.0);
    float lighting = 0.25 + diffuse * 0.75;
    vec3 color = albedo * lighting;

    if (texture2D(s_collision, vec2(v_texcoord0.x, 1.0-v_texcoord0.y)).r > 0.5) {
        color = mix(color, vec3(0.20, 0.45, 1.00), 0.65);
    }

    gl_FragColor = vec4(color, 1.0);
}
