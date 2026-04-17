$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_worldPos, 0);

uniform vec4 u_shapeData0;
uniform vec4 u_shapeData1;
uniform vec4 u_shapeData2;
uniform vec4 u_shapeData3;
uniform vec4 u_shapeType;

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

bool insideShape(vec3 point)
{
    int type = int(u_shapeType.x);
    if (type == 0) {
        return insideSphere(point, u_shapeData0);
    }
    if (type == 1) {
        return insideCylinder(point, u_shapeData0, u_shapeData1);
    }
    if (type == 2) {
        return insideCapsule(point, u_shapeData0, u_shapeData1);
    }
    if (type == 3) {
        return insideObb(point, u_shapeData0, u_shapeData1, u_shapeData2, u_shapeData3);
    }
    return false;
}

void main()
{
    vec3 world_pos = texture2D(s_worldPos, v_texcoord0).xyz;
    if (insideShape(world_pos)) {
        gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);
    } else {
        discard;
    }
}
