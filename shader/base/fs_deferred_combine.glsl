$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_albedo, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);
SAMPLER2D(s_collision, 3);

uniform vec4 u_lightDir;
uniform vec4 u_space_div;
uniform vec4 u_space_div_mix;

uniform vec4 u_mousePos;
uniform vec4 u_mouseHighlight;

uniform vec4 u_mouseOri;
uniform vec4 u_mouseDir;

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

    // 必须反转y轴，bgfx中rt的y轴是反的
    if (texture2D(s_collision, vec2(v_texcoord0.x, 1.0-v_texcoord0.y)).r > 0.5) {
        color = mix(color, vec3(0.20, 0.45, 1.00), 0.65);
    }

    if (u_mouseHighlight.x > 0.5) {
        vec3 mouse_world_pos = vec3(u_mousePos.x, u_mousePos.y, u_mousePos.z);
        float dist = length(mouse_world_pos - world_pos);
        if (dist < 3) {
            color = mix(color, vec3(1.00, 0.45, 0.20), 0.65);
        }
    }
    // 利用点、向量描述的直线，把鼠标射线周围3以内的染成红色
    // 计算点到鼠标射线的距离
    vec3 ray_origin = u_mouseOri.xyz;
    vec3 ray_dir = normalize(u_mouseDir.xyz);
    vec3 point_to_ray = world_pos - ray_origin;
    
    // 投影到射线方向上的长度
    float projection = dot(point_to_ray, ray_dir);
    
    // 计算射线上的最近点
    vec3 closest_point = ray_origin + ray_dir * max(0.0, projection);
    
    // 计算点到射线的距离
    float ray_distance = length(world_pos - closest_point);
    
    // 如果距离小于3，混合红色
    if (ray_distance < 3.0) {
        color = mix(color, vec3(1.0, 0.0, 0.0), 0.65);
    }


    gl_FragColor = vec4(color, 1.0);
}
