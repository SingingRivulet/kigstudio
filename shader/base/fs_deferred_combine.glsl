$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_albedo, 0);
SAMPLER2D(s_normal, 1);
SAMPLER2D(s_worldPos, 2);
SAMPLER2D(s_collision, 3);

uniform vec4 u_lightDir;
uniform vec4 u_space_div;
uniform vec4 u_space_div_mix;

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

    gl_FragColor = vec4(color, 1.0);
}
