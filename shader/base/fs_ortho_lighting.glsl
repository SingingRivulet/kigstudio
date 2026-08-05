$input v_normal

#include <bgfx_shader.sh>

uniform vec4 u_baseColor;

void main()
{
    vec3 n = normalize(v_normal);
    // Directional light from upper-right-front
    float diff = max(dot(n, normalize(vec3(0.4, 0.6, 0.7))), 0.0);
    float ambient = 0.4;
    float lighting = ambient + diff * (1.0 - ambient);

    gl_FragColor = vec4(u_baseColor.rgb * lighting, 1.0);
}
