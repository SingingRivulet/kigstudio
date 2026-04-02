
$input v_normal

#include <bgfx_shader.sh>

void main()
{
    vec3 n = normalize(v_normal);
    float lighting = dot(n, vec3(0.3, 0.5, 0.8));
    lighting = lighting * 0.5 + 0.5;

    gl_FragColor = vec4(lighting, lighting, lighting, 1.0);
}