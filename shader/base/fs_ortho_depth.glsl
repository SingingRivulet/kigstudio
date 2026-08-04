$input v_normal, v_pos

#include <bgfx_shader.sh>

uniform vec4 u_viewDir;
uniform vec4 u_centerPos;
uniform vec4 u_depthScale;

void main()
{
    // Project world-space position onto view direction relative to center
    vec3 rel = v_pos - u_centerPos.xyz;
    float depth = dot(rel, u_viewDir.xyz);

    // Normalize: viewport_size maps to [0, 1] centered at 0.5
    float t = depth * u_depthScale.x + 0.5;
    t = clamp(t, 0.0, 1.0);

    // Heatmap: blue (near) -> cyan -> green -> yellow -> red (far)
    vec3 color;
    if (t < 0.2) {
        color = mix(vec3(0.2, 0.2, 0.9), vec3(0.2, 0.7, 0.9), t / 0.2);
    } else if (t < 0.4) {
        color = mix(vec3(0.2, 0.7, 0.9), vec3(0.2, 0.9, 0.4), (t - 0.2) / 0.2);
    } else if (t < 0.6) {
        color = mix(vec3(0.2, 0.9, 0.4), vec3(0.9, 0.9, 0.2), (t - 0.4) / 0.2);
    } else if (t < 0.8) {
        color = mix(vec3(0.9, 0.9, 0.2), vec3(0.9, 0.4, 0.2), (t - 0.6) / 0.2);
    } else {
        color = mix(vec3(0.9, 0.4, 0.2), vec3(0.9, 0.2, 0.2), (t - 0.8) / 0.2);
    }

    gl_FragColor = vec4(color, 1.0);
}
