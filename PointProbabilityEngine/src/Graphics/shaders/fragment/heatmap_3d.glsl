#version 330 core
out vec4 FragColor;

uniform float[20] heatmapValues; // Array of heatmap values

vec4 colorMap(float value)
{
    // Assume minVal = 0.0, maxVal = 1.0
    float t = clamp(value, 0.0, 1.0);

    // 5 color stops: blue, cyan, green, yellow, red
    vec3 colors[5];
    colors[0] = vec3(0.0, 0.0, 1.0); // blue
    colors[1] = vec3(0.0, 1.0, 1.0); // cyan
    colors[2] = vec3(0.0, 1.0, 0.0); // green
    colors[3] = vec3(1.0, 1.0, 0.0); // yellow
    colors[4] = vec3(1.0, 0.0, 0.0); // red

    float idx = t * 4.0;
    int idx0 = int(floor(idx));
    int idx1 = min(idx0 + 1, 4);
    float frac = idx - float(idx0);

    vec3 c0 = colors[idx0];
    vec3 c1 = colors[idx1];

    vec3 rgb = mix(c0, c1, frac);

    return vec4(rgb, 0.33);
}

void main()
{
    FragColor = colorMap(heatmapValues[gl_PrimitiveID]); // Default color
}