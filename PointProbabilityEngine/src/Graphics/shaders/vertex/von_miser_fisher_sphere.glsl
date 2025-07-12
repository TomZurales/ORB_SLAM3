#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 changeOfBasis;

void main()
{
    // Correct view so it contains no rotation
    mat4 correctedView = mat4(view[0][0], view[0][1], view[0][2], 0.0f,
                        view[1][0], view[1][1], view[1][2], 0.0f,
                        view[2][0], view[2][1], view[2][2], 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f);

    gl_Position = changeOfBasis * projection * correctedView * model * vec4(aPos, 1.0);
}