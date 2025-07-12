#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 model;
uniform mat4 static_view; // This is the view matrix for the static view
uniform mat4 projection;
uniform mat4 changeOfBasis;

void main()
{
    gl_Position = changeOfBasis * projection * static_view * model * vec4(aPos, 1.0);
}