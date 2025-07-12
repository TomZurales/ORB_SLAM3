#pragma once

#include <GL/glew.h>

#include "ShaderManager.h"

class VonMiserSphere
{
    GLuint VAO;
    int n_sides;

public:
    VonMiserSphere(int n_sides = 360);
    void draw() const;
};