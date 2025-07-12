#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <memory>

#include "ShaderManager.h"

class FloorPlane
{
    GLuint linesVAO = 0;
    glm::mat4 modelPose = glm::mat4(1.0f); // Floor plane is always at the origin

public:
    FloorPlane();
    void draw() const;
};