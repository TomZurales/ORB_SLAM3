#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>

#include "PPEPointInterface.h"
#include "PPECameraInterface.h"
#include "ShaderManager.h"
#include "Shader.h"

class PointCloud
{
    GLuint VAO = 0;
    GLuint VBO = 0;
    glm::mat4 modelPose = glm::mat4(1.0f); // Point cloud is always at the origin

public:
    PointCloud();
    void draw(std::vector<PPEPointInterface *> points) const;
};