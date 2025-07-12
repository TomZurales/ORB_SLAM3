#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <memory>

#include "ShaderManager.h"
#include "PPECameraInterface.h"
#include "Utility.h"

class CameraModel
{
    GLuint VAO = 0;

public:
    CameraModel();
    void draw(Eigen::Matrix4f cameraPose) const;
};