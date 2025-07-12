#pragma once

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <set>

#include "PPEPointInterface.h"
#include "Icosahedron.h"
#include "ShaderManager.h"
#include "Shader.h"

class IcosModel
{
    GLuint icosVAO = 0;
    GLuint icosEBO = 0;

public:
    IcosModel();
    void draw(PPEPointInterface *point, Icosahedron *icos) const;
};