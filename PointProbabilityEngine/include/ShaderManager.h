#pragma once

#include <map>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "Shader.h"

class ShaderManager
{
private:
    static ShaderManager *instance;
    std::map<std::string, Shader *> shaders;

    ShaderManager() = default; // Private constructor for singleton pattern
    ~ShaderManager();
    ShaderManager(const ShaderManager &) = delete;            // Delete copy constructor
    ShaderManager &operator=(const ShaderManager &) = delete; // Delete copy assignment operator
    std::string activeShaderName;

public:
    static ShaderManager *getInstance();
    void addShader(const std::string &name, const char *vertexPath, const char *fragmentPath);
    void useShader(const std::string &name);

    void setModelMatrix(const glm::mat4 &matrix);
    void setViewMatrix(const glm::mat4 &matrix);
    void setStaticViewMatrix(const glm::mat4 &matrix);
    void setProjectionMatrix(const glm::mat4 &matrix);
    void setColor(const glm::vec4 &color);
    void setFaceValues(const std::vector<float> &values);
    void setChangeOfBasis(const glm::mat4 &matrix);
    std::string getActiveShaderName() const;
};