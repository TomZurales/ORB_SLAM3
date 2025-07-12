#include "ShaderManager.h"
#include <iostream>

// Initialize static member
ShaderManager *ShaderManager::instance = nullptr;

ShaderManager::~ShaderManager()
{
    // Delete all shaders
    for (auto &pair : shaders)
    {
        delete pair.second;
    }
    shaders.clear();
}

ShaderManager *ShaderManager::getInstance()
{
    if (instance == nullptr)
    {
        instance = new ShaderManager();
    }
    return instance;
}

void ShaderManager::addShader(const std::string &name, const char *vertexPath, const char *fragmentPath)
{
    if (shaders.find(name) != shaders.end())
    {
        std::cerr << "Warning: Shader with name '" << name << "' already exists. Overwriting." << std::endl;
        delete shaders[name];
    }

    shaders[name] = new Shader(vertexPath, fragmentPath);
}

void ShaderManager::useShader(const std::string &name)
{
    auto it = shaders.find(name);
    if (it != shaders.end())
    {
        it->second->use();
        activeShaderName = name;
    }
    else
    {
        std::cerr << "Error: Shader '" << name << "' not found!" << std::endl;
    }
}

void ShaderManager::setModelMatrix(const glm::mat4 &matrix)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setMatrix4fv("model", matrix);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setViewMatrix(const glm::mat4 &matrix)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setMatrix4fv("view", matrix);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setStaticViewMatrix(const glm::mat4 &matrix)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setMatrix4fv("static_view", matrix);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setProjectionMatrix(const glm::mat4 &matrix)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setMatrix4fv("projection", matrix);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setColor(const glm::vec4 &color)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setVector4f("color", color);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setFaceValues(const std::vector<float> &values)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setFloatArray("heatmapValues", values);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}

void ShaderManager::setChangeOfBasis(const glm::mat4 &matrix)
{
    for (auto &pair : shaders)
    {
        pair.second->use();
        pair.second->setMatrix4fv("changeOfBasis", matrix);
    }
    useShader(activeShaderName); // Ensure the active shader is set after changing matrices
}