#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glm/glm.hpp>

Eigen::Matrix4f LookAtTransform(const Eigen::Vector3f &position, const Eigen::Vector3f &lookAt);
std::vector<float> colorMap(float value, float minValue = 0.0f, float maxValue = 1.0f);
glm::mat4 eigenToGlm(const Eigen::Matrix4f &m);
Eigen::Matrix4f glmToEigen(const glm::mat4 &m);