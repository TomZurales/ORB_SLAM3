#pragma once

#include <Eigen/Core>
#include "PPEPointInterface.h"

class PointImpl : public PPEPointInterface
{
    Eigen::Vector3f position;

public:
    PointImpl(const Eigen::Vector3f &pos) : position(pos) {}

    Eigen::Vector3f getPosition() const override;

    bool isBad() override { return false; }
};