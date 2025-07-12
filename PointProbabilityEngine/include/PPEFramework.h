#pragma once

#include <Eigen/Core>

class PPEFramework
{
public:
    virtual Eigen::Vector3f getCameraPose() = 0;
    virtual void getPointsInCameraFrustum() = 0;
    virtual void getPointsSeenByCameraInLastFrame() = 0;
};