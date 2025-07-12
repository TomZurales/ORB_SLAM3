#pragma once

#include "PPEPointInterface.h"
#include "PPECameraInterface.h"

#include <vector>
#include <Eigen/Core>

class PPEMapInterface
{
public:
    // Returns all points in the map which fall within the camera's view frustum
    //
    // Note: The viewMatrix is the camera's world-to-camera transformation matrix
    virtual std::vector<PPEPointInterface *> getPointsInCameraView(PPECameraInterface *camera, const Eigen::Matrix4f &viewMatrix) const = 0;
};
