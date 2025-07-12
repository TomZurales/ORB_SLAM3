#pragma once

#include <vector>

class PPECameraInterface
{
public:
    // Returns camera parameters in the form [fx, fy, cx, cy, width, height]
    //
    // Note: This should be implemented to output the values post distortion correction,
    // ignoring any areas which are not part of the valid image.
    virtual std::vector<float> getCameraParameters() const = 0;
};