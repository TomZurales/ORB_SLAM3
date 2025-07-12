#pragma once

#include "PPECameraInterface.h"

class CameraImpl : public PPECameraInterface
{
public:
    std::vector<float> getCameraParameters() const override;
};