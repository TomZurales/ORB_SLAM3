#pragma once

#include "PPEMapInterface.h"
#include "PointImpl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/frustum_culling.h>

class MapImpl : public PPEMapInterface
{
    std::vector<PointImpl *> points;

public:
    void addPoint(PointImpl *point)
    {
        points.push_back(point);
    }
    std::vector<PPEPointInterface *> getPointsInCameraView(PPECameraInterface *camera, const Eigen::Matrix4f &viewMatrix) const override;
};