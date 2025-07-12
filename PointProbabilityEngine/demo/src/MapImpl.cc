#include "MapImpl.h"

std::vector<PPEPointInterface *> MapImpl::getPointsInCameraView(PPECameraInterface *camera, const Eigen::Matrix4f &viewMatrix) const
{
    std::vector<PPEPointInterface *> result;
    auto cameraParams = camera->getCameraParameters();
    float fx = cameraParams[0];
    float fy = cameraParams[1];
    float cx = cameraParams[2];
    float cy = cameraParams[3];
    int width = cameraParams[4];
    int height = cameraParams[5];

    for (auto *point : points)
    {
        point->setInView(false);
        point->setVisible(false);
        // Get point position in world coordinates
        Eigen::Vector4f worldPos(point->getPosition().x(), point->getPosition().y(), point->getPosition().z(), 1.0f);

        // Transform to camera space
        Eigen::Vector4f cameraPos = viewMatrix.inverse() * worldPos;

        // Check if point is in front of camera
        if (cameraPos.z() <= 0)
            continue;

        // Project to image coordinates
        float x = (fx * cameraPos.x() / cameraPos.z()) + cx;
        float y = (fy * cameraPos.y() / cameraPos.z()) + cy;

        // Check if point is within camera view
        if (x >= 0 && x < width && y >= 0 && y < height)
        {
            point->setInView(true);
            result.push_back(point);
        }
    }

    return result;
}