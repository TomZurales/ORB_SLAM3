#include "PointProbability.h"

namespace ORB_SLAM3
{
    PointProbability::PointProbability(Atlas *pAtlas, Settings *pSettings) : pAtlas(pAtlas), pSettings(pSettings)
    {
        std::cout << "Initializing the PointProbability Engine" << std::endl;
    }

    std::vector<MapPoint *> PointProbability::GetExpectedMapPoints(Frame *pFrame)
    {
        // Get the map
        auto map = pAtlas->GetCurrentMap();

        // Get the pose from the frame
        auto pose = pFrame->GetPose();

        // Determine the set of map points inside the camera's frustum based on the frame's position

        // Camera Parameters
        float cx = pFrame->cx;
        float cy = pFrame->cy;
        float fx = pFrame->fx;
        float fy = pFrame->fy;

        Sophus::Matrix3f K;
        K << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;

        auto mapPoints = map->GetAllMapPoints();
        for (auto mapPoint : mapPoints)
        {
            if (!mapPoint)
                continue;
            auto pointPos = mapPoint->GetWorldPos();

            // auto pointInCameraCoords = pose.inverse() * Sophus::Vector3f(pointPos);
            // if (pointInCameraCoords.z() <= 0)
            // {
            //     continue;
            // }
            // std::cout << "Unnormalized: " << pointInCameraCoords << std::endl;
            // // auto pixelCoords = K * Sophus::Vector3f(pointPos.x() / pointPos.z(), pointPos.y() / pointPos.z(), 1);
            // // std::cout << "My Pixel Coords: " << pixelCoords << std::endl;

            // std::cout << "Their Pixel Coords: " << pSettings->camera1()->project(pointInCameraCoords) << std::endl;
        }
    }
}