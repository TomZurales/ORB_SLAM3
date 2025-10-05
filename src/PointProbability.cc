#include "PointProbability.h"

namespace ORB_SLAM3 {
PointProbability::PointProbability(Atlas *pAtlas, Settings *pSettings)
    : pAtlas(pAtlas), pSettings(pSettings) {
  std::cout << "Initializing the PointProbability Engine" << std::endl;
}

std::vector<MapPoint *> PointProbability::GetExpectedMapPoints(Frame *pFrame) {
  auto out = std::vector<MapPoint *>();
  // Get the map
  auto map = pAtlas->GetCurrentMap();

  // Get the pose from the frame
  auto pose = pFrame->GetPose();

  // Determine the set of map points inside the camera's frustum based on the
  // frame's position
  auto mapPoints = map->GetAllMapPoints();
  for (auto mapPoint : mapPoints) {
    auto pointPos = mapPoint->GetWorldPos();

    auto pointInCameraCoords = pose * Sophus::Vector3f(pointPos);
    if (pointInCameraCoords.z() <= 0) {
      mapPoint->isCurrentlySeen = false;
      continue;
    }

    auto point = pSettings->camera1()->project(pointInCameraCoords);

    if (point[0] <= 752 && point[0] >= 0 && point[1] <= 480 && point[1] >= 0) {
      mapPoint->isCurrentlySeen = true;
      out.push_back(mapPoint);
    } else {
      mapPoint->isCurrentlySeen = false;
    }
  }

  return out;
}
} // namespace ORB_SLAM3