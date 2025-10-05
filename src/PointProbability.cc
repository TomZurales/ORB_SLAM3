#include "PointProbability.h"

namespace ORB_SLAM3 {
PointProbability::PointProbability(Atlas *pAtlas, Settings *pSettings)
    : pAtlas(pAtlas), pSettings(pSettings) {
  pointData = std::map<MapPoint *, PointProbabilityMetadataIcos>();
  std::cout << "Initializing the PointProbability Engine" << std::endl;
}

PointProbabilityMetadataIcos
PointProbability::getPointProbabilityMetadata(MapPoint *mp) {
  if (pointData.find(mp) == pointData.end()) {
    pointData[mp] = PointProbabilityMetadataIcos(mp);
  }
  return pointData[mp];
}

void PointProbability::GetExpectedMapPoints(Frame *pFrame) {
  auto expectedMapPoints = std::vector<MapPoint *>();
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

    if (point[0] <= 700 && point[0] >= 50 && point[1] <= 430 &&
        point[1] >= 50) {
      mapPoint->isCurrentlySeen = true;
      expectedMapPoints.push_back(mapPoint);
    } else {
      mapPoint->isCurrentlySeen = false;
    }
  }

  // return out;
  // 'out' now contains all the map points we would expect to see based on our
  // current estimated position. We can now go through, and update pExists
  // for each point which we expect to see but do not, and update the validViews
  // for each point which we expect to see and do see.

  for (auto point : expectedMapPoints) {
    if (pointData.find(point) == pointData.end()) {
      pointData[point] = PointProbabilityMetadataIcos(point);
    }

    auto pointProbMeta = pointData[point];

    float dist =
        (point->GetWorldPos() - pFrame->GetPose().translation().cast<float>())
            .norm();
    int face = pointProbMeta.getClosestIcosFaceIndex(
        pFrame->GetPose().translation().cast<float>());

    if (std::find(pFrame->mvpMapPoints.begin(), pFrame->mvpMapPoints.end(),
                  point) == pFrame->mvpMapPoints.end()) {
      // Expected to see point, but did not
      auto minDist = pointProbMeta.getIcosDist(face);

      if (dist <= minDist) {
        pointProbMeta.offsetProbExists(-0.01);
      }
    } else {
      // Expected to see point, and did
      pointProbMeta.setIcosDist(face, dist);
      pointProbMeta.offsetProbExists(1);
    }
  }
}
} // namespace ORB_SLAM3