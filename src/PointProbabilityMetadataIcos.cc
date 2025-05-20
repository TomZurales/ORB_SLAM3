#include "PointProbabilityMetadataIcos.h"

namespace ORB_SLAM3
{
  PointProbabilityMetadataIcos::PointProbabilityMetadataIcos(MapPoint *pMapPoint) : pMapPoint(pMapPoint)
  {
  }
  float PointProbabilityMetadataIcos::getPexists()
  {
    return pExists;
  }
  void PointProbabilityMetadataIcos::offsetProbExists(float offset)
  {
    pExists += offset;
  }

  void PointProbabilityMetadataIcos::setIcosDist(int face, float dist)
  {
    if (icosDists[face] < dist)
    {
      std::cout << "Increasing distance on face " << face << " of map point " << pMapPoint->mnId << " to " << dist << std::endl;
      icosDists[face] = dist;
    }
  }

  float PointProbabilityMetadataIcos::getIcosDist(int face)
  {
    return icosDists[face];
  }

  int PointProbabilityMetadataIcos::getClosestIcosFaceIndex(Eigen::Vector3f cameraPose)
  {
    // Generate a vector from the point pose to the camera pose
    auto pointToCam = (pMapPoint->GetWorldPos() - cameraPose).normalized();
    float maxDot = -1.0f;
    int closestIdx = -1;
    for (size_t i = 0; i < icosFaceCenterVecs.size(); ++i)
    {
      float dot = pointToCam.dot(icosFaceCenterVecs[i]);
      if (dot > maxDot)
      {
        maxDot = dot;
        closestIdx = static_cast<int>(i);
      }
    }
    return closestIdx;
  }
}