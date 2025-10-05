#pragma once

#include <map>
#include <vector>

#include "Atlas.h"
#include "Frame.h"
#include "PointProbabilityMetadataIcos.h"
#include "Settings.h"

namespace ORB_SLAM3 {
class PointProbability {
  Atlas *pAtlas;
  Settings *pSettings;

  std::map<MapPoint *, PointProbabilityMetadataIcos> pointData;

public:
  PointProbability(Atlas *pAtlas, Settings *pSettings);
  PointProbabilityMetadataIcos getPointProbabilityMetadata(MapPoint *mp);
  void GetExpectedMapPoints(Frame *pFrame);
};
} // namespace ORB_SLAM3
