#pragma once

#include <vector>

#include "Atlas.h"
#include "Frame.h"
#include "Settings.h"

namespace ORB_SLAM3 {
class PointProbability {
  Atlas *pAtlas;
  Settings *pSettings;

public:
  PointProbability(Atlas *pAtlas, Settings *pSettings);

  std::vector<MapPoint *> GetExpectedMapPoints(Frame *pFrame);
};
} // namespace ORB_SLAM3
