#pragma once

#include <vector>
#include <map>

#include "Atlas.h"
#include "Frame.h"
#include "Settings.h"
#include "PointProbabilityMetadataIcos.h"

namespace ORB_SLAM3
{
  class PointProbability
  {
    Atlas *pAtlas;
    Settings *pSettings;

    std::map<MapPoint *, PointProbabilityMetadataIcos> pointData;

  public:
    PointProbability(Atlas *pAtlas, Settings *pSettings);
    PointProbabilityMetadataIcos getPointProbabilityMetadata(MapPoint *mp);
    void GetExpectedMapPoints(Frame *pFrame);
  };
}
