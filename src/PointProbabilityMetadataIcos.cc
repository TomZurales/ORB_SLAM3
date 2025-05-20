#include "PointProbabilityMetadataIcos.h"

namespace ORB_SLAM3
{
  float PointProbabilityMetadataIcos::getPexists()
  {
    return pExists;
  }
  void PointProbabilityMetadataIcos::offsetProbExists(float offset)
  {
    pExists += offset;
  }
}