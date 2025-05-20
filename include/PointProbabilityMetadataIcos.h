#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

namespace ORB_SLAM3
{
  class PointProbabilityMetadataIcos
  {
  private:
    float pExists = 1.0;
    std::vector<float> icosProbs = std::vector<float>(20, 0.0f);
    std::vector<float> icosDists = std::vector<float>(20, 1.0f);

    std::vector<Eigen::Vector3f> icosFaceCenterVecs = {
        Eigen::Vector3f(1.0, 0.0, 0.0)};

  public:
    float getPexists();
    void offsetProbExists(float offset);
  };
}