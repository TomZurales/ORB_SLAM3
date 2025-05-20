#pragma once

#include <vector>
#include <eigen3/Eigen/Core>
#include "MapPoint.h"

namespace ORB_SLAM3
{
  class PointProbabilityMetadataIcos
  {
  private:
    float pExists = 1.0;
    std::vector<float> icosProbs = std::vector<float>(20, 0.0f);
    std::vector<float> icosDists = std::vector<float>(20, 1.0f);
    MapPoint *pMapPoint;

    std::vector<Eigen::Vector3f> icosFaceCenterVecs = {
        Eigen::Vector3f(-0.57735027, -0.57735027, -0.57735027),
        Eigen::Vector3f(-0.57735027, 0.57735027, -0.57735027),
        Eigen::Vector3f(0.35682209, 0., -0.93417236),
        Eigen::Vector3f(-0.35682209, 0., -0.93417236),
        Eigen::Vector3f(0., -0.93417236, 0.35682209),
        Eigen::Vector3f(0.57735027, -0.57735027, -0.57735027),
        Eigen::Vector3f(0., -0.93417236, -0.35682209),
        Eigen::Vector3f(-0.93417236, 0.35682209, 0.),
        Eigen::Vector3f(-0.93417236, -0.35682209, 0.),
        Eigen::Vector3f(-0.57735027, -0.57735027, 0.57735027),
        Eigen::Vector3f(-0.35682209, 0., 0.93417236),
        Eigen::Vector3f(-0.57735027, 0.57735027, 0.57735027),
        Eigen::Vector3f(0., 0.93417236, 0.35682209),
        Eigen::Vector3f(0., 0.93417236, -0.35682209),
        Eigen::Vector3f(0.57735027, 0.57735027, -0.57735027),
        Eigen::Vector3f(0.93417236, -0.35682209, 0.),
        Eigen::Vector3f(0.35682209, 0., 0.93417236),
        Eigen::Vector3f(0.57735027, -0.57735027, 0.57735027),
        Eigen::Vector3f(0.57735027, 0.57735027, 0.57735027),
        Eigen::Vector3f(0.93417236, 0.35682209, 0.)};

  public:
    PointProbabilityMetadataIcos() = default;
    PointProbabilityMetadataIcos(MapPoint *);

    float getPexists();
    void offsetProbExists(float offset);
    void setIcosDist(int face, float dist);
    float getIcosDist(int face);

    int getClosestIcosFaceIndex(Eigen::Vector3f cameraPose);
  };
}