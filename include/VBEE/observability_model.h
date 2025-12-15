#pragma once

#include "observation.h"
#include "viewpoint.h"
#include <boost/serialization/serialization.hpp>
#include <vector>

typedef struct {
  int k;
  int n;
  float angle_threshold;
  float feedback_threshold;
} ObservabilityModelParams;

class ObservabilityModel {
  friend class VBEE;
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & prev_observations;
  }

public:
  ObservabilityModel() = default;
  ObservabilityModel(ObservabilityModelParams params) : params(params){};
  ~ObservabilityModel() = default;

  float Estimate(const Viewpoint &viewpoint);

  void Update(const Observation &observation, float feedback);

private:
  ObservabilityModelParams params;

protected:
  std::vector<Observation> prev_observations;
};