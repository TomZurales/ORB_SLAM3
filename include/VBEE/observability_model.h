#pragma once

#include "observation.h"
#include "viewpoint.h"
#include <boost/serialization/serialization.hpp>
#include <vector>
#include <mutex>
#include <memory>

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
  ObservabilityModel() : mtx_prev_observations(std::make_shared<std::mutex>()) {};
  ObservabilityModel(ObservabilityModelParams params) : params(params), mtx_prev_observations(std::make_shared<std::mutex>()) {};
  ~ObservabilityModel() = default;

  float Estimate(const Viewpoint &viewpoint);

  float Update(const Observation &observation, float feedback);

  std::shared_ptr<std::mutex> mtx_prev_observations;

private:
  ObservabilityModelParams params;
  float updatePositiveObservationRatio();
  float past_positive_observation_ratio = 0.5f;
  float last_estimate = 0.5f;

protected:
  std::vector<Observation> prev_observations;
};