#include "VBEE/observability_model.h"
#include <iostream>

float ObservabilityModel::Estimate(const Viewpoint &viewpoint) {
  std::vector<Observation> candidates;
  for (const auto &obs : prev_observations) {
    float dot_product = obs.v.dot(viewpoint);
    float norm_obs = obs.v.norm();
    float norm_view = viewpoint.norm();
    float angle = std::acos(dot_product / (norm_obs * norm_view));
    if (angle < params.angle_threshold) {
      candidates.push_back(obs);
    }
  }

  if (candidates.empty()) {
    return 0.5f; // No observations within angle threshold
  }

  // Sort candidates by euclidean distance to the viewpoint
  std::sort(candidates.begin(), candidates.end(),
            [&viewpoint](const Observation &a, const Observation &b) {
              return (a.v - viewpoint).norm() < (b.v - viewpoint).norm();
            });

  int count = std::min(static_cast<int>(candidates.size()), params.k);
  float sum = 0.0f;
  for (int i = 0; i < count; ++i) {
    sum += candidates[i].s;
  }
  return sum / count;
}

void ObservabilityModel::Update(const Observation &observation,
                                float feedback) {
  if (static_cast<int>(prev_observations.size()) < params.n) {
    prev_observations.push_back(observation);
    return;
  }

  if (feedback < params.feedback_threshold && feedback > -params.feedback_threshold) {
    return; // Ignore observations with low impact
  }
  std::sort(prev_observations.begin(), prev_observations.end(),
            [&observation](const Observation &a, const Observation &b) {
              return (a.v - observation.v).norm() <
                     (b.v - observation.v).norm();
            });

  for (int i = 0; i < prev_observations.size(); ++i) {
    if (prev_observations[i].s != observation.s) {
      prev_observations[i] = observation; // Update the observation
      return;
    }
  }

  // Replace the nearest observation if no nearby opposite observations are
  // found
  prev_observations[0] = observation;
}