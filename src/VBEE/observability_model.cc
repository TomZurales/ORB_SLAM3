#include "VBEE/observability_model.h"

float ObservabilityModel::Estimate(const Viewpoint &viewpoint) {
  std::vector<Observation> candidates;
  {
    std::lock_guard<std::mutex> lock(*mtx_prev_observations);
    for (const auto &obs : prev_observations) {
      float dot_product = obs.v.dot(viewpoint);
      float norm_obs = obs.v.norm();
      float norm_view = viewpoint.norm();
      float angle = std::acos(dot_product / (norm_obs * norm_view));
      if (angle < params.angle_threshold) {
        candidates.push_back(obs);
      }
    }
  }

  if (candidates.empty()) {
    last_estimate = 0.5f;
    return last_estimate; // No observations within angle threshold
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
  last_estimate = sum / count;
  return last_estimate;
}

float ObservabilityModel::updatePositiveObservationRatio() {
  float tot = 0.0f;
  for(int i = 0; i < prev_observations.size(); ++i) {
    tot += prev_observations[i].s;
  }
  past_positive_observation_ratio = tot / prev_observations.size();
  return past_positive_observation_ratio;
}

float ObservabilityModel::Update(const Observation &observation,
                                float feedback) {
  std::lock_guard<std::mutex> lock(*mtx_prev_observations);
  
  if (static_cast<int>(prev_observations.size()) < params.n) {
    prev_observations.push_back(observation);
    return updatePositiveObservationRatio();
  }

  float error = observation.s - last_estimate;
  if (error < params.feedback_threshold &&
      error > -params.feedback_threshold) {
    return past_positive_observation_ratio; // Ignore observations with low impact
  }
  std::sort(prev_observations.begin(), prev_observations.end(),
            [&observation](const Observation &a, const Observation &b) {
              return (a.v - observation.v).norm() <
                     (b.v - observation.v).norm();
            });

  for (int i = 0; i < prev_observations.size(); ++i) {
    if (prev_observations[i].s != observation.s) {
      prev_observations[i] = observation; // Update the observation
      return updatePositiveObservationRatio();
    }
  }

  // Replace the nearest observation if no nearby opposite observations are
  // found
  prev_observations[0] = observation;
  return past_positive_observation_ratio;
}