#include "VBEE/observability_model.h"
#include "VBEE/vbee.h"
#include <iostream>
#include <mutex>
#include <vector>

extern VBEESettings global_vbee_settings;

float calculateAngleBetweenVectors(const Viewpoint &v1, const Viewpoint &v2) {
  // Calculate dot product
  float dot_product = v1.dot(v2);

  // Calculate magnitudes
  float norm_v1 = v1.norm();
  float norm_v2 = v2.norm();

  // Handle zero vectors
  if (norm_v1 == 0.0f || norm_v2 == 0.0f) {
    return 0.0f;
  }

  // Calculate cosine of angle
  float cos_angle = dot_product / (norm_v1 * norm_v2);

  // Clamp to valid range [-1, 1] to handle numerical errors
  cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));

  // Calculate angle in radians
  return std::acos(cos_angle);
}

float calculateEuclideanDistance(const Viewpoint &v1, const Viewpoint &v2) {
  // Calculate the difference vector
  Viewpoint diff = v1 - v2;

  // Return the magnitude (Euclidean distance)
  return diff.norm();
}

void ObservationHistory::AddObservationDeleteOldest(const Observation &obs) {
  std::lock_guard<std::mutex> lock( *mtx_history);
  if (history.size() == global_vbee_settings.n) {
    deleteOldest();
    history.push_back(std::make_pair(obs, 0));
  } else {
    history.push_back(std::make_pair(obs, 0));
  }
  for (auto &entry : history) {
    entry.second++; // Increment age of all observations
  }
}

void ObservationHistory::AddObservationReplaceNearestOpposite(
    const Observation &obs) {
  std::lock_guard<std::mutex> lock(*mtx_history);

  if (history.size() == global_vbee_settings.n) {
    // Sort by distance to the new observation's viewpoint
    std::sort(history.begin(), history.end(),
              [&obs](const std::pair<Observation, int> &a,
                     const std::pair<Observation, int> &b) {
                return calculateEuclideanDistance(a.first.v, obs.v) <
                       calculateEuclideanDistance(b.first.v, obs.v);
              });

    // Find the nearest observation with opposite status
    bool opposite_found = false;
    for (auto it = history.begin(); it != history.end(); ++it) {
      if (it->first.s != obs.s) {
        history.erase(it);
        break;
      }
    }

    // If no opposite observation found, remove the oldest observation
    if (history.size() == global_vbee_settings.n) {
      deleteOldest();
    }
    history.push_back(std::make_pair(obs, 0));
  } else {
    history.push_back(std::make_pair(obs, 0));
  }
  for (auto &entry : history) {
    entry.second++; // Increment age of all observations
  }
}

std::pair<float, float>
ObservabilityModel::Estimate(const Viewpoint &viewpoint) {
  std::vector<Observation> candidates;
  std::vector<Observation> prev_observations = history.getObservations();

  for (const auto &obs : prev_observations) {
    float angle = calculateAngleBetweenVectors(obs.v, viewpoint);
    float distance = calculateEuclideanDistance(obs.v, viewpoint);
    if (angle >
        global_vbee_settings.angle_threshold) {
      continue;
    }
    if (distance >
        global_vbee_settings.distance_threshold) {
      continue;
    }
    candidates.push_back(obs);
  }

  if (candidates.empty()) {
    (*last_estimate) = global_vbee_settings.unknown_psge_value;
    (*last_confidence) = 0.0f;
    return std::make_pair(last_estimate->load(), last_confidence->load());
  }

  // Sort candidates by euclidean distance to the viewpoint
  std::sort(candidates.begin(), candidates.end(),
            [&viewpoint](const Observation &a, const Observation &b) {
              return calculateEuclideanDistance(a.v, viewpoint) <
                     calculateEuclideanDistance(b.v, viewpoint);
            });

  int count =
      std::min(static_cast<int>(candidates.size()), global_vbee_settings.k);
  float sum = 0.0f;
  for (int i = 0; i < count; ++i) {
    sum += candidates[i].s;
  }
  (*last_estimate) = 
      sum / count;
  (*last_confidence) = static_cast<float>(count) / static_cast<float>(global_vbee_settings.k);
  // Returns the estimate and the ratio of neighbors to possible neighbors
  return std::make_pair(last_estimate->load(), last_confidence->load());
}

float ObservabilityModel::Update(const Observation &observation) {
  didUpdate = true;
  // If we have not yet filled the observation history, just add the new
  // observation
  if (history.size() < global_vbee_settings.n) {
    history.AddObservation(observation);
    return history.updatePositiveObservationRatio();
  }

  // Error is the difference between the observed status and the last
  // estimated observed status
  float error = std::abs(observation.s - last_estimate->load());

  // If we estimated correctly and with high confidence, ignore the
  // observation
  if (error < global_vbee_settings.max_error_threshold &&
      last_confidence->load() > global_vbee_settings.min_confidence_threshold) {
    didUpdate = false;
    return history.getSavedPositiveObservationRatio(); // Ignore low-confidence
                                                       // observations
  }

  // If the issue was low confidence, delete the oldest observation and add
  // the new one
  if (last_confidence->load() <= global_vbee_settings.min_confidence_threshold) {
    history.AddObservationDeleteOldest(observation);
    return history.updatePositiveObservationRatio();
  }

  // If the problem was an incorrect estimate, delete the nearest observation with a different status
  history.AddObservationReplaceNearestOpposite(observation);
  return history.updatePositiveObservationRatio();
}