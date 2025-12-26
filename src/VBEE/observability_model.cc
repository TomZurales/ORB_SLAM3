#include "VBEE/observability_model.h"
#include "VBEE/vbee.h"
#include <iostream>
#include <mutex>
#include <utility>
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
    // Find the nearest observation with opposite status without full sorting
    float min_distance = std::numeric_limits<float>::max();
    auto nearest_opposite_it = history.end();
    
    for (auto it = history.begin(); it != history.end(); ++it) {
      if (it->first.s != obs.s) {
        float distance = calculateEuclideanDistance(it->first.v, obs.v);
        if (distance < min_distance) {
          min_distance = distance;
          nearest_opposite_it = it;
        }
      }
    }

    // If opposite observation found, remove it; otherwise remove oldest
    if (nearest_opposite_it != history.end()) {
      history.erase(nearest_opposite_it);
    } else {
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
  std::vector<std::pair<Observation, float>> candidates_with_distance;
  std::vector<Observation> prev_observations = history.getObservations();
  
  // Pre-allocate to avoid reallocations
  candidates_with_distance.reserve(prev_observations.size());

  for (const auto &obs : prev_observations) {
    float distance = calculateEuclideanDistance(obs.v, viewpoint);
    
    // Early distance check before expensive angle calculation
    if (distance > global_vbee_settings.distance_threshold) {
      continue;
    }
    
    float angle = calculateAngleBetweenVectors(obs.v, viewpoint);
    if (angle > global_vbee_settings.angle_threshold) {
      continue;
    }
    
    candidates_with_distance.emplace_back(obs, distance);
  }

  if (candidates_with_distance.empty()) {
    (*last_estimate) = global_vbee_settings.unknown_psge_value;
    (*last_confidence) = 0.0f;
    return std::make_pair(last_estimate->load(), last_confidence->load());
  }

  // Use partial_sort since we only need the k nearest neighbors
  int count = std::min(static_cast<int>(candidates_with_distance.size()), global_vbee_settings.k);
  std::partial_sort(candidates_with_distance.begin(), 
                    candidates_with_distance.begin() + count,
                    candidates_with_distance.end(),
                    [](const std::pair<Observation, float> &a, const std::pair<Observation, float> &b) {
                      return a.second < b.second; // Sort by pre-calculated distance
                    });

  float sum = 0.0f;
  for (int i = 0; i < count; ++i) {
    sum += candidates_with_distance[i].first.s;
  }
  
  float estimate = sum / count;
  float confidence = static_cast<float>(count) / static_cast<float>(global_vbee_settings.k);
  
  // Store results
  (*last_estimate) = estimate;
  (*last_confidence) = confidence;
  
  return std::make_pair(estimate, confidence);
}

std::pair<float, bool> ObservabilityModel::Update(const Observation &observation) {
  // If we have not yet filled the observation history, just add the new
  // observation
  if (history.size() < global_vbee_settings.n) {
    history.AddObservation(observation);
    return std::make_pair(history.updatePositiveObservationRatio(), false);
  }

  // Cache atomic values to avoid multiple loads
  float current_estimate = last_estimate->load();
  float current_confidence = last_confidence->load();

  // Error is the difference between the observed status and the last
  // estimated observed status
  float error = std::abs(observation.s - current_estimate);

  // If we estimated correctly and with high confidence, ignore the
  // observation
  if (error < global_vbee_settings.max_error_threshold &&
      current_confidence > global_vbee_settings.min_confidence_threshold) {
    return std::make_pair(history.getSavedPositiveObservationRatio(), false);
  }

  // If the issue was low confidence, delete the oldest observation and add
  // the new one
  if (current_confidence <= global_vbee_settings.min_confidence_threshold) {
    history.AddObservationDeleteOldest(observation);
    return std::make_pair(history.updatePositiveObservationRatio(), true);
  }

  // If the problem was an incorrect estimate, delete the nearest observation with a different status
  history.AddObservationReplaceNearestOpposite(observation);
  return std::make_pair(history.updatePositiveObservationRatio(), true);
}