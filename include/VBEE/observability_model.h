#pragma once

#include "observation.h"
#include "viewpoint.h"
#include <boost/serialization/serialization.hpp>
#include <cstddef>
#include <vector>
#include <mutex>
#include <memory>

typedef struct {
  int k;
  int n;
  float angle_threshold;
  float feedback_threshold;
} ObservabilityModelParams;

class ObservationHistory {
  // Vector of (observation, age) pairs
  std::vector<std::pair<Observation, int>> history;
  std::shared_ptr<std::mutex> mtx_history;
  float savedPositiveObservationRatio = 0.5f;

  void deleteOldest() {
    if (history.empty())
      return;

    int maxAgeIndex = 0;
    for (int i = 1; i < history.size(); ++i) {
      if (history[i].second > history[maxAgeIndex].second) {
        maxAgeIndex = i;
      }
    }
    history.erase(history.begin() + maxAgeIndex);
  }

public:
  ObservationHistory() : mtx_history(std::make_shared<std::mutex>()) {}

  void AddObservation(const Observation &obs) {
    std::lock_guard<std::mutex> lock(*mtx_history);
    history.push_back(std::make_pair(obs, 0));
    for (auto &entry : history) {
      entry.second++; // Increment age of all observations
    }
  }

  void AddObservationDeleteOldest(const Observation &obs);

  void AddObservationReplaceNearestOpposite(const Observation &obs);

  const std::vector<Observation> getObservations() {
    std::lock_guard<std::mutex> lock(*mtx_history);
    std::vector<Observation> observations;
    for (const auto &entry : history) {
      observations.push_back(entry.first);
    }
    return observations;
  }

  const size_t size() {
    std::lock_guard<std::mutex> lock(*mtx_history);
    return history.size();
  }

  float updatePositiveObservationRatio() {
    std::lock_guard<std::mutex> lock(*mtx_history);
    float tot = 0.0f;
    for (const auto &entry : history) {
      tot += entry.first.s;
    }
    savedPositiveObservationRatio = tot / history.size();
    return savedPositiveObservationRatio;
  }

  const float getSavedPositiveObservationRatio() {
    std::lock_guard<std::mutex> lock(*mtx_history);
    return savedPositiveObservationRatio;
  }
};

class ObservabilityModel {
  // friend class VBEE;
  // friend class boost::serialization::access;
  // template <class Archive>
  // void serialize(Archive &ar, const unsigned int version) {
  //   ar & prev_observations;
  // }

public:
  ObservabilityModel() : mtx_prev_observations(std::make_shared<std::mutex>()) {};
  ObservabilityModel(ObservabilityModelParams params) : params(params), mtx_prev_observations(std::make_shared<std::mutex>()) {};
  ~ObservabilityModel() = default;

  // Returns (estimated P(S|E), confidence in estimate)
  std::pair<float, float> Estimate(const Viewpoint &viewpoint);

  std::pair<float, bool> Update(const Observation &observation);

  std::shared_ptr<std::mutex> mtx_prev_observations;

  float getPctFull() {
    std::lock_guard<std::mutex> lock(*mtx_prev_observations);
    return static_cast<float>(history.size()) / static_cast<float>(params.n);
  }

private:
  ObservabilityModelParams params;
  std::shared_ptr<std::atomic<float>> last_estimate{std::make_shared<std::atomic<float>>(0.0f)};
  std::shared_ptr<std::atomic<float>> last_confidence{std::make_shared<std::atomic<float>>(0.0f)};

protected:
  ObservationHistory history;
};