#pragma once
#include <algorithm>
#include <boost/serialization/serialization.hpp>
#include <string>

#include "VBEE/seen_status.h"
#include "existence_probability_estimator.h"
#include "observability_model.h"
#include "observation.h"

typedef struct {
  std::string model;
  float init_p_e;
  float damping_coeff;
  float init_observability;
  float observability_damping_coeff;
} VBEEParams;

class VBEE {
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & p_e;
    ar & observability;
    ar & model;
  }

public:
  VBEE() = default;
  VBEE(int);
  VBEE(VBEEParams, ObservabilityModelParams, int);
  ~VBEE() = default;

  void setInUse(bool use) { in_use = use; }
  void setWeightRansac(bool weight) { weight_ransac = weight; }

  float Update(Observation, bool commit, bool updatePExists = true);
  float Update(Eigen::Vector3f v, bool seen);
  float Update(Eigen::Vector3f observerPose, Eigen::Vector3f mapPointPose,
               bool seen);
  void Merge(VBEE &other);
  float Query(bool ransac = false) const;
  void Reset();
  int getNObservations() const { return n_observations; }

  void resetPe() { p_e = params.init_p_e; }

  SeenStatus GetSeenStatus() const { return seenStatus; }

  void SetSeenStatus(SeenStatus status) { seenStatus = status; }

  void PrintSettings() const;

  float GetObservability() const { return observability; }

  float GetPSeenGivenExists(Viewpoint v);

  float commitUncommittedObservation() {
    if (hasUncommittedObservation) {
      hasUncommittedObservation = false;
      return Update(uncommittedObservation, true);
    }
    return p_e;
  }

private:
  bool hasUncommittedObservation = false;
  Observation uncommittedObservation;
  // Parameters
  VBEEParams params;

  SeenStatus seenStatus = NOT_SEEN;

  bool beenSeen = false;

  ObservabilityModel model;
  ObservabilityModel model_dyn;
  ExistenceProbabilityEstimator epe;

  float p_e;
  float observability;

  int mpID = -1;

  void set_pe(float pe) { p_e = std::min(0.999f, std::max(0.001f, pe)); }

  void set_observability(float obs) {
    observability = std::min(0.75f, std::max(0.25f, obs));
  }

  bool in_use = true;
  bool weight_ransac = true; // Whether to use VBEE for RANSAC weighting
  int n_observations = 0;

  Viewpoint last_observer_position;

  std::vector<Observation> negative_observation_buffer;
  float UpdateMany(std::vector<Observation> observations);
};

struct VBEESettings {
  bool in_use = false;
  bool weight_ransac = false;
  bool fake_eliminations = true;
  float bad_threshold = 0.3847240852794481f;
  float init_p_e = 0.6192374475709618f;
  float damping_coeff = 0.6527140319328085f;
  float init_observability = 0.5556968284559984f;
  float observability_damping_coeff = 0.21177525800247346f;
  int k = 58;
  int n = 219;
  float angle_threshold = 1.5154090163903902f;
  float feedback_threshold = 0.36685880172731006f;
  double sigmoid_steepness = 3.9187396850862326;
  double sigmoid_midpoint = 100.4331555052815;
  float falseNegativeRate = 0.0642218339567518f;
  float falsePositiveRate = 0.005;
};

// struct VBEESettings {
//   bool in_use = false;
//   bool weight_ransac = false;
//   float bad_threshold = 0.05f;
//   float init_p_e = 0.9f;
//   float damping_coeff = 0.05f;
//   float init_observability = 0.5f;
//   float observability_damping_coeff = 0.02f;
//   int k = 10;
//   int n = 500;
//   float angle_threshold = 1.0f;
//   float feedback_threshold = 0.05f;
//   double sigmoid_steepness = 0.10;
//   double sigmoid_midpoint = 150.0;
//   float falseNegativeRate = 0.1f;
//   float falsePositiveRate = 0.0001f;
// };

  // bad_threshold: 0.3847240852794481
  // init_p_e: 0.6192374475709618
  // damping_coefficient: 0.6527140319328085
  // init_observability: 0.5556968284559984
  // observability_damping_coefficient: 0.21177525800247346
  // n: 219
  // k: 58
  // angle_threshold: 1.5154090163903902
  // feedback_threshold: 0.36685880172731006
  // sigmoid_steepness: 3.9187396850862326
  // sigmoid_midpoint: 316.4331555052815
  // false_negative_rate: 0.2642218339567518
  // false_positive_rate: 0.1340057182574792