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

  std::pair<float, float> GetPSeenGivenExists(Viewpoint v);

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
  // Use VBEE globally
  bool in_use = false;

  // Use VBEE P(E) for RANSAC weighting
  bool weight_ransac = false;

  bool random_sparsification = false;
  float random_sparsification_rate = 0.05f;

  // Only set the fake_bad flag when a map point is determined to be bad by VBEE
  // or random_sparsification. Used to determine the number of incorrect eliminations
  bool fake_eliminations = true;

  // Observability model parameters
  // Number of historical observations to store
  int n = 48;
  // Number of nearest neighbors to use for estimation
  int k = 10;
  // Maximum cosine angle between viewpoints to be considered neighbors
  float angle_threshold = 1.5302977172404146f;
  // Maximum distance between viewpoints to be considered neighbors
  float distance_threshold = 1.206475206960167f;
  // Value to return if no neighbors are found during estimation
  float unknown_psge_value = 0.10142435924100086f;
  // Confidence threshold below which the observation must be included
  float min_confidence_threshold = 0.07345447719215628f;
  // Error threshold above which the observation is included
  float max_error_threshold = 0.924289431543622f;

  // VBEE Parameters
  
  // Initial existence probability
  float init_p_e = 0.6192374475709618f;
  // P(E) below which a map point is considered bad
  float bad_threshold = 0.3847240852794481f;
  // Damping coefficient for P(E) updates
  float damping_coeff = 0.6527140319328085f;
  // Initial observability
  float init_observability = 0.5556968284559984f;
  // Damping coefficient for observability updates
  float observability_damping_coeff = 0.21177525800247346f;

  // Sigmoid parameters for mapping number of observations to feedback strength
  double sigmoid_midpoint = 400.4331555052815;
  double sigmoid_steepness = 3.9187396850862326;

  // Sensor noise characteristics
  float falseNegativeRate = 0.2642218339567518f; // Point not seen when it exists and is observable
  float falsePositiveRate = 0.005; // Point seen when it does not exist or is not observable
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