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

  float Update(Observation);
  float Update(Eigen::Vector3f v, bool seen);
  float Update(Eigen::Vector3f observerPose, Eigen::Vector3f mapPointPose,
               bool seen);
  void Merge(VBEE &other);
  float Query(bool ransac = false) const;
  void Reset();

  SeenStatus GetSeenStatus() const { return seenStatus; }

  void SetSeenStatus(SeenStatus status) { seenStatus = status; }

  void PrintSettings() const;

  float GetPSeenGivenExists(Viewpoint v);

private:
  // Parameters
  VBEEParams params;

  SeenStatus seenStatus = NOT_SEEN;

  bool beenSeen = false;

  ObservabilityModel model;
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
};

struct VBEESettings {
  bool in_use = false;
  bool weight_ransac = false;
};