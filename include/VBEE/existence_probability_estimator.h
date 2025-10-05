#pragma once

#include "observation.h"

class ExistenceProbabilityEstimator {
public:
  ExistenceProbabilityEstimator();
  ~ExistenceProbabilityEstimator() = default;

  float Update(Observation o, float prior, float model_estimate);

private:
  float fn; // false negative rate
  float fp; // false positive rate
};