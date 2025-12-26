#include "VBEE/vbee.h"

#include <iostream>

#define OBSERVABILITY_MIN 0.001f
#define OBSERVABILITY_MAX 0.75f

extern VBEESettings global_vbee_settings;

VBEE::VBEE(VBEEParams params, ObservabilityModelParams obsParams, int mpID)
    : params(params), model(obsParams), epe(), p_e(params.init_p_e),
      observability(params.init_observability), mpID(mpID), in_use(true) {}

VBEE::VBEE(int mpID) : mpID(mpID) {

  params = VBEEParams{
      .model = "KNN",
      .init_p_e = global_vbee_settings.init_p_e,                     // 0.9f,
      .damping_coeff = global_vbee_settings.damping_coeff,           // 0.05f,
      .init_observability = global_vbee_settings.init_observability, // 0.5f,
      .observability_damping_coeff =
          global_vbee_settings.observability_damping_coeff // 0.02f
  };

  model = ObservabilityModel();

  model_dyn = ObservabilityModel();

  epe = ExistenceProbabilityEstimator();
  p_e = params.init_p_e;
  observability = params.init_observability;
};

inline float Sigmoid(float x) {
  float k = global_vbee_settings.sigmoid_steepness; // 0.10;
  float x0 = global_vbee_settings.sigmoid_midpoint; // 150.0;
  float out = 1.0 / (1.0 + std::exp(-k * (x - x0)));

  return std::max(0.1f, std::min(0.9f, out));
}

float VBEE::Update(Observation observation, bool commit, bool updatePExists) {
  if (!in_use)
    return 1.0f;
  if (observation.s == 0.0f) {
    seenStatus = NOT_SEEN;
  } else {
    seenStatus = SEEN;
  }

  if (!beenSeen && observation.s == 0.0f)
    return p_e;

  if (last_observer_position.norm() > 0 && observation.v.dot(last_observer_position) < 0.05f) {
    return p_e;
  }

  last_observer_position = observation.v;

  beenSeen = true;

  n_observations++;

  float prior = p_e;

  // Raise the bar for negative observations. Points tend to not be "under-seen" more often than "over-seen"
  if (observation.s == 0.0f) {
    negative_observation_buffer.push_back(observation);
    // 20 negative observations in a row causes all to be processed at once.
    if (negative_observation_buffer.size() == 20) {
      UpdateMany(negative_observation_buffer);
      negative_observation_buffer.clear();
      return p_e;
    } else {
      return p_e;
    }
  } else {
    // But one positive observation causes all negative observations to be ignored, only processing the positive one
    negative_observation_buffer.clear();
  }

  float model_estimate;

  // If the map point is seen, then the dynamic model is more accurate
  // because the map point still exists. If the point is not seen, then we
  // rely on the observability model from the last time the point was seen.
  if(observation.s == 1.0f) {
    auto model_output = model_dyn.Estimate(observation.v);
    model_estimate =
        std::min(0.999f, std::max(0.001f, model_output.first));
  } else {
    auto model_output = model.Estimate(observation.v);
    model_estimate =
        std::min(0.999f, std::max(0.001f, model_output.first));
  }
  // // Clamp model estimate between 0.001 and 0.999
  // float model_estimate =
  //     std::min(0.999f, std::max(0.001f, model.Estimate(observation.v)));

  // Update the posterior probability using EPE
  float posterior = epe.Update(observation, prior, model_estimate);

  float weight =
      (1.0f - observability) * params.damping_coeff * Sigmoid(n_observations);
  // float tmp_p_e = std::min(
  //     0.999f, std::max(0.001f, prior * (1.0f - weight) + posterior *
  //     weight));
  p_e = std::min(
      0.999f, std::max(0.001f, prior * (1.0f - weight) + posterior * weight));
  // if (updatePExists)
  //   p_e = tmp_p_e;

  if(observation.s == 1.0f) {
    observability = model_dyn.Update(observation).first;
    model = model_dyn; // Replace the static model with the current dynamic model
  } else {
    model_dyn.Update(observation);
  }

  // float obs_damping_coeff = params.observability_damping_coeff;
  // // Use s as the observation value
  // // If the estimated observability is higher than the actual, then our
  // observability should decrease.
  // // If the estimated observability is lower than the actual, then our
  // observability should increase.
  // // Minimum observability is set to 0.25 to prevent low observability from
  // preventing any updates.

  // float error = observation.s - model_estimate; // Positive if we
  // underestimated observability, negative if overestimated observability =
  // std::min(
  //     OBSERVABILITY_MAX,
  //     std::max(OBSERVABILITY_MIN, observability + (obs_damping_coeff *
  //     error)));

  return p_e;
}

float VBEE::UpdateMany(std::vector<Observation> observations) {
  for (auto observation : observations) {
    n_observations++;

    float prior = p_e;

    // Clamp model estimate between 0.001 and 0.999
    auto model_output = model.Estimate(observation.v);
    float model_estimate = std::min(0.999f, std::max(0.001f, model_output.first));

    // Update the posterior probability using EPE
    float posterior = epe.Update(observation, prior, model_estimate);

    float weight =
        (1.0f - observability) * params.damping_coeff * Sigmoid(n_observations);
    // float tmp_p_e = std::min(
    //     0.999f, std::max(0.001f, prior * (1.0f - weight) + posterior *
    //     weight));
    p_e = std::min(
        0.999f, std::max(0.001f, prior * (1.0f - weight) + posterior * weight));
    // if (updatePExists)
    //   p_e = tmp_p_e;

    float feedback = p_e - prior;
    observability = model_dyn.Update(observation).first;
  }
  return p_e;
}

std::pair<float, float> VBEE::GetPSeenGivenExists(Viewpoint v) {
  return model.Estimate(v);
}

float VBEE::Update(Eigen::Vector3f v, bool seen) {
  float s = seen ? 1.0f : 0.0f;
  return Update(Observation{.v = v, .s = s}, true);
}

float VBEE::Update(Eigen::Vector3f observerPose, Eigen::Vector3f mapPointPose,
                   bool seen) {
  return Update((mapPointPose - observerPose).normalized(), seen);
}

float VBEE::Query(bool ransac) const {
  if (!in_use)
    return 1.0f;
  if (ransac && !weight_ransac)
    return 1.0f;
  return p_e;
}

void VBEE::Merge(VBEE &other) {
  if (!in_use)
    return;
  // Merge the existence probabilities
  float avg_p_e = std::min(0.999f, std::max(0.001f, (p_e + other.p_e) / 2.0f));

  // Merge observability
  float avg_observability =
      std::min(OBSERVABILITY_MAX,
               std::max(OBSERVABILITY_MIN,
                        (observability + other.observability) / 2.0f));

  // for (auto observation : other.model.) {
  //   this->Update(observation, true); // No feedback for merging
  //   this->set_observability(avg_observability);
  //   this->set_pe(avg_p_e);
  // }
  this->set_observability(avg_observability);
  this->set_pe(avg_p_e);
  this->n_observations += other.n_observations;
}

void VBEE::Reset() { p_e = params.init_p_e; }

void VBEE::PrintSettings() const {
  std::string status = in_use ? "In Use" : "Not In Use";
  std::cout << "VBEE ID: " << mpID << "\n"
            << "Status: " << status << "\n"
            << "Existence Probability (p_e): " << p_e << "\n"
            << "Observability: " << observability << "\n"
            << "Parameters:\n"
            << "  Initial p_e: " << params.init_p_e << "\n"
            << "  Damping Coefficient: " << params.damping_coeff << "\n"
            << "  Initial Observability: " << params.init_observability << "\n"
            << "  Observability Damping Coefficient: "
            << params.observability_damping_coeff << "\n";
}