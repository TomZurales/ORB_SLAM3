
#include "VBEE/vbee.h"

VBEE::VBEE(VBEEParams params, ObservabilityModelParams obsParams, int mpID)
    : params(params), model(obsParams), epe(), p_e(params.init_p_e), observability(params.init_observability), mpID(mpID), in_use(true) {}

VBEE::VBEE(int mpID): mpID(mpID)
{
    DBParams dbParams = DatabaseManager::Instance().getParams();

    in_use = dbParams.use_vbee;

    if(!in_use)
        return;

    params = VBEEParams{
        .model = "KNN",
        .init_p_e = dbParams.init_p_e,
        .damping_coeff = dbParams.damp_coeff,
        .init_observability = dbParams.init_obs,
        .observability_damping_coeff = dbParams.obs_damp_coeff
    };

    model = ObservabilityModel(ObservabilityModelParams{
        .k = dbParams.k,
        .n = dbParams.n,
        .angle_threshold = dbParams.a_th,
        .feedback_threshold = dbParams.f_th
    });

    epe = ExistenceProbabilityEstimator();
    p_e = params.init_p_e;
    observability = params.init_observability;

};

float VBEE::Update(Observation observation)
{
    if(!in_use)
        return 1.0f;
    if (observation.s == 0.0f)
    {
        seenStatus = NOT_SEEN;
    }
    else
    {
        seenStatus = SEEN;
    }

    if (!beenSeen && observation.s == 0.0f)
        return p_e;
    beenSeen = true;

    float prior = p_e;

    // Clamp model estimate between 0.001 and 0.999
    float model_estimate = std::min(0.999f, std::max(0.001f, model.Estimate(observation.v)));

    // Update the posterior probability using EPE
    float posterior = epe.Update(observation, prior, model_estimate);

    float weight = (1.0f - observability) * params.damping_coeff;
    p_e = std::min(0.999f, std::max(0.001f, prior * (1.0f - weight) + posterior * weight));

    float feedback = p_e - prior;
    model.Update(observation, feedback);

    float obs_damping_coeff = params.observability_damping_coeff;
    // Use s as the observation value
    observability = std::min(0.75f, std::max(0.25f, observability - obs_damping_coeff * (observation.s - model_estimate)));

    DatabaseManager::Instance().addVBEELine(mpID, std::min(0.999f, std::max(0.001f, p_e)), model_estimate, observability, seenStatus == SEEN);

    return std::min(0.999f, std::max(0.001f, p_e));
}

float VBEE::Update(Eigen::Vector3f v, bool seen)
{
    float s = seen ? 1.0f : 0.0f;
    return Update(Observation{.v = v, .s = s});
}

float VBEE::Update(Eigen::Vector3f observerPose, Eigen::Vector3f mapPointPose, bool seen)
{
    return Update((mapPointPose - observerPose).normalized(), seen);
}

float VBEE::Query() const
{
    if(!in_use)
        return 1.0f;
    return p_e;
}

void VBEE::Merge(VBEE &other)
{
    if(!in_use)
        return;
    // Merge the existence probabilities
    float avg_p_e = std::min(0.999f, std::max(0.001f, (p_e + other.p_e) / 2.0f));

    // Merge observability
    float avg_observability = std::min(0.75f, std::max(0.25f, (observability + other.observability) / 2.0f));

    for (auto observation : other.model.prev_observations)
    {
        this->Update(observation); // No feedback for merging
        this->set_observability(avg_observability);
        this->set_pe(avg_p_e);
    }
    this->set_observability(avg_observability);
    this->set_pe(avg_p_e);
}

void VBEE::Reset()
{
    p_e = params.init_p_e;
    observability = params.init_observability;

    model = ObservabilityModel(model);
    epe = ExistenceProbabilityEstimator();
}