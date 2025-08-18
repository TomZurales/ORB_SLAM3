#include "existence_probability_estimator.h"

ExistenceProbabilityEstimator::ExistenceProbabilityEstimator()
{
    fn = 0.01;
    fp = 0.0001;
}

float ExistenceProbabilityEstimator::Update(Observation o, float prior, float model_estimate)
{
    float posterior;
    if (o.s == 1.0f)
    {
        float numerator = model_estimate * (1 - fn) * prior;
        float denominator = numerator + (fp * (1 - prior));
        posterior = numerator / denominator;
    }
    else
    {
        float numerator = (1 - model_estimate * (1 - fn)) * prior;
        float denominator = numerator + ((1 - fp) * (1 - prior));
        
        posterior = numerator / denominator;
    }
    return posterior;
}