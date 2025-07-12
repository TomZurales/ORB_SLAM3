#include "ContinuousBackend.h"

float vonMisesFisher(Eigen::Vector3f x, Eigen::Vector3f mu, float kappa)
{
    constexpr double pi = M_PI;
    // Avoid underflow for small kappa
    if (kappa < 1e-5)
    {
        return 1.0 / (4.0 * pi); // Uniform distribution on sphere
    }

    // Normalization constant for S^2 (unit sphere in R^3)
    double c = kappa / (2.0 * pi * (std::exp(kappa) - std::exp(-kappa)));

    // PDF value
    return c * std::exp(kappa * mu.dot(x));
}