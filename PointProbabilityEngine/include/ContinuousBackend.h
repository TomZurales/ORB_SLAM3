#pragma once

#include "PPEBackend.h"
#include <eigen3/Eigen/Core>

class ContinuousBackend : public PPEBackend
{
public:
    void showState() override;
    PPEPointInterface *getActivePoint() const override;

private:
    void _addSuccessfulObservation(PPEPointInterface *point) override;
    void _addFailedObservation(PPEPointInterface *point) override;
};
