#pragma once

#include <eigen3/Eigen/Core>
#include "PPEPointInterface.h"

// This class is the interface for the backend model of the Point Probability Engine (PPE).
// The interface implements an addObservation method, which is called by the PPE and contains
// the camera pose, and the set of points which are seen and not seen which fall in the camera's view.
// The backend model is responsible for updating the probability of existence of the point
// based on the observations.
class PPEBackend
{
protected:
    std::map<PPEPointInterface *, float> pointProbExists;
    Eigen::Matrix4f currentCameraPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f lastCameraPose = Eigen::Matrix4f::Identity();
    bool isObservationActive = false;

    virtual void _addSuccessfulObservation(PPEPointInterface *point) = 0;
    virtual void _addFailedObservation(PPEPointInterface *point) = 0;

public:
    // virtual void addObservation(Eigen::Matrix4f cameraPose, std::vector<Point *> seen, std::vector<Point *> notSeen) = 0;
    void beginObservation(Eigen::Matrix4f cameraPose)
    {
        if (isObservationActive)
        {
            std::cerr << "Observation already active, cannot begin a new one." << std::endl;
            return;
        }
        lastCameraPose = currentCameraPose;
        currentCameraPose = cameraPose;
        isObservationActive = true;
    }
    void endObservation()
    {
        if (!isObservationActive)
        {
            std::cerr << "No active observation to end." << std::endl;
            return;
        }
        isObservationActive = false;
    }
    void addSuccessfulObservation(PPEPointInterface *point)
    {
        if (!isObservationActive)
        {
            std::cerr << "No active observation." << std::endl;
            return;
        }
        _addSuccessfulObservation(point);
    }
    void addFailedObservation(PPEPointInterface *point)
    {
        if (!isObservationActive)
        {
            std::cerr << "No active observation." << std::endl;
            return;
        }
        _addFailedObservation(point);
    }
    virtual void showState() = 0;
    virtual PPEPointInterface *getActivePoint() const = 0;
    virtual int getNumPoints() const = 0;
    virtual std::vector<PPEPointInterface *> getAllPoints() const = 0;
    Eigen::Matrix4f getLastCameraPose() const { return lastCameraPose; }
};