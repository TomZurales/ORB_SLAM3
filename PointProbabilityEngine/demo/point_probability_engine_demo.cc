// #include "Visualizer.h"
#include <memory>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>

#include "PointProbabilityEngine.h"
#include "Utility.h"

#include "MapImpl.h"
#include "CameraImpl.h"
#include "PointImpl.h"

int main()
{
    // Initialize a map with a grid of points
    MapImpl *theMap = new MapImpl();
    std::vector<PointImpl *> points;
    for (int i = -5; i < 5; i++)
    {
        for (int j = -5; j < 5; j++)
        {
            for (int k = -5; k < 5; k++)
            {
                points.push_back(new PointImpl(Eigen::Vector3f(i * 5, j * 5, k * 5)));
            }
        }
    }
    for (auto p : points)
        theMap->addPoint(p);

    // Initialize the camera
    CameraImpl *theCamera = new CameraImpl();

    // Initialize the Point Probability Engine
    auto engine = PointProbabilityEngine(PPEModel::ICOSAHEDRON, theMap, theCamera, true);

    // Define our camera pose
    Eigen::Vector3f cameraPos(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f lookAt(0.0f, 0.0f, 0.0f);

    float xVelocity = 0.0f;
    float yVelocity = 0.0f;
    float zVelocity = 0.0f;

    float lookAtXVelocity = 0.0f;
    float lookAtYVelocity = 0.0f;
    float lookAtZVelocity = 0.0f;

    float time = 0.0f;
    while (!engine.getShouldClose())
    {

        // Randomly add or subtract a small amount from all velocity values
        float deltaRange = 0.03f;
        xVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;
        yVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;
        zVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;
        lookAtXVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;
        lookAtYVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;
        lookAtZVelocity += (static_cast<float>(std::rand()) / RAND_MAX - 0.5f) * 2.0f * deltaRange;

        // Apply velocities to position and lookAt
        cameraPos.x() += xVelocity;
        cameraPos.y() += yVelocity;
        cameraPos.z() += zVelocity;
        lookAt.x() += lookAtXVelocity;
        lookAt.y() += lookAtYVelocity;
        lookAt.z() += lookAtZVelocity;

        // Clamp velocities to max of 1.0f
        float maxVel = 1.0f;
        xVelocity = std::max(-maxVel, std::min(maxVel, xVelocity));
        yVelocity = std::max(-maxVel, std::min(maxVel, yVelocity));
        zVelocity = std::max(-maxVel, std::min(maxVel, zVelocity));
        lookAtXVelocity = std::max(-maxVel, std::min(maxVel, lookAtXVelocity));
        lookAtYVelocity = std::max(-maxVel, std::min(maxVel, lookAtYVelocity));
        lookAtZVelocity = std::max(-maxVel, std::min(maxVel, lookAtZVelocity));

        // Lock cameraPos to radius of 8.0f from origin
        float cameraPosRadius = cameraPos.norm();
        if (cameraPosRadius > 8.0f)
        {
            cameraPos = cameraPos.normalized() * 8.0f;
        }

        // Lock lookAt to radius of 2.0f from origin
        float lookAtRadius = lookAt.norm();
        if (lookAtRadius > 2.0f)
        {
            lookAt = lookAt.normalized() * 2.0f;
        }

        // Get a random subset of points (around 100)
        std::vector<PPEPointInterface *> seenPoints;
        std::srand(static_cast<unsigned>(std::time(nullptr)));

        int numPointsToSelect = std::min(100, static_cast<int>(points.size()));
        std::vector<bool> selected(points.size(), false);

        for (int i = 0; i < numPointsToSelect; ++i)
        {
            int randomIndex;
            do
            {
                randomIndex = std::rand() % points.size();
            } while (selected[randomIndex]);

            selected[randomIndex] = true;
            seenPoints.push_back(static_cast<PPEPointInterface *>(points[randomIndex]));
        }

        auto pointsInView = theMap->getPointsInCameraView(theCamera, LookAtTransform(cameraPos, lookAt));
        // Filter pointsInView to keep only about 80% of the points
        if (!pointsInView.empty())
        {
            int numToKeep = static_cast<int>(pointsInView.size() * 0.8);
            std::random_shuffle(pointsInView.begin(), pointsInView.end());
            pointsInView.resize(numToKeep);
        }

        engine.Update(LookAtTransform(cameraPos, lookAt), pointsInView);

        // Increment time for smooth rotation
        time += 0.01f;
        if (time >= 2 * M_PI)
            time = 0;
    }
    return 0;
}