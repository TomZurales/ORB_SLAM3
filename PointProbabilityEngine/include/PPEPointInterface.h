#pragma once

#include <Eigen/Core>

class PPEPointInterface
{
private:
    bool isInViewFlag = false;
    bool isVisibleFlag = false;
    int id = -1; // Unique identifier for the point

public:
    // Returns the 3D position of the point in world coordinates
    virtual Eigen::Vector3f getPosition() const = 0;

    bool isInView() const { return isInViewFlag; }
    void setInView(bool inView) { isInViewFlag = inView; }
    bool isVisible() const { return isVisibleFlag; }
    void setVisible(bool visible) { isVisibleFlag = visible; }
    int getId() const { return id; }
    virtual bool isBad() = 0;
};