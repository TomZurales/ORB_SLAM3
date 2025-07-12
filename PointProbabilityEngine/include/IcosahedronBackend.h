#pragma once

#include <vector>
#include <map>
#include <eigen3/Eigen/Core>
#include <iostream>

#include "imgui.h"

#include "PPEBackend.h"
#include "PPEPointInterface.h"
#include "Icosahedron.h"
#include "Utility.h"

class IcosahedronBackend : public PPEBackend
{
private:
  std::map<PPEPointInterface *, Icosahedron *> pointIcosMap = std::map<PPEPointInterface *, Icosahedron *>();

  // Visualization parameters
  std::map<PPEPointInterface *, Icosahedron *>::iterator pointIcosMapIndex = pointIcosMap.begin();

public:
  IcosahedronBackend() = default;

  void _addFailedObservation(PPEPointInterface *point) override;
  void _addSuccessfulObservation(PPEPointInterface *point) override;

  void showState() override;

  PPEPointInterface *getActivePoint() const override;

  void show2DView(std::string name, Icosahedron *selectedIcos, int currentFace, bool isPointInView, bool isPointVisible) const;
  Icosahedron *getActiveIcosahedron() const;

  int getNumPoints() const override { return static_cast<int>(pointIcosMap.size()); }

  std::vector<PPEPointInterface *> getAllPoints() const override;

  void cleanup()
  {
    // Remove any points that are marked as bad
    for (auto it = pointIcosMap.begin(); it != pointIcosMap.end();)
    {
      if (it->first->isBad())
      {
        delete it->second; // Free the allocated Icosahedron
        it = pointIcosMap.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
};