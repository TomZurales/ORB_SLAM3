#pragma once

#include <vector>
#include <map>
#include <iostream>
#include <memory>

#include "IcosahedronBackend.h"
#include "PPEBackend.h"
#include "FloorPlane.h"
#include "PointCloud.h"
#include "IcosModel.h"
#include "ShaderManager.h"
#include "CameraModel.h"
#include "PPECameraInterface.h"
#include "PPEMapInterface.h"

enum PPEModel
{
  ICOSAHEDRON,
  SPHERE
};

class Viewer; // Forward declaration to avoid circular dependency

class PointProbabilityEngine
{
  PPECameraInterface *cameraInterface;
  PPEBackend *backend;
  PPEMapInterface *mapInterface;
  Viewer *viewer; // Pointer to the viewer, if used
  PPEModel model;

  bool useViewer;
  bool shouldClose = false;

  // Visualization Parameters
  bool isPaused = false;
  bool doStep = false;

private:
  void initializeModel();

public:
  PointProbabilityEngine(PPEModel model = PPEModel::ICOSAHEDRON, PPEMapInterface *map = nullptr, PPECameraInterface *camera = nullptr, bool useViewer = true);

  bool isReady() const
  {
    return (mapInterface != nullptr) && (cameraInterface != nullptr) && (backend != nullptr);
  }

  void setMap(PPEMapInterface *map) { mapInterface = map; }
  void setCamera(PPECameraInterface *camera) { cameraInterface = camera; }

  void Update(Eigen::Matrix4f, std::vector<PPEPointInterface *>);

  bool getShouldClose() const { return shouldClose; }

  // Visualization methods
  void showState() const;
  void showControls();
  void showBackendState() const;

  // Stuff related to the 3D View
private:
  uint camera_fbo;
  uint camera_texture;
  GLuint camera_rbo;

  uint static_fbo;
  uint static_texture;
  GLuint static_rbo;

  FloorPlane *floorPlane = nullptr;
  PointCloud *pointCloud = nullptr;
  IcosModel *icosModel = nullptr;
  CameraModel *cameraModel = nullptr;

  glm::mat4 staticViewMatrix = glm::lookAt(
      glm::vec3(10.0f, 10.0f, 30.0f), // Static camera position
      glm::vec3(0.0f, 0.0f, 0.0f),    // Look at point
      glm::vec3(0.0f, 1.0f, 0.0f)     // Up vector
  );

public:
  void init3DView();
  void showCameraView() const;
  void showStaticView();
};