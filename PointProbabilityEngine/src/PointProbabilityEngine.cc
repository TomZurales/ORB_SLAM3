#include "PointProbabilityEngine.h"
#include "Viewer.h"

PointProbabilityEngine::PointProbabilityEngine(PPEModel model, PPEMapInterface *map, PPECameraInterface *camera, bool useViewer) : model(model), mapInterface(map), cameraInterface(camera), useViewer(useViewer)
{
  initializeModel();

  if (useViewer)
  {
    std::cout << "Visualizer enabled." << std::endl;
    viewer = new Viewer(this);
    init3DView(); // Initialize the 3D view if the viewer is used
  }
}

void PointProbabilityEngine::Update(Eigen::Matrix4f observationPose, std::vector<PPEPointInterface *> visiblePoints)
{
  if (!isPaused || doStep)
  {
    // auto mapPointsInCameraFrustum = mapInterface->getPointsInCameraView(cameraInterface, observationPose);

    backend->beginObservation(observationPose);
    // for (auto *point : mapPointsInCameraFrustum)
    // {
    //   if (std::find(visiblePoints.begin(), visiblePoints.end(), point) != visiblePoints.end())
    //   {
    //     point->setVisible(true);
    //     backend->addSuccessfulObservation(point);
    //   }
    //   else
    //   {
    //     backend->addFailedObservation(point);
    //   }
    // }

    for (auto point : visiblePoints)
    {
      point->setVisible(true);
      backend->addSuccessfulObservation(point);
    }

    backend->endObservation();
    // // Call the backend to update probabilities
    // backend->addObservation(camera->getPose(), seenPoints, notSeenPoints);
  }

  // If a viewer is used, update it with the current state
  if (useViewer)
  {
    viewer->update();
    viewer->render();
    if (viewer->getShouldClose())
    {
      delete viewer;
      viewer = nullptr;
      useViewer = false;
      std::cout << "Visualizer closed." << std::endl;

      // Shut down the engine
      shouldClose = true;
    }
  }
}

void PointProbabilityEngine::showState() const
{
  ImGui::Begin("Point Probability Engine State");

  ImGui::Text("# Map Points: %d    ", backend->getNumPoints());
  ImGui::SameLine();
  ImGui::Text("Camera Pose: (%.2f, %.2f, %.2f)", backend->getLastCameraPose()(0, 3), backend->getLastCameraPose()(1, 3), backend->getLastCameraPose()(2, 3));
  ImGui::End();
}

void PointProbabilityEngine::showControls()
{
  ImGui::Begin("Controls");

  ImGui::Checkbox("Pause", &isPaused);
  if (isPaused)
    doStep = ImGui::Button("Step");
  else
    doStep = false;

  ImGui::End();
}

void PointProbabilityEngine::showBackendState() const
{
  backend->showState(); // Assuming the backend has a method to show its state
}

void PointProbabilityEngine::initializeModel()
{
  switch (model)
  {
  case ICOSAHEDRON:
    backend = new IcosahedronBackend();
    break;

  case SPHERE:
    // Initialize for Sphere model
    break;

  default:
    std::cerr << "Unknown model type!" << std::endl;
    exit(EXIT_FAILURE);
  }
}