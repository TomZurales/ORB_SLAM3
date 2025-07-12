#include "IcosahedronBackend.h"
#include <algorithm>

// void IcosahedronBackend::addFailedObservation(Eigen::Matrix4f observerPose, Eigen::Vector3f pointPose)
// {
//   float distance = (observerPose.block<3, 1>(0, 3) - pointPose).norm();
//   int face = icos.getNearestFace(observerPose.block<3, 1>(0, 3));
//   std::cout << face << std::endl;
//   if (distance > faceDists[face])
//     return;
//   // Decrease p(seen | exists)
//   faceWeights[face] = (1 - alpha) * faceWeights[face];

//   // Use Bayes theorem to update pExists
//   float pSeenGivenExists = faceWeights[face];
//   float pNotSeenGivenExists = 1.0f - pSeenGivenExists;
//   float pNotSeenGivenNotExists = 1.0f; // If it doesn't exist, it can't be seen
//   float pExistsPrior = pExists;
//   float pNotExistsPrior = 1.0f - pExistsPrior;

//   // Bayes: p(exists|not seen) = [p(not seen|exists) * p(exists)] / p(not seen)
//   float pNotSeen = pNotSeenGivenExists * pExistsPrior + pNotSeenGivenNotExists * pNotExistsPrior;
//   if (pNotSeen > 0.0f)
//     pExists = (pNotSeenGivenExists * pExistsPrior) / pNotSeen;
//   else
//     pExists = 0.0f;
//   std::cout << pExists << std::endl;
// }

// void IcosahedronBackend::addSuccessfulObservation(Eigen::Matrix4f observerPose, Eigen::Vector3f pointPose)
// {
//   float distance = (observerPose.block<3, 1>(0, 3) - pointPose).norm();
//   int face = icos.getNearestFace(observerPose.block<3, 1>(0, 3));
//   std::cout << face << std::endl;

//   faceDists[face] = std::max(faceDists[face], distance);

//   // Increase P(seen | exists)
//   faceWeights[face] = alpha + ((1 - alpha) * faceWeights[face]);
//   pExists = 0.9;
//   std::cout << pExists << std::endl;
// }

void IcosahedronBackend::_addSuccessfulObservation(PPEPointInterface *point)
{
  cleanup();
  if (pointIcosMap.find(point) == pointIcosMap.end())
  {
    pointIcosMap[point] = new Icosahedron();
    pointIcosMap[point]->createValueBuffer("missedCount", 0.0f);
    pointIcosMap[point]->createValueBuffer("seenCount", 0.0f);
    pointIcosMap[point]->createValueBuffer("probSeen", 0.0f);
    pointIcosMap[point]->createValueBuffer("dists", 1.0f);
  }
  auto icos = pointIcosMap[point];
  float distance = (currentCameraPose.block<3, 1>(0, 3) - point->getPosition()).norm();
  Eigen::Vector3f direction = point->getPosition() - currentCameraPose.block<3, 1>(0, 3);
  int face = icos->getNearestFace(direction);

  // If we are still viewing the same face (the camera hasn't moved much), require that
  // there is a sufficient ammount of paralax to call this a new observation
  if (face == icos->getLastFace())
  {
    bool enoughMovement = true;

    for (auto prevObservation : icos->getPrevObservations())
    {
      // Check angular similarity using dot product of normalized vectors
      Eigen::Vector3f prevNorm = prevObservation.normalized();
      Eigen::Vector3f currNorm = direction.normalized();
      float similarity = prevNorm.dot(currNorm);

      // If angle between vectors is less than ~5 degrees, not enough movement
      float angleThreshold = std::cos(5.0f * M_PI / 180.0f); // cos(5°) ≈ 0.996
      if (similarity > angleThreshold)
      {
        enoughMovement = false;
        break;
      }
    }
    if (enoughMovement)
    {
      icos->addPrevObservation(direction);
    }
    else
    {
      // If we haven't moved enough, we don't update the last face
      return;
    }
  }
  else
  {
    icos->clearPrevObservations();
    icos->setLastFace(face);
  }

  if (icos->getValue("dists", face) < distance)
  {
    icos->setValue("dists", face, distance);
  }
  // Update the seen count for this face
  float newSeenCount = icos->getValue("seenCount", face) + 1.0f;
  float missedCount = icos->getValue("missedCount", face);
  icos->setValue("seenCount", face, newSeenCount);
  icos->setValue("probSeen", face, newSeenCount / (newSeenCount + missedCount));

  icos->setProbExists(0.95f);
}

void IcosahedronBackend::_addFailedObservation(PPEPointInterface *point)
{
  cleanup();
  if (pointIcosMap.find(point) == pointIcosMap.end())
  {
    pointIcosMap[point] = new Icosahedron();
    pointIcosMap[point]->createValueBuffer("missedCount", 0.0f);
    pointIcosMap[point]->createValueBuffer("seenCount", 0.0f);
    pointIcosMap[point]->createValueBuffer("probSeen", 0.0f);
    pointIcosMap[point]->createValueBuffer("dists", 1.0f);
  }
  auto icos = pointIcosMap[point];
  float distance = (currentCameraPose.block<3, 1>(0, 3) - point->getPosition()).norm();
  Eigen::Vector3f direction = point->getPosition() - currentCameraPose.block<3, 1>(0, 3);
  int face = icos->getNearestFace(direction);

  // If we are still viewing the same face (the camera hasn't moved much), require that
  // there is a sufficient ammount of paralax to call this a new observation
  if (face == icos->getLastFace())
  {
    bool enoughMovement = true;

    for (auto prevObservation : icos->getPrevObservations())
    {
      // Check angular similarity using dot product of normalized vectors
      Eigen::Vector3f prevNorm = prevObservation.normalized();
      Eigen::Vector3f currNorm = direction.normalized();
      float similarity = prevNorm.dot(currNorm);

      // If angle between vectors is less than ~5 degrees, not enough movement
      float angleThreshold = std::cos(5.0f * M_PI / 180.0f); // cos(5°) ≈ 0.996
      if (similarity > angleThreshold)
      {
        enoughMovement = false;
        break;
      }
    }
    if (enoughMovement)
    {
      icos->addPrevObservation(direction);
    }
    else
    {
      // If we haven't moved enough, we don't update the last face
      return;
    }
  }
  else
  {
    icos->clearPrevObservations();
    icos->setLastFace(face);
  }

  // We have never had a successful observation from this distance, so we update nothing
  if (icos->getValue("dists", face) < distance)
  {
    return;
  }

  // We have had a successful observation from this distance, so we update the missed count
  float newMissedCount = icos->getValue("missedCount", face) + 1.0f;
  float seenCount = icos->getValue("seenCount", face);
  float newProbSeen = seenCount / (seenCount + newMissedCount);
  icos->setValue("missedCount", face, newMissedCount);
  icos->setValue("probSeen", face, newProbSeen);

  // Perform a Bayesean update to set probExists based on the missed observation;
  float currentProbExists = icos->getPDFValue("probSeen", face);
  float pSeenGivenExists = currentProbExists;
  float pNotSeenGivenExists = 1.0f - pSeenGivenExists;
  float pNotSeenGivenNotExists = 1.0f; // If it doesn't exist, it can't be seen
  float pExistsPrior = icos->getProbExists();
  float pNotExistsPrior = 1.0f - pExistsPrior;

  // Bayes: p(exists|not seen) = [p(not seen|exists) * p(exists)] / p(not seen)
  float pNotSeen = pNotSeenGivenExists * pExistsPrior + pNotSeenGivenNotExists * pNotExistsPrior;
  if (pNotSeen > 0.0f)
  {
    float newProbExists = (pNotSeenGivenExists * pExistsPrior) / pNotSeen;
    icos->setProbExists(newProbExists);
  }
  else
  {
    icos->setProbExists(0.0f); // If pNotSeen is zero, we set probExists to zero
  }
}

void IcosahedronBackend::showState()
{
  ImGui::Begin("Icosahedron Backend State");

  if (pointIcosMapIndex != pointIcosMap.end())
  {
    // Step 1. Let's build the state to visualize
    auto selectedPoint = pointIcosMapIndex->first;
    auto selectedIcos = pointIcosMapIndex->second;
    auto currentFace = selectedIcos->getNearestFace(currentCameraPose.block<3, 1>(0, 3) - selectedPoint->getPosition());
    auto currentDistance = (currentCameraPose.block<3, 1>(0, 3) - selectedPoint->getPosition()).norm();
    auto isPointInView = selectedPoint->isInView();
    auto isPointVisible = selectedPoint->isVisible();

    ImGui::Text("Current Point ID: %d", selectedPoint->getId());
    ImGui::SameLine();
    if (ImGui::Button("Next"))
    {
      ++pointIcosMapIndex;
      if (pointIcosMapIndex == pointIcosMap.end())
      {
        pointIcosMapIndex = pointIcosMap.begin(); // Loop back to the first point
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Previous"))
    {
      if (pointIcosMapIndex == pointIcosMap.begin())
      {
        pointIcosMapIndex = std::prev(pointIcosMap.end()); // Loop back to the last point
      }
      else
      {
        --pointIcosMapIndex;
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Random"))
    {
      // Randomly select a point from the map
      auto it = pointIcosMap.begin();
      std::advance(it, rand() % pointIcosMap.size());
      pointIcosMapIndex = it;
    }
    ImGui::Text("Point Pose: (%.2f, %.2f, %.2f)",
                selectedPoint->getPosition().x(),
                selectedPoint->getPosition().y(),
                selectedPoint->getPosition().z());
    ImGui::Text("Current Face: %d", currentFace);
    ImGui::Text("Current Distance: %.2f", currentDistance);
    ImGui::Text("Point In View?: %s", isPointInView ? "Yes" : "No");
    ImGui::Text("Point Seen?: %s", isPointVisible ? "Yes" : "No");
    ImGui::Text("Probability Exists: %.2f", selectedIcos->getProbExists());
    ImGui::Separator();
    ImGui::BeginTable("Face Values", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg);
    ImGui::TableSetupColumn("Face", ImGuiTableColumnFlags_WidthFixed, 50.0f);
    ImGui::TableSetupColumn("Distance", ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableSetupColumn("Seen Count", ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableSetupColumn("Missed Count", ImGuiTableColumnFlags_WidthFixed, 100.0f);
    ImGui::TableHeadersRow();
    for (int i = 0; i < 20; ++i)
    {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("%d", i + 1);
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.2f", selectedIcos->getValue("dists", i));
      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%.2f", selectedIcos->getValue("seenCount", i));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.2f", selectedIcos->getValue("missedCount", i));
    }
    ImGui::EndTable();
    ImGui::Separator();
    show2DView("dists", selectedIcos, currentFace, isPointInView, isPointVisible);
  }
  else
  {
    ImGui::Text("No points in the backend.");
    pointIcosMapIndex = pointIcosMap.begin(); // Reset to the first point if no points are available
  }
  ImGui::End();
}

void IcosahedronBackend::show2DView(std::string name, Icosahedron *selectedIcos, int currentFace, bool isPointInView, bool isPointVisible) const
{
  ImGui::Dummy(ImVec2(0, 7.5)); // Add some space for the drawing
  ImGui::Text("2D View of %s", name.c_str());
  ImDrawList *drawList = ImGui::GetWindowDrawList();
  float scale = 40.0f; // Scale for the drawing
  for (int i = 0; i < selectedIcos->faces_2d.size(); ++i)
  {
    std::vector<int> face = selectedIcos->faces_2d[i];
    ImVec2 p1 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[0]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[0]].y() * scale);
    ImVec2 p2 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[1]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[1]].y() * scale);
    ImVec2 p3 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[2]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[2]].y() * scale);
    auto color = colorMap(selectedIcos->getNormalizedValue(name, i));
    drawList->AddTriangleFilled(p1, p2, p3, IM_COL32(color[0] * 255, color[1] * 255, color[2] * 255, 255));
  }
  for (int i = 0; i < selectedIcos->faces_2d.size(); ++i)
  {
    std::vector<int> face = selectedIcos->faces_2d[i];
    ImVec2 p1 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[0]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[0]].y() * scale);
    ImVec2 p2 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[1]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[1]].y() * scale);
    ImVec2 p3 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[2]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[2]].y() * scale);
    drawList->AddTriangle(p1, p2, p3, IM_COL32(0, 0, 0, 255));
    auto text = std::to_string(i + 1).c_str();

    // Calculate the center of the text to calculate an offset to center it
    ImVec2 textSize = ImGui::CalcTextSize(text);
    auto color = colorMap(selectedIcos->getNormalizedValue(name, i));
    // Determine if the background color is light or dark
    float luminance = 0.299f * color[0] + 0.587f * color[1] + 0.114f * color[2];
    ImU32 textColor = (luminance > 0.5f) ? IM_COL32(0, 0, 0, 255) : IM_COL32(255, 255, 255, 255);
    drawList->AddText(ImVec2((p1.x + p2.x + p3.x) / 3 - textSize.x / 2, (p1.y + p2.y + p3.y) / 3 - textSize.y / 2), textColor, text);
  }
  // Draw the border for the current face in a different color to highlight it
  std::vector<int> face = selectedIcos->faces_2d[currentFace];
  ImVec2 p1 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[0]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[0]].y() * scale);
  ImVec2 p2 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[1]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[1]].y() * scale);
  ImVec2 p3 = ImVec2(ImGui::GetCursorScreenPos().x + selectedIcos->vertices_2d[face[2]].x() * scale, ImGui::GetCursorScreenPos().y + selectedIcos->vertices_2d[face[2]].y() * scale);
  if (isPointInView && isPointVisible)
  {
    drawList->AddTriangle(p1, p2, p3, IM_COL32(0, 255, 0, 255)); // Green for visible points
  }
  else
  {
    drawList->AddTriangle(p1, p2, p3, IM_COL32(255, 255, 255, 255)); // White for not visible points
  }
  ImGui::Dummy(ImVec2(0, 7.5 + ((3 * sqrt(3) / 2) * scale))); // Add some space before the button
}

PPEPointInterface *IcosahedronBackend::getActivePoint() const
{
  if (pointIcosMapIndex != pointIcosMap.end())
  {
    return pointIcosMapIndex->first;
  }
  return nullptr; // Return nullptr if no points are available
}

Icosahedron *IcosahedronBackend::getActiveIcosahedron() const
{
  if (pointIcosMapIndex != pointIcosMap.end())
  {
    return pointIcosMapIndex->second;
  }
  return nullptr; // Return nullptr if no points are available
}

std::vector<PPEPointInterface *> IcosahedronBackend::getAllPoints() const
{
  std::vector<PPEPointInterface *> points;
  for (const auto &entry : pointIcosMap)
  {
    points.push_back(entry.first);
  }
  return points;
}