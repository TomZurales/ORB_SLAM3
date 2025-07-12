#include "Icosahedron.h"

Icosahedron::Icosahedron()
{
  // Initialize face centers
  for (auto face_vertices : faces)
  {
    float avg_x = 0;
    float avg_y = 0;
    float avg_z = 0;
    avg_x += vertices[face_vertices[0]].x();
    avg_x += vertices[face_vertices[1]].x();
    avg_x += vertices[face_vertices[2]].x();

    avg_y += vertices[face_vertices[0]].y();
    avg_y += vertices[face_vertices[1]].y();
    avg_y += vertices[face_vertices[2]].y();

    avg_z += vertices[face_vertices[0]].z();
    avg_z += vertices[face_vertices[1]].z();
    avg_z += vertices[face_vertices[2]].z();

    avg_x /= 3;
    avg_y /= 3;
    avg_z /= 3;

    face_centers.push_back(Eigen::Vector3f(avg_x, avg_y, avg_z).normalized());
  }
}

int Icosahedron::getNearestFace(Eigen::Vector3f in)
{
  auto in_norm = in.normalized();
  int nearest_index = 0;
  float max_dot = face_centers[0].dot(in_norm);

  for (size_t i = 1; i < face_centers.size(); ++i)
  {
    float dot = face_centers[i].dot(in_norm);
    if (dot > max_dot)
    {
      max_dot = dot;
      nearest_index = static_cast<int>(i);
    }
  }
  return nearest_index;
}

void Icosahedron::createValueBuffer(const std::string &name, float initialValue)
{
  valueBuffers[name] = std::vector<float>(faces.size(), initialValue);
}
float Icosahedron::getValue(const std::string &name, int index)
{
  if (valueBuffers.find(name) != valueBuffers.end())
  {
    if (index >= 0 && index < valueBuffers[name].size())
    {
      return valueBuffers[name][index];
    }
  }
  std::cerr << "Value buffer " << name << " does not exist." << std::endl;
  exit(EXIT_FAILURE);
}
void Icosahedron::setValue(const std::string &name, int index, float value)
{
  if (valueBuffers.find(name) != valueBuffers.end())
  {
    if (index >= 0 && index < valueBuffers[name].size())
    {
      valueBuffers[name][index] = value;
    }
  }
  else
  {
    std::cerr << "Value buffer " << name << " does not exist." << std::endl;
    exit(EXIT_FAILURE);
  }
}

float Icosahedron::getMinValue(const std::string &name)
{
  if (valueBuffers.find(name) != valueBuffers.end())
  {
    float minValue = valueBuffers[name][0];
    for (const auto &value : valueBuffers[name])
    {
      if (value < minValue)
      {
        minValue = value;
      }
    }
    return minValue;
  }
  std::cerr << "Value buffer " << name << " does not exist." << std::endl;
  exit(EXIT_FAILURE);
}
float Icosahedron::getMaxValue(const std::string &name)
{
  if (valueBuffers.find(name) != valueBuffers.end())
  {
    float maxValue = valueBuffers[name][0];
    for (const auto &value : valueBuffers[name])
    {
      if (value > maxValue)
      {
        maxValue = value;
      }
    }
    return maxValue;
  }
  std::cerr << "Value buffer " << name << " does not exist." << std::endl;
  exit(EXIT_FAILURE);
}
// Gets a value normalized to the range [0, 1]
float Icosahedron::getNormalizedValue(const std::string &name, int index)
{
  if (valueBuffers.find(name) != valueBuffers.end())
  {
    float minValue = getMinValue(name);
    float maxValue = getMaxValue(name);
    float value = getValue(name, index);
    return (value - minValue) / (maxValue - minValue);
  }
  std::cerr << "Value buffer " << name << " does not exist." << std::endl;
  return 0.0f; // Return 0 if the buffer does not exist or is empty
}