#include "stable_checker.h"

StableChecker::StableChecker(int window_size) : window_size(window_size) {}

bool StableChecker::addValue(float value) {
  values.push_back(value);
  if (values.size() > window_size) {
    values.erase(values.begin());
  }
  if (values.size() < window_size) {
    return false;
  }
  float mean = 0.0f;
  for (const auto &v : values) {
    mean += v;
  }
  mean /= static_cast<float>(values.size());
  float variance = 0.0f;
  for (const auto &v : values) {
    variance += (v - mean) * (v - mean);
  }
  variance /= static_cast<float>(values.size());
  return variance < 0.001f;
}