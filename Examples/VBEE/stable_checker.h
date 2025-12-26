#ifndef STABLE_CHECKER_H
#define STABLE_CHECKER_H

#include <vector>

class StableChecker {
private:
  int window_size;
  std::vector<float> values;

public:
  StableChecker(int window_size = 20);
  bool addValue(float value);
};

#endif // STABLE_CHECKER_H