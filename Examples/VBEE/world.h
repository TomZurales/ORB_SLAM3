#pragma once

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Core>

#include "vbee.h"

#define N_TESTERS 10000
#define MIN_NOT_IN_KEEPOUT N_TESTERS * 0.2
#define MIN_NOT_BLOCKED N_TESTERS * 0.2
#define WORLD_RADIUS 10.0f
#define MIN_VIEWPOINT_DISTANCE 0.1f
#define KEEP_OUT_ANGLE_RAD 0.2f
#define BLOCKER_ANGLE_RAD 0.2f
#define FALSE_POSITIVE_RATE 0.005f
#define FALSE_NEGATIVE_RATE 0.3f
#define MAX_ITERS 2000
#define MAX_VIEWPOINTS 100000
#define NUM_TEST_POINTS 1000
#define MAX_STABLE_ATTEMPTS 100

extern VBEESettings global_vbee_settings;

class World {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & blockers;
    ar & keep_outs;
    ar & blocked_rate;
    ar & keepout_rate;
  }

private:
  std::vector<Eigen::Vector3f> blockers;
  std::vector<Eigen::Vector3f> keep_outs;

  float blocked_rate;
  float keepout_rate;

  bool pointDeleted = false;
  bool doErrors = false;

  // returns (meets_target_blocked, error)
  std::pair<bool, int> meetsTargetBlocked(int target_n_blocked);

  // returns (meets_target_in_keepout, error)
  std::pair<bool, int> meetsTargetKeepout(int target_n_in_keepout);

public:
  World() = default; // Default constructor for serialization
  World(float goal_blocked_rate, float goal_keepout_rate);

  bool isInKeepout(Eigen::Vector3f point) const;
  std::pair<float, float> getRatePair() const;
  void deletePoint();
  bool isPointDeleted() const;
  void restorePoint();
  void enableErrors();
  void disableErrors();
  bool isSeen(Eigen::Vector3f point) const;
  Eigen::Vector3f getRandomViewpoint() const;
  Eigen::Vector3f getRandomValidViewpoint() const;
  Eigen::Vector3f getRandomValidPositiveViewpoint() const;
  Eigen::Vector3f getRandomValidNegativeViewpoint() const;
  std::string getName() const;
};