#include "world.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <thread>
#include <iostream>
#include <atomic>

// returns (meets_target_blocked, error)
std::pair<bool, int> World::meetsTargetBlocked(int target_n_blocked) {
    int min_blocked =
        std::max(0, target_n_blocked - static_cast<int>(N_TESTERS * 0.025));
    int max_blocked = std::min(
        N_TESTERS, target_n_blocked + static_cast<int>(N_TESTERS * 0.025));
    int n_blocked = 0;
    std::atomic<int> n_blocked_atomic(0);
    std::vector<std::thread> threads;
    int num_threads = std::thread::hardware_concurrency();
    int tests_per_thread = N_TESTERS / num_threads;

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&, tests_per_thread, t]() {
        int local_blocked = 0;
        int start = t * tests_per_thread;
        int end =
            (t == num_threads - 1) ? N_TESTERS : (t + 1) * tests_per_thread;

        for (int i = start; i < end; ++i) {
          if (!isSeen(getRandomValidViewpoint())) {
            local_blocked++;
          }
        }
        n_blocked_atomic += local_blocked;
      });
    }

    for (auto &thread : threads) {
      thread.join();
    }

    n_blocked = n_blocked_atomic.load();
    int error = target_n_blocked - n_blocked;
    if (n_blocked < min_blocked) {
      return std::make_pair(false, error);
    } else if (n_blocked > max_blocked) {
      return std::make_pair(false, error);
    }
    return std::make_pair(true, error);
  }

  // returns (meets_target_in_keepout, error)
  std::pair<bool, int> World::meetsTargetKeepout(int target_n_in_keepout) {
    int min_in_keepout =
        std::max(0, target_n_in_keepout - static_cast<int>(N_TESTERS * 0.025));
    int max_in_keepout = std::min(
        N_TESTERS, target_n_in_keepout + static_cast<int>(N_TESTERS * 0.025));
    int n_in_keepout = 0;
    std::atomic<int> n_in_keepout_atomic(0);
    std::vector<std::thread> threads;
    int num_threads = std::thread::hardware_concurrency();
    int tests_per_thread = N_TESTERS / num_threads;

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&, tests_per_thread, t]() {
        int local_in_keepout = 0;
        int start = t * tests_per_thread;
        int end =
            (t == num_threads - 1) ? N_TESTERS : (t + 1) * tests_per_thread;

        for (int i = start; i < end; ++i) {
          if (isInKeepout(getRandomViewpoint())) {
            local_in_keepout++;
          }
        }
        n_in_keepout_atomic += local_in_keepout;
      });
    }

    for (auto &thread : threads) {
      thread.join();
    }

    n_in_keepout = n_in_keepout_atomic.load();
    int error = target_n_in_keepout - n_in_keepout;
    if (n_in_keepout < min_in_keepout) {
      return std::make_pair(false, error);
    } else if (n_in_keepout > max_in_keepout) {
      return std::make_pair(false, error);
    }
    return std::make_pair(true, error);
  }

World::World(float goal_blocked_rate, float goal_keepout_rate)
    : blocked_rate(goal_blocked_rate), keepout_rate(goal_keepout_rate) {
    int n_blockers = 1000 * goal_blocked_rate;
    int n_keep_outs = 1000 * goal_keepout_rate;
    bool found_solution = false;
    int jump = 200;
    std::vector<Eigen::Vector3f> best_blockers;
    std::vector<Eigen::Vector3f> best_keepouts;
    int best_error = N_TESTERS;

    for (int i = 0; i < MAX_ITERS; i++) {
      blockers.clear();
      keep_outs.clear();
      blockers.reserve(n_blockers);
      for (int j = 0; j < n_blockers; ++j) {
        blockers.emplace_back(getRandomViewpoint());
      }
      keep_outs.reserve(n_keep_outs);
      for (int j = 0; j < n_keep_outs; ++j) {
        keep_outs.emplace_back(getRandomViewpoint());
      }

      std::pair<bool, int> blocked_info;
      std::pair<bool, int> keepout_info;
      std::thread keepout_thread([&]() {
        keepout_info =
            meetsTargetKeepout(static_cast<int>(N_TESTERS * goal_keepout_rate));
      });
      std::thread blocked_thread([&]() {
        blocked_info =
            meetsTargetBlocked(static_cast<int>(N_TESTERS * goal_blocked_rate));
      });

      keepout_thread.join();
      blocked_thread.join();

      if (keepout_info.first && blocked_info.first) {
        found_solution = true;
        break;
      }

      int total_error =
          std::abs(keepout_info.second) + std::abs(blocked_info.second);
      if (total_error < best_error) {
        best_error = total_error;
        best_blockers = blockers;
        best_keepouts = keep_outs;
      }

      jump = static_cast<int>(std::max(
          1.0f,
          std::min(50.0f, best_error *
                              (goal_blocked_rate + goal_keepout_rate / 2) *
                              ((1000 - (3 * i)) / 1000.0f))));

      if (i > 100 && i % 50 == 0) {
        std::cout << getName() << " - Iteration " << i << ": Trying with "
                  << n_blockers << " blockers and " << n_keep_outs
                  << " keepouts." << std::endl;
        std::cout << "Best error: " << best_error << ", Jump: " << jump
                  << std::endl;
      }
      if (!keepout_info.first) {
        if (keepout_info.second < 0) {
          n_keep_outs =
              std::max(0, std::min(MAX_VIEWPOINTS, n_keep_outs - jump));
        } else {
          n_keep_outs =
              std::max(0, std::min(MAX_VIEWPOINTS, n_keep_outs + jump));
        }
      }

      if (!blocked_info.first) {
        if (blocked_info.second < 0) {
          n_blockers = std::max(0, std::min(MAX_VIEWPOINTS, n_blockers - jump));
        } else {
          n_blockers = std::max(0, std::min(MAX_VIEWPOINTS, n_blockers + jump));
        }
      }
    }
    if (!found_solution) {
      std::cout << "Warning: Could not find perfect blocker/keepout "
                   "configuration. Using best found."
                << std::endl;
      blockers = best_blockers;
      keep_outs = best_keepouts;
    }
  }

bool World::isInKeepout(Eigen::Vector3f point) const {
    for (const auto &keep_out : keep_outs) {
      float angle = acos(point.normalized().dot(keep_out.normalized()));
      if (angle < KEEP_OUT_ANGLE_RAD && point.norm() > keep_out.norm()) {
        return true;
      }
    }
    return false;
  }

std::pair<float, float> World::getRatePair() const {
    return std::pair<float, float>(blocked_rate, keepout_rate);
  }

void World::deletePoint() { pointDeleted = true; }

bool World::isPointDeleted() const { return pointDeleted; }

void World::restorePoint() { pointDeleted = false; }

void World::enableErrors() { doErrors = true; }

void World::disableErrors() { doErrors = false; }

bool World::isSeen(Eigen::Vector3f point) const {
    if (pointDeleted) {
      if (!doErrors) {
        return false;
      }
      if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
          global_vbee_settings.falsePositiveRate) {
        return true;
      }
      return false;
    }
    for (const auto &blocker : blockers) {
      float angle = acos(point.normalized().dot(blocker.normalized()));
      if (angle < BLOCKER_ANGLE_RAD && point.norm() > blocker.norm()) {
        if (!doErrors) {
          return false;
        }
        if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
            global_vbee_settings.falsePositiveRate) {
          return true;
        }
        return false;
      }
    }
    if (!doErrors) {
      return true;
    }
    if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
        global_vbee_settings.falseNegativeRate) {
      return false;
    }
    return true;
  }

Eigen::Vector3f World::getRandomViewpoint() const {
    float length =
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX) *
            (WORLD_RADIUS - MIN_VIEWPOINT_DISTANCE) +
        MIN_VIEWPOINT_DISTANCE; // Random length between MIN_VIEWPOINT_DISTANCE
                                // and WORLD_RADIUS
    return Eigen::Vector3f::Random().normalized() *
           length; // Random point inside sphere of radius WORLD_RADIUS
}

Eigen::Vector3f World::getRandomValidViewpoint() const {
    Eigen::Vector3f point;
    do {
      point = getRandomViewpoint();
    } while (isInKeepout(point));
    return point;
}

Eigen::Vector3f World::getRandomValidPositiveViewpoint() const {
    Eigen::Vector3f point;
    do {
      point = getRandomViewpoint();
    } while (isInKeepout(point) || !isSeen(point));
    return point;
}

Eigen::Vector3f World::getRandomValidNegativeViewpoint() const{
    Eigen::Vector3f point;
    do {
      point = getRandomViewpoint();
    } while (isInKeepout(point) || isSeen(point));
    return point;
}

std::string World::getName() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "BR: " << blocked_rate << ", KR: " << keepout_rate;
    return ss.str();
}