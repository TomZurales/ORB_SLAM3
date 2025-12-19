#include "VBEE/vbee.h"
#include <atomic>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <thread>

#define N_TESTERS 10000
#define MIN_NOT_IN_KEEPOUT N_TESTERS * 0.2
#define MIN_NOT_BLOCKED N_TESTERS * 0.2
#define WORLD_RADIUS 10.0f
#define MIN_VIEWPOINT_DISTANCE 0.1f
#define KEEP_OUT_ANGLE_RAD 0.2f
#define BLOCKER_ANGLE_RAD 0.2f
#define FALSE_POSITIVE_RATE 0.005f
#define FALSE_NEGATIVE_RATE 0.1f
#define MAX_ITERS 2000
#define MAX_VIEWPOINTS 100000

// Serialization support for Eigen::Vector3f
namespace boost {
namespace serialization {
template <class Archive>
void serialize(Archive &ar, Eigen::Vector3f &v, const unsigned int version) {
  ar & v.x();
  ar & v.y();
  ar & v.z();
}
} // namespace serialization
} // namespace boost

class World {
  friend class boost::serialization::access;

  template <class Archive>
  void serialize(Archive &ar, const unsigned int version) {
    ar & blockers;
    ar & keep_outs;
    ar & blocked_rate;
    ar & keepout_rate;
  }

  std::vector<Eigen::Vector3f> blockers;
  std::vector<Eigen::Vector3f> keep_outs;

  float blocked_rate;
  float keepout_rate;

  bool pointDeleted = false;
  bool doErrors = false;

  // returns (meets_target_blocked, error)
  std::pair<bool, int> meetsTargetBlocked(int target_n_blocked) {
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
  std::pair<bool, int> meetsTargetKeepout(int target_n_in_keepout) {
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

public:
  World() = default; // Default constructor for serialization

  World(float goal_blocked_rate, float goal_keepout_rate)
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
      //   std::cout << "Iteration " << i << ": Trying with " << n_blockers
      //             << " blockers and " << n_keep_outs << " keepouts." <<
      //             std::endl;
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
          1.0f, std::min(1000.0f, best_error *
                                      (goal_blocked_rate + goal_blocked_rate) *
                                      ((1000 - i) / 1000.0f))));

      //   std::cout << "Best error: " << best_error << ", Jump: " << jump
      //             << std::endl;

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

  bool isInKeepout(Eigen::Vector3f point) {
    for (const auto &keep_out : keep_outs) {
      float angle = acos(point.normalized().dot(keep_out.normalized()));
      if (angle < KEEP_OUT_ANGLE_RAD && point.norm() > keep_out.norm()) {
        return true;
      }
    }
    return false;
  }

  std::pair<float, float> getRatePair() const {
    return std::pair<float, float>(blocked_rate, keepout_rate);
  }

  void deletePoint() { pointDeleted = true; }

  void restorePoint() { pointDeleted = false; }

  void enableErrors() { doErrors = true; }

  void disableErrors() { doErrors = false; }

  bool isSeen(Eigen::Vector3f point) {
    if (pointDeleted) {
      if (!doErrors) {
        return false;
      }
      if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
          FALSE_POSITIVE_RATE) {
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
            FALSE_POSITIVE_RATE) {
          return true;
        }
        return false;
      }
    }
    if (!doErrors) {
      return true;
    }
    if (static_cast<float>(rand()) / static_cast<float>(RAND_MAX) <
        FALSE_NEGATIVE_RATE) {
      return false;
    }
    return true;
  }

  Eigen::Vector3f getRandomViewpoint() {
    float length =
        static_cast<float>(rand()) / static_cast<float>(RAND_MAX) *
            (WORLD_RADIUS - MIN_VIEWPOINT_DISTANCE) +
        MIN_VIEWPOINT_DISTANCE; // Random length between MIN_VIEWPOINT_DISTANCE
                                // and WORLD_RADIUS
    return Eigen::Vector3f::Random().normalized() *
           length; // Random point inside sphere of radius WORLD_RADIUS
  }

  Eigen::Vector3f getRandomValidViewpoint() {
    Eigen::Vector3f point;
    do {
      point = getRandomViewpoint();
    } while (isInKeepout(point));
    return point;
  }
};

int main(int argc, char **argv) {
  // Check if test_worlds.bin exists and load it
  std::vector<World> worlds;
  std::set<std::pair<float, float>> existing_rate_pairs;

  std::ifstream test_file("test_worlds.bin");
  if (test_file.good()) {
    test_file.close();
    std::cout << "Loading worlds from 'test_worlds.bin'..." << std::endl;
    try {
      std::ifstream ifs("test_worlds.bin", std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      std::vector<World> loaded_worlds;
      ia >> loaded_worlds;
      std::cout << "Successfully loaded " << loaded_worlds.size()
                << " worlds from file." << std::endl;

      // Use loaded worlds instead of generating new ones
      for (const auto &world : loaded_worlds) {
        worlds.push_back(world);
        existing_rate_pairs.insert(world.getRatePair());
      }
    } catch (const std::exception &e) {
      std::cerr << "Error loading worlds: " << e.what() << std::endl;
      std::cout << "Generating new worlds instead..." << std::endl;
    }
  }

  // Pass any string to the program to generate worlds
  if (argc == 2) {
    std::mutex worlds_mutex;
    std::vector<std::thread> threads;

    std::atomic<int> n_done(0);

    std::vector<std::pair<float, float>> rate_pairs;

    int n_skipped = 0;
    for (float blocked_rate = 0.05f; blocked_rate <= 0.95f;
         blocked_rate += 0.05f) {
      for (float keepout_rate = 0.05f; keepout_rate <= 0.95f;
           keepout_rate += 0.05f) {
        if (existing_rate_pairs.find(std::pair<float, float>(
                blocked_rate, keepout_rate)) == existing_rate_pairs.end()) {
          rate_pairs.emplace_back(blocked_rate, keepout_rate);
        } else {
          n_skipped++;
        }
      }
    }
    std::random_shuffle(rate_pairs.begin(), rate_pairs.end());

    std::cout << "Generating " << rate_pairs.size()
              << " new worlds (skipped " << n_skipped << " existing)..."
              << std::endl;
    for (const auto &rate_pair : rate_pairs) {
      threads.emplace_back(
          [blocked_rate = rate_pair.first, keepout_rate = rate_pair.second,
           &worlds, &worlds_mutex, &n_done, total = rate_pairs.size()]() {
            World w(blocked_rate, keepout_rate);

            n_done++;
            std::cout << n_done << " out of " << total << " worlds done."
                      << std::endl;
            std::lock_guard<std::mutex> lock(worlds_mutex);
            worlds.push_back(std::move(w));
            try {
              std::ofstream ofs("test_worlds.bin", std::ios::binary);
              boost::archive::binary_oarchive oa(ofs);
              oa << worlds;
              std::cout << "Successfully saved worlds to 'test_worlds.bin'"
                        << std::endl;
            } catch (const std::exception &e) {
              std::cerr << "Error saving worlds: " << e.what() << std::endl;
            }
          });
    }

    for (auto &thread : threads) {
      thread.join();
    }

    // Save worlds to file
    std::cout << "Saving " << worlds.size() << " worlds to file..."
              << std::endl;
    std::lock_guard<std::mutex> lock(worlds_mutex);
    try {
      std::ofstream ofs("test_worlds.bin", std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << worlds;
      std::cout << "Successfully saved worlds to 'test_worlds.bin'" << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error saving worlds: " << e.what() << std::endl;
    }
  }

  std::vector<std::thread> test_threads;

  // Create output file and mutex for thread-safe writing
  std::ofstream data_file("data.csv");
  std::mutex file_mutex;
  
  // Write CSV header
  data_file << "world_idx,iteration,p_exists,observability\n";

  for (size_t world_idx = 0; world_idx < worlds.size(); ++world_idx) {
    test_threads.emplace_back([world = worlds[world_idx], world_idx, &data_file, &file_mutex]() mutable {
      VBEE vbee(0);
      for (int i = 0; i < 5000; i++) {
        Eigen::Vector3f viewpoint = world.getRandomValidViewpoint();
        bool seen = world.isSeen(viewpoint);
        vbee.Update(viewpoint, seen);

        float p_exists = vbee.Query();
        float p_seen_given_exists = vbee.GetPSeenGivenExists(viewpoint);
        float observability = vbee.GetObservability();

        // std::cout << "World Test - Camera position: (" << viewpoint.x() << ", "
        //           << viewpoint.y() << ", " << viewpoint.z() << ")" << std::endl;
        // std::cout << "Seen? " << (seen ? "Yes" : "No") << std::endl;
        // std::cout << "Estimated pExists: " << p_exists << std::endl;
        // std::cout << "Observability Estimate: " << p_seen_given_exists << std::endl;
        // std::cout << "Observability: " << observability << std::endl;

        // Thread-safe write to CSV file
        {
          std::lock_guard<std::mutex> lock(file_mutex);
          data_file << world_idx << ","
                    << i << ","
                    // << viewpoint.x() << ","
                    // << viewpoint.y() << ","
                    // << viewpoint.z() << ","
                    // << (seen ? 1 : 0) << ","
                    << p_exists << ","
                    << observability << "\n";
        }
      }
    });
  }

  for (auto &thread : test_threads) {
    thread.join();
  }
}