#include "VBEE/vbee.h"
#include <atomic>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iomanip>
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
#define FALSE_NEGATIVE_RATE 0.3f
#define MAX_ITERS 2000
#define MAX_VIEWPOINTS 100000

VBEESettings global_vbee_settings;

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

  bool isInKeepout(Eigen::Vector3f point) const {
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
  bool isPointDeleted() const { return pointDeleted; }

  void restorePoint() { pointDeleted = false; }

  void enableErrors() { doErrors = true; }

  void disableErrors() { doErrors = false; }

  bool isSeen(Eigen::Vector3f point) {
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

  std::string getName() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "BR: " << blocked_rate << ", KR: " << keepout_rate;
    return ss.str();
  }
};

class StableChecker {
  int window_size;
  std::vector<float> values;
public:
  StableChecker(int window_size = 20) : window_size(window_size) {}

  bool addValue(float value) {
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
};

int main(int argc, char **argv) {
  if(argc != 1 && argc != 15) {
    std::cout << "Usage: vbee_test_random_accuracy bad_threshold init_pe damping_coeff init_observability observability_damping_coeff k n angle_threshold feedback_threshold sigmoid_steepness sigmoid_midpoint falseNegativeRate falsePositiveRate output_filename" << std::endl;
    return -1;
  }

  if(argc == 15) {
    global_vbee_settings.in_use = true;
    global_vbee_settings.weight_ransac = true;
    global_vbee_settings.bad_threshold = std::stof(argv[1]);
    global_vbee_settings.init_p_e = std::stof(argv[2]);
    global_vbee_settings.damping_coeff = std::stof(argv[3]);
    global_vbee_settings.init_observability = std::stof(argv[4]);
    global_vbee_settings.observability_damping_coeff = std::stof(argv[5]);
    global_vbee_settings.k = std::stoi(argv[6]);
    global_vbee_settings.n = std::stoi(argv[7]);
    global_vbee_settings.angle_threshold = std::stof(argv[8]);
    // global_vbee_settings.feedback_threshold = std::stof(argv[9]);
    global_vbee_settings.sigmoid_steepness = std::stod(argv[10]);
    global_vbee_settings.sigmoid_midpoint = std::stod(argv[11]);
    global_vbee_settings.falseNegativeRate = std::stof(argv[12]);
    global_vbee_settings.falsePositiveRate = std::stof(argv[13]);
  }

  std::string output_filename(argv[14]);
  // Check if worlds.bin exists and load it
  std::vector<World> worlds;
  std::set<std::pair<float, float>> existing_rate_pairs;

  std::ifstream test_file("worlds.bin");
  if (test_file.good()) {
    test_file.close();
    std::cout << "Loading worlds from 'worlds.bin'..." << std::endl;
    try {
      std::ifstream ifs("worlds.bin", std::ios::binary);
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

  std::vector<std::thread> test_threads;

  // Create output file and mutex for thread-safe writing
  std::ofstream data_file(output_filename);
  // Write CSV header

  // Store model estimation errors
  std::mutex psge_errors_mutex;
  std::vector<float> psge_errors(worlds.size(), -1.0f);

  std::mutex p_e_errors_mutex;
  std::vector<float> p_e_errors(worlds.size(), -1.0f);

  std::mutex obs_stab_mutex;
  std::vector<int> obs_stab_times(worlds.size(), 10000);

  std::mutex delete_time_mutex;
  std::vector<int> delete_times(worlds.size(), 10000);

  for (size_t world_idx = 0; world_idx < worlds.size(); ++world_idx) {
    test_threads.emplace_back([world = worlds[world_idx], world_idx,&psge_errors_mutex, &p_e_errors_mutex, &psge_errors, &p_e_errors, &obs_stab_mutex, &obs_stab_times, &delete_time_mutex, &delete_times]() mutable {
      VBEE vbee(0);
      float p_e_error = 0.0f;
      float psge_error = 0.0f;

      int observability_stab_time = -1;
      bool observability_stable = false;
      StableChecker observability_sc(50);

      int delete_time = 5000;
      bool is_deleted = false;

      for(int i = 0; i < 10000; i++) {
        Eigen::Vector3f viewpoint = world.getRandomValidViewpoint();
        bool seen = world.isSeen(viewpoint);
        vbee.Update(viewpoint, seen);

        float p_exists = vbee.Query();
        float p_seen_given_exists = vbee.GetPSeenGivenExists(viewpoint).first;
        float observability = vbee.GetObservability();

        if(!observability_stable && observability_sc.addValue(observability)) {
          observability_stable = true;
          observability_stab_time = i;
        }

        p_e_error += std::abs(p_exists - (world.isPointDeleted() ? 0.0f : 1.0f));
        psge_error += std::abs(p_seen_given_exists - (seen ? 1.0f : 0.0f));

        if(i == 10000 / 2) {
          world.deletePoint();
        }

        if(!is_deleted && p_exists < global_vbee_settings.bad_threshold) {
          is_deleted = true;
          delete_time = i - (10000 / 2);
          if(delete_time < 0) {
            delete_time = (10000 / 2);
          }
        }
      }
      {
        std::lock_guard<std::mutex> lock(psge_errors_mutex);
        psge_errors[world_idx] = psge_error / 10000.0f;
      }
      {
        std::lock_guard<std::mutex> lock(p_e_errors_mutex);
        p_e_errors[world_idx] = p_e_error / 10000.0f;
      }
      {
        std::lock_guard<std::mutex> lock(obs_stab_mutex);
        obs_stab_times[world_idx] = observability_stab_time;
      }
      {
        std::lock_guard<std::mutex> lock(delete_time_mutex);
        delete_times[world_idx] = delete_time;
      }
    });
  }

  for (auto &thread : test_threads) {
    thread.join();
  }

  float avg_p_e_error = 0.0f;
  for (const auto& error : p_e_errors) {
    avg_p_e_error += error;
  }
  avg_p_e_error /= static_cast<float>(p_e_errors.size());
  std::cout << "Average P(E) error: " << avg_p_e_error << std::endl;

  float avg_psge_error = 0.0f;
  for (const auto& error : psge_errors) {
    avg_psge_error += error;
  }
  avg_psge_error /= static_cast<float>(psge_errors.size());
  std::cout << "Average P(S|E) error: " << avg_psge_error << std::endl;
  float avg_obs_stab_time = 0.0f;
  for (const auto& time : obs_stab_times) {
    avg_obs_stab_time += static_cast<float>(time); 
  }

  avg_obs_stab_time /= static_cast<float>(obs_stab_times.size());
  std::cout << "Average Observability Stabilization Time: " << avg_obs_stab_time << std::endl;

  float avg_delete_time = 0.0f;
  for (const auto& time : delete_times) {
    avg_delete_time += static_cast<float>(time);
  }
  avg_delete_time /= static_cast<float>(delete_times.size());
  std::cout << "Average Delete Time: " << avg_delete_time << std::endl;

  data_file << avg_p_e_error << ","
            << avg_psge_error << ","
            << avg_obs_stab_time << ","
            << avg_delete_time << "\n";
  data_file.close();
  return 0;
}
      // for (int i = 0; i < 5000; i++) {
      //   Eigen::Vector3f viewpoint = world.getRandomValidViewpoint();
      //   bool seen = world.isSeen(viewpoint);
      //   vbee.Update(viewpoint, seen);

      //   float p_exists = vbee.Query();
      //   float p_seen_given_exists = vbee.GetPSeenGivenExists(viewpoint);
      //   float observability = vbee.GetObservability();

      //   if(!observability_stable && observability_sc.addValue(observability)) {
      //     observability_stable = true;
      //     observability_stab_time = i;
      //   }

      //   {
      //     std::lock_guard<std::mutex> lock(file_mutex);
      //     data_file << world_idx << "," << i
      //               << ","
      //               // << viewpoint.x() << ","
      //               // << viewpoint.y() << ","
      //               // << viewpoint.z() << ","
      //               // << (seen ? 1 : 0) << ","
      //               << p_exists << "," << p_seen_given_exists << "," << observability << "\n";
      //   }
      // }
      // world.deletePoint();
      // for (int i = 0; i < 5000; i++) {
      //   Eigen::Vector3f viewpoint = world.getRandomValidViewpoint();
      //   bool seen = world.isSeen(viewpoint);
      //   vbee.Update(viewpoint, seen);

      //   float p_exists = vbee.Query();
      //   float p_seen_given_exists = vbee.GetPSeenGivenExists(viewpoint);
      //   float observability = vbee.GetObservability();
      //   {
      //     std::lock_guard<std::mutex> lock(file_mutex);
      //     data_file << world_idx << "," << 5000 + i
      //               << ","
      //               << p_exists << "," << p_seen_given_exists << "," << observability << "\n";
      //   }
      // }

  //   // Pass any string to the program to generate worlds
  //   if (argc == 2) {
  //     std::mutex worlds_mutex;
  //     std::vector<std::thread> threads;

  //     std::atomic<int> n_done(0);

  //     std::vector<std::pair<float, float>> rate_pairs;

  //     int n_skipped = 0;
  //     for (float blocked_rate = 0.05f; blocked_rate <= 0.95f;
  //          blocked_rate += 0.05f) {
  //       for (float keepout_rate = 0.05f; keepout_rate <= 0.95f;
  //            keepout_rate += 0.05f) {
  //         if (existing_rate_pairs.find(std::pair<float, float>(
  //                 blocked_rate, keepout_rate)) == existing_rate_pairs.end())
  //                 {
  //           rate_pairs.emplace_back(blocked_rate, keepout_rate);
  //         } else {
  //           n_skipped++;
  //         }
  //       }
  //     }
  //     std::random_shuffle(rate_pairs.begin(), rate_pairs.end());

  //     std::cout << "Generating " << rate_pairs.size()
  //               << " new worlds (skipped " << n_skipped << " existing)..."
  //               << std::endl;
  //     for (const auto &rate_pair : rate_pairs) {
  //       threads.emplace_back(
  //           [blocked_rate = rate_pair.first, keepout_rate = rate_pair.second,
  //            &worlds, &worlds_mutex, &n_done, total = rate_pairs.size()]() {
  //             World w(blocked_rate, keepout_rate);

  //             n_done++;
  //             std::cout << n_done << " out of " << total << " worlds done."
  //                       << std::endl;
  //             std::lock_guard<std::mutex> lock(worlds_mutex);
  //             worlds.push_back(std::move(w));
  //             try {
  //               std::ofstream ofs("worlds.bin", std::ios::binary);
  //               boost::archive::binary_oarchive oa(ofs);
  //               oa << worlds;
  //               std::cout << "Successfully saved worlds to 'worlds.bin'"
  //                         << std::endl;
  //             } catch (const std::exception &e) {
  //               std::cerr << "Error saving worlds: " << e.what() <<
  //               std::endl;
  //             }
  //           });
  //     }

  //     for (auto &thread : threads) {
  //       thread.join();
  //     }

  //     // Save worlds to file
  //     std::cout << "Saving " << worlds.size() << " worlds to file..."
  //               << std::endl;
  //     std::lock_guard<std::mutex> lock(worlds_mutex);
  //     try {
  //       std::ofstream ofs("worlds.bin", std::ios::binary);
  //       boost::archive::binary_oarchive oa(ofs);
  //       oa << worlds;
  //       std::cout << "Successfully saved worlds to 'worlds.bin'" <<
  //       std::endl;
  //     } catch (const std::exception &e) {
  //       std::cerr << "Error saving worlds: " << e.what() << std::endl;
  //     }
  //   }