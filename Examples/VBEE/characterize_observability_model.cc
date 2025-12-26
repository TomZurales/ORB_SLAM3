#include "VBEE/observability_model.h"
#include "VBEE/observation.h"
#include "VBEE/vbee.h"
#include "stable_checker.h"
#include "world.h"
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <utility>

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

std::map<std::string, std::vector<Observation>> observation_cache;

std::pair<float, float> estimateModelError(World w, ObservabilityModel model) {
  float total_error = 0.0f;
  float total_confidence = 0.0f;

  std::mutex data_mutex;
  for (auto &observation : observation_cache[w.getName()]) {
    auto estimate = model.Estimate(observation.v);
    float error = std::abs(estimate.first - observation.s);
    total_error += error;
    total_confidence += estimate.second;
  }
  return std::make_pair(
      total_error / static_cast<float>(observation_cache[w.getName()].size()),
      total_confidence /
          static_cast<float>(observation_cache[w.getName()].size()));
}

int main(int argc, char **argv) {
  if (argc != 1 && argc != 9) {
    std::cout << "Usage: characterize_observability_model <observability model "
                 "params> output_filename"
              << std::endl;
    return -1;
  }

  if (argc == 9) {
    global_vbee_settings.n = std::stoi(argv[1]);
    global_vbee_settings.k = std::stoi(argv[2]);
    global_vbee_settings.angle_threshold = std::stof(argv[3]);
    global_vbee_settings.distance_threshold = std::stof(argv[4]);
    global_vbee_settings.unknown_psge_value = std::stof(argv[5]);
    global_vbee_settings.min_confidence_threshold = std::stof(argv[6]);
    global_vbee_settings.max_error_threshold = std::stof(argv[7]);
  }

  std::string output_filename(argv[8]);
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

  // Check if world_observations.bin exists and load it
  std::ifstream test_obs_file("world_observations.bin");
  if (test_obs_file.good()) {
    test_obs_file.close();
    std::cout << "Loading world observations from 'world_observations.bin'..."
              << std::endl;
    try {
      std::ifstream ifs("world_observations.bin", std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      ia >> observation_cache;
      std::cout << "Successfully loaded observations for "
                << observation_cache.size() << " worlds from file."
                << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error loading world observations: " << e.what()
                << std::endl;
      std::cout << "Generating new observations instead..." << std::endl;
      observation_cache.clear();
    }
  } else {
    std::cout << "Generating observations for each world..." << std::endl;
    for (int i = 0; i < worlds.size(); i++) {
      std::string world_name = worlds[i].getName();
      std::vector<Observation> observations;
      observations.reserve(NUM_TEST_POINTS);
      for (int j = 0; j < NUM_TEST_POINTS; j++) {
        Eigen::Vector3f viewpoint = worlds[i].getRandomValidViewpoint();
        bool seen = worlds[i].isSeen(viewpoint);
        observations.push_back(Observation{viewpoint, seen ? 1.0f : 0.0f});
      }
      observation_cache[world_name] = observations;
    }
    // Archive world observations to file
    std::cout << "Archiving world observations to 'world_observations.bin'..."
              << std::endl;
    try {
      std::ofstream ofs("world_observations.bin", std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << observation_cache;
      std::cout << "Successfully archived " << observation_cache.size()
                << " world observations." << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error archiving world observations: " << e.what()
                << std::endl;
    }
    std::cout << "Done filling world cache." << std::endl;
  }

  std::vector<std::thread> test_threads;

  std::ofstream data_file(output_filename);

  std::mutex world_stats_mutex;
  std::vector<float> errors(worlds.size(), -1.0f);
  std::vector<float> confidences(worlds.size(), -1.0f);
  std::vector<int> error_stab_times(worlds.size(), 10000);
  std::vector<int> n_updates(worlds.size(), 10000);
  std::mutex stdout_mutex;

  for (size_t world_idx = 0; world_idx < worlds.size(); ++world_idx) {
    test_threads.emplace_back([world = worlds[world_idx], world_idx,
                               &error_stab_times, &world_stats_mutex, &errors,
                               &n_updates, &confidences, &stdout_mutex]() mutable {
      {
        std::lock_guard<std::mutex> lock(stdout_mutex);
        std::cout << "Starting test for world " << world.getName() << std::endl;
      }
      ObservabilityModel model;
      StableChecker error_sc;
      StableChecker confidence_sc;

      for (int i = 0; i < MAX_STABLE_ATTEMPTS; i++) {
        Viewpoint vp = world.getRandomValidViewpoint();
        bool seen = world.isSeen(vp);

        Observation observation{vp, seen ? 1.0f : 0.0f};
        model.Update(observation);

        auto error_confidence = estimateModelError(world, model);
        if (i >= global_vbee_settings.n &&
            error_sc.addValue(error_confidence.first) &&
            confidence_sc.addValue(error_confidence.second)) {
          std::lock_guard<std::mutex> lock(world_stats_mutex);
          error_stab_times[world_idx] = i - global_vbee_settings.n;
          errors[world_idx] = error_confidence.first;
          confidences[world_idx] = error_confidence.second;
          break;
        }
      }

      int updates_count = 0;
      for (int i = 0; i < 1000; i++) {
        Viewpoint vp = world.getRandomValidViewpoint();
        bool seen = world.isSeen(vp);

        Observation observation{vp, seen ? 1.0f : 0.0f};
        model.Estimate(vp);
        model.Update(observation);

        if (model.hasUpdated())
          updates_count++;
      }
      {
        std::lock_guard<std::mutex> lock(world_stats_mutex);
        n_updates[world_idx] = static_cast<float>(updates_count) / 1000.0f;
      }
      {
        std::lock_guard<std::mutex> lock(stdout_mutex);
        std::cout << "Finished " << world.getName() << std::endl;
      }
    });
  }

  for (auto &thread : test_threads) {
    thread.join();
  }

  float avg_error = 0.0f;
  for (const auto &error : errors) {
    avg_error += error;
  }
  avg_error /= static_cast<float>(errors.size());
  std::cout << "Average Model Error: " << avg_error << std::endl;

  float avg_confidence = 0.0f;
  for (const auto &confidence : confidences) {
    avg_confidence += confidence;
  }
  avg_confidence /= static_cast<float>(confidences.size());
  std::cout << "Average Model Confidence: " << avg_confidence << std::endl;

  float avg_error_stab_time = 0.0f;
  for (const auto &time : error_stab_times) {
    avg_error_stab_time += static_cast<float>(time);
  }
  avg_error_stab_time /= static_cast<float>(error_stab_times.size());
  std::cout << "Average Error Stabilization Time: " << avg_error_stab_time
            << std::endl;

  float avg_n_updates = 0.0f;
  for (const auto &n_update : n_updates) {
    avg_n_updates += static_cast<float>(n_update);
  }
  avg_n_updates /= static_cast<float>(n_updates.size());
  std::cout << "Average Number of Updates after Stabilization: "
            << avg_n_updates << std::endl;

  data_file << avg_error << "," << avg_error_stab_time << "," << avg_n_updates
            << "\n";
  data_file.close();
  return 0;
}
