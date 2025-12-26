// Project headers
#include "VBEE/observability_model.h"
#include "VBEE/observation.h"
#include "VBEE/vbee.h"
#include "stable_checker.h"
#include "world.h"

// Boost serialization
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>

// Standard library
#include <chrono>
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

/**
 * Calculate the average error and confidence for a given world and model
 * Returns pair of (average_error, average_confidence)
 */
std::pair<float, float> estimateModelError(World world, ObservabilityModel model) {
  float total_error = 0.0f;
  float total_confidence = 0.0f;
  
  const std::string world_name = world.getName();
  const auto& observations = observation_cache[world_name];
  const float num_observations = static_cast<float>(observations.size());

  for (const auto& observation : observations) {
    auto estimate = model.Estimate(observation.v);
    float error = std::abs(estimate.first - observation.s);
    total_error += error;
    total_confidence += estimate.second;
  }
  
  return std::make_pair(total_error / num_observations, 
                       total_confidence / num_observations);
}

constexpr int EXPECTED_ARGC_WITH_PARAMS = 9;
constexpr int POST_STABILIZATION_TESTS = 1000;
constexpr int DEFAULT_STABILIZATION_FAILURE_TIME = 2000;
constexpr int MAX_STABILIZATION_FAILURE_TIME = 10000;

int main(int argc, char **argv) {
  // Validate command line arguments
  if (argc != 1 && argc != EXPECTED_ARGC_WITH_PARAMS) {
    std::cout << "Usage: characterize_observability_model <observability model "
                 "params> output_filename\n";
    return -1;
  }

  // Parse VBEE settings from command line
  if (argc == EXPECTED_ARGC_WITH_PARAMS) {
    global_vbee_settings.n = std::stoi(argv[1]);
    global_vbee_settings.k = std::stoi(argv[2]);
    global_vbee_settings.angle_threshold = std::stof(argv[3]);
    global_vbee_settings.distance_threshold = std::stof(argv[4]);
    global_vbee_settings.unknown_psge_value = std::stof(argv[5]);
    global_vbee_settings.min_confidence_threshold = std::stof(argv[6]);
    global_vbee_settings.max_error_threshold = std::stof(argv[7]);
  }

  // Initialize output files
  // std::ofstream("observability_model_history.txt").close(); // Clear previous history file
  // std::mutex history_file_mutex;
  // std::ofstream history_file("observability_model_history.txt", std::ios::app);
  
  const std::string output_filename(argv[8]);
  
  // Initialize world data structures
  std::vector<World> worlds;
  std::set<std::pair<float, float>> existing_rate_pairs;

  // Load existing worlds from binary file if available
  std::ifstream worlds_file("worlds.bin");
  if (worlds_file.good()) {
    worlds_file.close();
    std::cout << "Loading worlds from 'worlds.bin'..." << std::endl;
    
    try {
      std::ifstream ifs("worlds.bin", std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      std::vector<World> loaded_worlds;
      ia >> loaded_worlds;
      
      std::cout << "Successfully loaded " << loaded_worlds.size()
                << " worlds from file." << std::endl;

      // Store loaded worlds and track their rate pairs
      for (const auto &world : loaded_worlds) {
        worlds.push_back(world);
        existing_rate_pairs.insert(world.getRatePair());
      }
    } catch (const std::exception &e) {
      std::cerr << "Error loading worlds: " << e.what() << std::endl;
      std::cout << "Generating new worlds instead..." << std::endl;
    }
  }

  // Load or generate world observations
  std::ifstream observations_file("world_observations.bin");
  if (observations_file.good()) {
    observations_file.close();
    std::cout << "Loading world observations from 'world_observations.bin'..." << std::endl;
    
    try {
      std::ifstream ifs("world_observations.bin", std::ios::binary);
      boost::archive::binary_iarchive ia(ifs);
      ia >> observation_cache;
      
      std::cout << "Successfully loaded observations for "
                << observation_cache.size() << " worlds from file." << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error loading world observations: " << e.what() << std::endl;
      std::cout << "Generating new observations instead..." << std::endl;
      observation_cache.clear();
    }
  } else {
    // Generate new observations for each world
    std::cout << "Generating observations for each world..." << std::endl;
    
    for (size_t i = 0; i < worlds.size(); i++) {
      const std::string world_name = worlds[i].getName();
      std::vector<Observation> observations;
      observations.reserve(NUM_TEST_POINTS);
      
      for (int j = 0; j < NUM_TEST_POINTS; j++) {
        Eigen::Vector3f viewpoint = worlds[i].getRandomValidViewpoint();
        bool seen = worlds[i].isSeen(viewpoint);
        observations.push_back(Observation{viewpoint, seen ? 1.0f : 0.0f});
      }
      observation_cache[world_name] = observations;
    }
    
    // Archive generated observations to file for future use
    std::cout << "Archiving world observations to 'world_observations.bin'..." << std::endl;
    try {
      std::ofstream ofs("world_observations.bin", std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << observation_cache;
      
      std::cout << "Successfully archived " << observation_cache.size()
                << " world observations." << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "Error archiving world observations: " << e.what() << std::endl;
    }
    std::cout << "Done filling world cache." << std::endl;
  }

  // Initialize threading and data structures for world processing
  std::vector<std::thread> worker_threads;
  std::ofstream results_file(output_filename);

  // Thread-safe data structures for collecting results
  std::mutex world_stats_mutex;
  std::mutex stdout_mutex;
  
  std::vector<float> final_errors(worlds.size(), -1.0f);
  std::vector<float> final_confidences(worlds.size(), -1.0f);
  std::vector<int> stabilization_times(worlds.size(), MAX_STABILIZATION_FAILURE_TIME);
  std::vector<int> post_stabilization_updates(worlds.size(), MAX_STABILIZATION_FAILURE_TIME);

  // Process each world in separate threads
  for (size_t world_idx = 0; world_idx < worlds.size(); ++world_idx) {
    worker_threads.emplace_back([&worlds, world_idx, &stabilization_times, &world_stats_mutex, 
                                &final_errors, &post_stabilization_updates, &final_confidences,
                                &stdout_mutex/*, &history_file_mutex, &history_file*/]() {
      const World& world = worlds[world_idx];
      ObservabilityModel model;
      StableChecker error_stability_checker;
      StableChecker confidence_stability_checker;

      // Phase 1: Find stabilization point
      bool model_stabilized = false;
      const int max_iterations = global_vbee_settings.n + MAX_STABLE_ATTEMPTS;
      
      for (int iteration = 0; iteration < max_iterations; iteration++) {
        // Generate observation and update model
        Viewpoint viewpoint = world.getRandomValidViewpoint();
        bool is_seen = world.isSeen(viewpoint);
        Observation observation{viewpoint, is_seen ? 1.0f : 0.0f};
        model.Update(observation);

        // Evaluate current model performance
        auto result = estimateModelError(world, model);
        float current_error = result.first;
        float current_confidence = result.second;
        
        // Log progress to history file
        // {
        //   std::lock_guard<std::mutex> lock(history_file_mutex);
        //   history_file << world_idx << "," << iteration << ","
        //               << current_error << "," << current_confidence << std::endl;
        // }
        
        // Check for stabilization after minimum iterations
        if (iteration >= global_vbee_settings.n &&
            error_stability_checker.addValue(current_error) &&
            confidence_stability_checker.addValue(current_confidence)) {
          // Model has stabilized
          std::lock_guard<std::mutex> lock(world_stats_mutex);
          stabilization_times[world_idx] = iteration - global_vbee_settings.n;
          final_errors[world_idx] = current_error;
          final_confidences[world_idx] = current_confidence;
          model_stabilized = true;
          break;
        }
        
        // Handle case where model never stabilizes
        if (!model_stabilized && iteration == max_iterations - 1) {
          {
            std::lock_guard<std::mutex> lock(stdout_mutex);
            std::cout << "World " << world.getName()
                      << " did not stabilize within the maximum attempts." << std::endl;
          }
          std::lock_guard<std::mutex> lock(world_stats_mutex);
          stabilization_times[world_idx] = DEFAULT_STABILIZATION_FAILURE_TIME;
          final_errors[world_idx] = current_error;
          final_confidences[world_idx] = current_confidence;
        }
      }

      // Phase 2: Count updates after stabilization
      int update_count = 0;
      for (int test_iteration = 0; test_iteration < POST_STABILIZATION_TESTS; test_iteration++) {
        Viewpoint viewpoint = world.getRandomValidViewpoint();
        bool is_seen = world.isSeen(viewpoint);
        Observation observation{viewpoint, is_seen ? 1.0f : 0.0f};
        
        // Estimate before updating
        model.Estimate(viewpoint);
        auto update_result = model.Update(observation);

        // Count actual updates
        if (update_result.second) {
          update_count++;
        }
      }
      
      // Store final update count
      {
        std::lock_guard<std::mutex> lock(world_stats_mutex);
        post_stabilization_updates[world_idx] = update_count;
      }
    });
  }

  // Wait for all threads to complete
  for (auto &thread : worker_threads) {
    thread.join();
  }

  // Calculate and display aggregate statistics
  float average_error = 0.0f;
  for (const auto &error : final_errors) {
    average_error += error;
  }
  average_error /= static_cast<float>(final_errors.size());
  std::cout << "Average Model Error: " << average_error << std::endl;

  float average_confidence = 0.0f;
  for (const auto &confidence : final_confidences) {
    average_confidence += confidence;
  }
  average_confidence /= static_cast<float>(final_confidences.size());
  std::cout << "Average Model Confidence: " << average_confidence << std::endl;

  float average_stabilization_time = 0.0f;
  for (const auto &time : stabilization_times) {
    average_stabilization_time += static_cast<float>(time);
  }
  average_stabilization_time /= static_cast<float>(stabilization_times.size());
  std::cout << "Average Error Stabilization Time: " << average_stabilization_time / 100.0f << std::endl;

  float average_post_stab_updates = 0.0f;
  for (const auto &update_count : post_stabilization_updates) {
    average_post_stab_updates += static_cast<float>(update_count);
  }
  average_post_stab_updates /= static_cast<float>(post_stabilization_updates.size());
  std::cout << "Average Number of Updates after Stabilization: " << average_post_stab_updates / 1000.0f << std::endl;

  // Write results to output file
  results_file << average_error << "," << average_confidence << "," << average_stabilization_time / 100.0f << "," 
               << average_post_stab_updates / 1000.0f << "\n";
  results_file.close();
  
  return 0;
}
