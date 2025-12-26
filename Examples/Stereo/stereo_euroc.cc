/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

#include "MapPoint.h"
#include "Optimizer.h"
#include "VBEE/vbee.h"
#include "VBEE/TrackedStats.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

VBEESettings global_vbee_settings;
TrackedStats global_tracked_stats;

int main(int argc, char **argv) {
  if (argc != 6 && argc != 7 && argc != 8) {
    cerr << endl
         << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings "
            "path_to_sequence_folder path_to_times_file trajectory_file_name "
            "use_vbee vbee_ransac"
         << endl;

    return 1;
  }

  const int num_seq = 1;
  cout << "num_seq = " << num_seq << endl;
  bool bFileName = true;
  string file_name;
  if (bFileName) {
    file_name = string(argv[5]);
    cout << "file name: " << file_name << endl;
  }

  bool use_vbee = argc >= 7;
  bool vbee_ransac = argc == 8;

  if (use_vbee) {
    global_vbee_settings.in_use = true;
    std::cout << "Using VBEE" << std::endl;
  }
  if (vbee_ransac) {
    global_vbee_settings.weight_ransac = true;
    std::cout << "Using RANSAC" << std::endl;
  }
  // Load all sequences:
  int seq;
  vector<vector<string>> vstrImageLeft;
  vector<vector<string>> vstrImageRight;
  vector<vector<double>> vTimestampsCam;
  vector<int> nImages;

  vstrImageLeft.resize(num_seq);
  vstrImageRight.resize(num_seq);
  vTimestampsCam.resize(num_seq);
  nImages.resize(num_seq);

  std::vector<double> split_stamps;

  int tot_images = 0;
  for (seq = 0; seq < num_seq; seq++) {
    cout << "Loading images for sequence " << seq << "...";

    string pathSeq(argv[(2 * seq) + 3]);
    string pathTimeStamps(argv[(2 * seq) + 4]);

    string pathCam0 = pathSeq + "/mav0/cam0/data";
    string pathCam1 = pathSeq + "/mav0/cam1/data";

    LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq],
               vstrImageRight[seq], vTimestampsCam[seq]);
    cout << "LOADED!" << endl;

    nImages[seq] = vstrImageLeft[seq].size();
    tot_images += nImages[seq];

    string pathSplits = pathSeq + "/splits.txt";
    ifstream fSplits(pathSplits);
    if (fSplits.is_open()) {
      string line;
      while (getline(fSplits, line)) {
        if (!line.empty()) {
          try {
            double stamp = stod(line) / 1e9;
            split_stamps.push_back(stamp);
          } catch (const std::exception &) {
            // Skip lines that cannot be converted to long
          }
        }
      }
      fSplits.close();
    }
  }

  std::cout << "SPLITS FILE DETECTED! Here are the split stamps: " << std::endl;
  for (auto stamp : split_stamps) {
    std::cout << stamp << std::endl;
  }

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(tot_images);

  cout << endl << "-------" << endl;
  cout.precision(17);

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true,
                         use_vbee, vbee_ransac);

  cv::Mat imLeft, imRight;
  for (seq = 0; seq < num_seq; seq++) {

    // Seq loop
    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;
    int num_rect = 0;
    int proccIm = 0;
    for (int ni = 0; ni < nImages[seq]; ni++, proccIm++) {
      // Read left and right images from file
      imLeft = cv::imread(vstrImageLeft[seq][ni],
                          cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
      imRight = cv::imread(vstrImageRight[seq][ni],
                           cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);

      if (imLeft.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
             << endl;
        return 1;
      }

      if (imRight.empty()) {
        cerr << endl
             << "Failed to load image at: " << string(vstrImageRight[seq][ni])
             << endl;
        return 1;
      }

      double tframe = vTimestampsCam[seq][ni];

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t1 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t1 =
          std::chrono::monotonic_clock::now();
#endif

      // Pass the images to the SLAM system
      SLAM.TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(),
                       vstrImageLeft[seq][ni]);

      // // TEMPORARY: ONLY LEAVE IN WHILE CREATING VBEE GROUND TRUTHS
      //   if (ni == nImages[seq] - 2) {
      //     std::vector<ORB_SLAM3::Map *> vpMaps = SLAM.mpAtlas->GetAllMaps();
      //     for (ORB_SLAM3::Map *pMap : vpMaps) {
      //       if (pMap && pMap->GetAllKeyFrames().size() > 0) {
      //         ORB_SLAM3::Optimizer::GlobalBundleAdjustemnt(pMap, 200);
      //         std::cout << "Sleeping after optimization" << std::endl;
      //         std::this_thread::sleep_for(std::chrono::seconds(10));
      //       }
      //     }
      //   }
      // // END GROUND TRUTH CREATION

      if (std::find(split_stamps.begin(), split_stamps.end(), tframe) !=
          split_stamps.end()) {
        std::cout << "SPLIT STAMP FOUND. SETTING ALL MAP POINTS TO BE FROM "
                     "PREVIOUS MAP"
                  << std::endl;
        std::vector<ORB_SLAM3::Map *> vpMaps = SLAM.mpAtlas->GetAllMaps();
        for (ORB_SLAM3::Map *pMap : vpMaps) {
          for (ORB_SLAM3::MapPoint *pMP : pMap->GetAllMapPoints()) {
            pMP->fromPreviousMap = true;
            pMP->vbee.Reset();
          }
        }
      }

#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t2 =
          std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t2 =
          std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
      t_track =
          t_resize + t_rect +
          std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
              t2 - t1)
              .count();
      SLAM.InsertTrackTime(t_track);
#endif

      double ttrack =
          std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
              .count();

      vTimesTrack[ni] = ttrack;

      // Wait to load the next frame
      double T = 0;
      if (ni < nImages[seq] - 1)
        T = vTimestampsCam[seq][ni + 1] - tframe;
      else if (ni > 0)
        T = tframe - vTimestampsCam[seq][ni - 1];

      if (ttrack < T)
        usleep((T - ttrack) * 1e6); // 1e6
    }

    if (seq < num_seq - 1) {
      cout << "Changing the dataset" << endl;

      SLAM.ChangeDataset();
    }
  }

  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  if (bFileName) {
    const string kf_file = string(argv[5]) + "_KeyFrame.txt";
    const string f_file = string(argv[5]) + "_Frame.txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    global_tracked_stats.writeToFile(string(argv[5]) + "_TrackedStats.csv");
  } else {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
  }

  return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps) {
  ifstream fTimes;
  fTimes.open(strPathTimes.c_str());
  vTimeStamps.reserve(5000);
  vstrImageLeft.reserve(5000);
  vstrImageRight.reserve(5000);
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
      vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
      double t;
      ss >> t;
      vTimeStamps.push_back(t / 1e9);
    }
  }
}
