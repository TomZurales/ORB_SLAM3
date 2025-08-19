#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "Optimizer.h"
#include "DatabaseManager.h"

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: stereo_vbee <test name>" << std::endl;
        return 1;
    }

    std::string test_name(argv[1]);
    DatabaseManager::Init(test_name);

    std::vector<int> traj_ids = DatabaseManager::Instance().getTrajectoryIDs();

    if (traj_ids.empty())
    {
        std::cout << "No trajectories found in the database." << std::endl;
        return EXIT_FAILURE;
    }

    int num_seq = traj_ids.size();
    std::cout << "num_seq = " << num_seq << std::endl;

    vector<vector<string>> vstrImageLeft(num_seq);
    vector<vector<string>> vstrImageRight(num_seq);
    vector<vector<double>> vTimestampsCam(num_seq);
    vector<int> nImages(num_seq);

    int tot_images = 0;
    for (int seq = 0; seq < num_seq; ++seq)
    {
        std::cout << "Loading images for sequence " << seq << "...";
        Trajectory traj = DatabaseManager::Instance().getTrajectoryById(traj_ids[seq]);
        std::string dataset_root = traj.path;
        std::string pathTimeStamps = dataset_root + "/timestamps.txt";
        std::string pathCam0 = dataset_root + "/mav0/cam0/data";
        std::string pathCam1 = dataset_root + "/mav0/cam1/data";

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq],
                   vstrImageRight[seq], vTimestampsCam[seq]);
        std::cout << "LOADED!" << std::endl;

        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl
         << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM3::System SLAM("etc/ORBvoc.txt", "etc/VBEE.yaml", ORB_SLAM3::System::STEREO, true);

    //   SLAM.ActivateLocalizationMode();
    bool newTrajectory = false;
    cv::Mat imLeft, imRight;
    double tframe = 0.0;
    double CONSTANT_TFRAME = 1.0 / 15.0;
    for (int seq = 0; seq < num_seq; seq++)
    {
        // Seq loop
        double t_resize = 0;
        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
        int proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
            DatabaseManager::Instance().setTimestamp(tframe);
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],
                                cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],
                                 cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);

            if (imLeft.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << string(vstrImageLeft[seq][ni])
                     << endl;
                return 1;
            }

            if (imRight.empty())
            {
                cerr << endl
                     << "Failed to load image at: " << string(vstrImageRight[seq][ni])
                     << endl;
                return 1;
            }

            std::chrono::steady_clock::time_point t1 =
                std::chrono::steady_clock::now();
            if (newTrajectory)
            {
                bool result = SLAM.RelocalizeFrame(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni]);

                if (result)
                {
                    std::cout << "Successful Trajectory Swap" << std::endl;
                    newTrajectory = false;
                    DatabaseManager::Instance().nextTrajectory();
                    auto pose = SLAM.TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(),
                                                 vstrImageLeft[seq][ni]);

                    DatabaseManager::Instance().addFramePose(pose.translation().x(),
                                                             pose.translation().y(),
                                                             pose.translation().z(),
                                                             pose.unit_quaternion().x(),
                                                             pose.unit_quaternion().y(),
                                                             pose.unit_quaternion().z(),
                                                             pose.unit_quaternion().w());
                }
            }
            else
            {
                // Pass the images to the SLAM system
                auto pose = SLAM.TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(),
                                             vstrImageLeft[seq][ni]);

                DatabaseManager::Instance().addFramePose(pose.translation().x(),
                                                         pose.translation().y(),
                                                         pose.translation().z(),
                                                         pose.unit_quaternion().x(),
                                                         pose.unit_quaternion().y(),
                                                         pose.unit_quaternion().z(),
                                                         pose.unit_quaternion().w());
            }
            std::chrono::steady_clock::time_point t2 =
                std::chrono::steady_clock::now();

            double ttrack =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
                    .count();

            vTimesTrack[ni] = ttrack;

            // Wait to load the next frame
            double T = CONSTANT_TFRAME;

            if (ttrack < T)
                usleep((T - ttrack) * 1e6); // 1e6

            tframe += CONSTANT_TFRAME;
        }

        if (seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;
            newTrajectory = true;
        }
    }

    SLAM.GlobalBundleAdjustment();

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight,
                const string &strPathTimes, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
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
