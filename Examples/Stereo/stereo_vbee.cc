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
    if(argc != 2)
    {
        std::cout << "Usage: stereo_vbee <test name>" << std::endl;
        return 1;
    }

    std::string test_name(argv[1]);
    std::shared_ptr<DatabaseManager> dbManager = std::make_shared<DatabaseManager>(test_name + ".db");

    std::cout << "Number of trajectories in the database: " << dbManager->getNumTrajectories() << std::endl;
    return 0;

    const int num_seq = (argc - 3) / 2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName = (((argc - 3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        cout << "file name: " << file_name << endl;
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

    int tot_images = 0;
    for (seq = 0; seq < num_seq; seq++)
    {
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
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl
         << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to
    // process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);

    //   SLAM.ActivateLocalizationMode();
    bool newTrajectory = false;
    cv::Mat imLeft, imRight;
    double tframe = 0.0;
    double CONSTANT_TFRAME = 1.0 / 15.0;
    for (seq = 0; seq < num_seq; seq++)
    {
        // Seq loop
        double t_resize = 0;
        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
        int proccIm = 0;
        for (int ni = 0; ni < nImages[seq]; ni++, proccIm++)
        {
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

                if(result)
                {
                    std::cout << "Successful Trajectory Swap" << std::endl;
                    newTrajectory = false;
                }
            } else {
                // Pass the images to the SLAM system
                SLAM.TrackStereo(imLeft, imRight, tframe, vector<ORB_SLAM3::IMU::Point>(),
                                vstrImageLeft[seq][ni]);
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

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

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
