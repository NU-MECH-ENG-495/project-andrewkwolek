/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <cmath>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "MavlinkManager.hpp"
#include "CameraManager.hpp"

using namespace std;


bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

int main(int argc, char *argv[]) {
    if (argc < 1 || argc > 2) {
        cerr << endl
             << "Usage: ./slam_project (trajectory_file_name)"
             << endl;
        return 1;
    }

    gst_init(&argc, &argv);

    string vocab_file = "/home/kwolek/ws/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    string settings_file = "/home/kwolek/Northwestern/Winter2025/ME495/project-andrewkwolek/config/LL-HD-Camera.yaml";

    string file_name;

    if (argc == 2) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocab_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cout << "Starting Mavlink Manager." << endl;
    MavlinkManager mavlink_manager;
    std::thread mavlink_thread(&MavlinkManager::receive_mavlink_data, &mavlink_manager);

    cout << "Starting video stream." << endl;
    Video video;

    double timestamp;
    cv::Mat imCV;
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    cout << "Entering SLAM loop." << endl;
    while (!SLAM.isShutDown())
    {
        g_main_iteration(false);

        cv::Mat* frame = video.atomicFrame.load();
        if(frame) {
            imCV = video.atomicFrame.load()[0];
            timestamp = mavlink_manager.getCurrentTimestamp();
        }
        else {
            continue;
        }

        im = imCV.clone();

        vImuMeas = mavlink_manager.getIMUVector();

        if (vImuMeas.empty()) {
            continue;
        }

        if(imageScale != 0.5f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, timestamp, vImuMeas);

        // Clear the previous IMU measurements to load the new ones
        mavlink_manager.resetIMUVector();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop all threads
    SLAM.Shutdown();
    if (mavlink_thread.joinable()) {
        mavlink_thread.join();
    }
}
