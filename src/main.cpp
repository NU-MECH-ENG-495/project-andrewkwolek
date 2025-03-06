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

#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"
#include "MavlinkManager.hpp"
#include "CameraManager.hpp"

using namespace std;

ORB_SLAM3::IMU::Point convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data);
double getTimestampInSeconds(int64_t timestamp_us);

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

    MavlinkManager mavlink_manager;
    std::thread mavlink_thread(&MavlinkManager::receive_mavlink_data, &mavlink_manager);

    Video video;

    double timestamp;
    cv::Mat imCV;
    cv::Mat im;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    while (!SLAM.isShutDown())
    {
        g_main_iteration(false);

        cv::Mat* frame = video.atomicFrame.load();
        if(frame) {
            imCV = video.atomicFrame.load()[0];
        }
        else {
            continue;
        }

        im = imCV.clone();

        mavlink_raw_imu_t imu_data;
        if (!mavlink_manager.imu_buffer.is_empty()) {
            imu_data = mavlink_manager.imu_buffer.get_latest_data();
            ORB_SLAM3::IMU::Point imu_point = convertMavlinkToSLAM(imu_data);
            vImuMeas.push_back(imu_point);
            timestamp = getTimestampInSeconds(imu_data.time_usec);

        } else {
            std::cout << "No IMU data available yet." << std::endl;
            continue;
        }

        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // cv::cvtColor(im, im, cv::COLOR_BGR2GRAY);
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im, timestamp, vImuMeas);

        // Clear the previous IMU measurements to load the new ones
        vImuMeas.clear();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop all threads
    SLAM.Shutdown();
    if (mavlink_thread.joinable()) {
        mavlink_thread.join();
    }
}

ORB_SLAM3::IMU::Point convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data) {
    // Convert raw IMU data to appropriate units (depends on your sensor calibration)
    // This is a simplified conversion - you'll need to adjust based on your specific sensor
    
    // Assuming accelerometer data is in mG and needs to be converted to m/s^2
    float acc_x = imu_data.xacc * 9.81f / 1000.0f;
    float acc_y = imu_data.yacc * 9.81f / 1000.0f;
    float acc_z = imu_data.zacc * 9.81f / 1000.0f;
    
    // Assuming gyroscope data is in mrad/s and needs to be converted to rad/s
    float gyro_x = imu_data.xgyro / 1000.0f;
    float gyro_y = imu_data.ygyro / 1000.0f;
    float gyro_z = imu_data.zgyro / 1000.0f;
    
    // Convert timestamp from microseconds to seconds
    double timestamp = getTimestampInSeconds(imu_data.time_usec);
    
    return ORB_SLAM3::IMU::Point(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp);
}

double getTimestampInSeconds(int64_t timestamp_us) {
    return static_cast<double>(timestamp_us) / 1e6;
}