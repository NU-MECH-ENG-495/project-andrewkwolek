#include "DataManager.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp>  // For handling JSON responses
#include "settings.hpp"
#include "typedefs.hpp"
#include <opencv2/opencv.hpp> // For using OpenCV functionalities

DataManager::DataManager() {
    url = "http://" + std::string(VEHICLE_IP) + ":6040/v1/mavlink/vehicles/1/components/1/messages";
    isRecording = false;
}

DataManager::~DataManager() {
    if (isRecording) {
        stopRecording();
    }
}

void DataManager::startRecording() {
    if (isRecording) {
        std::cerr << "Already recording!" << std::endl;
        return;
    }
    recordedData.clear();
    isRecording = true;
    std::cout << "Recording started." << std::endl;
}

void DataManager::stopRecording() {
    if (!isRecording) {
        std::cerr << "No recording in progress!" << std::endl;
        return;
    }
    isRecording = false;
    std::cout << "Recording stopped." << std::endl;

    saveDataToCSV();
}

void DataManager::saveDataToCSV() {
    std::filesystem::create_directories(DATA_FILEPATH);
    
    std::time_t timestamp = std::time(nullptr);
    char buffer[100];
    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&timestamp));
    std::string filename = DATA_FILEPATH + "/slam_data_" + std::string(buffer) + ".csv";
    
    std::ofstream file(filename);
    if (file.is_open()) {
        std::cout << "Writing to CSV." << std::endl;
        // Write header
        for (const auto& [key, _] : recordedData) {
            file << key << " ";
        }
        file << std::endl;

        // Write rows of data
        size_t numRows = recordedData.begin()->second.size();
        for (size_t i = 0; i < numRows; ++i) {
            for (const auto& [key, values] : recordedData) {
                file << values[i] << " ";
            }
            file << std::endl;
        }
        
        std::cout << "Data saved to " << filename << std::endl;
    } else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }
}

// Mock methods that simulate fetching data from an external service
GPSData DataManager::getGPSData() {
    // Placeholder implementation
    GPSData gps;
    gps.timestamp = std::chrono::system_clock::now();
    gps.latitude = 52.5200;
    gps.longitude = 13.4050;
    gps.altitude = 34.0;
    return gps;
}

IMUData DataManager::getIMUData() {
    // Placeholder implementation
    IMUData imu;
    imu.timestamp = std::chrono::system_clock::now();
    imu.x_acc = 0.02;
    imu.y_acc = -0.01;
    imu.z_acc = 9.81;
    imu.x_gyro = 0.0;
    imu.y_gyro = 0.0;
    imu.z_gyro = 0.0;
    return imu;
}

AttitudeData DataManager::getAttitudeData() {
    // Placeholder implementation
    AttitudeData att;
    att.timestamp = std::chrono::system_clock::now();
    att.roll = 0.01;
    att.pitch = 0.02;
    att.yaw = 0.03;
    return att;
}

PressureData DataManager::getPressureData() {
    // Placeholder implementation
    PressureData press;
    press.timestamp = std::chrono::system_clock::now();
    press.press_abs = 1013.25;
    press.press_diff = 0.0;
    return press;
}

TimeData DataManager::getTimeData() {
    // Placeholder implementation
    TimeData timeData;
    timeData.timestamp = std::chrono::system_clock::now();
    return timeData;
}

LocalizationData DataManager::getLocalizationData() {
    GPSData gps = getGPSData();
    IMUData imu = getIMUData();
    AttitudeData att = getAttitudeData();
    PressureData press = getPressureData();

    LocalizationData locData;
    locData.timestamp = std::chrono::system_clock::now();
    locData.gps_data = gps;
    locData.imu_data = imu;
    locData.attitude_data = att;
    locData.pressure_data = press;

    return locData;
}

void DataManager::recordData() {
    std::cout << "Recording data is running!" << std::endl;
    while (isRecording) {
        GPSData gps_data = getGPSData();
        IMUData imu_data = getIMUData();
        AttitudeData att_data = getAttitudeData();
        PressureData press_data = getPressureData();

        // Store data
        recordedData["timestamp"].push_back(std::to_string(std::chrono::system_clock::to_time_t(gps_data.timestamp)));
        recordedData["latitude"].push_back(std::to_string(gps_data.latitude));
        recordedData["longitude"].push_back(std::to_string(gps_data.longitude));
        recordedData["altitude"].push_back(std::to_string(gps_data.altitude));
        // Store more fields...

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Coroutine completed." << std::endl;
}
