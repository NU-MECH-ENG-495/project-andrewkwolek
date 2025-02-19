#ifndef DATAMANAGER_HPP
#define DATAMANAGER_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include <future>
#include <ctime>
#include <chrono>
#include <opencv2/opencv.hpp>  // For OpenCV functionalities
#include "typedefs.h"

class DataManager {
public:
    DataManager();
    ~DataManager();

    void startRecording();
    void stopRecording();
    
    GPSData getGPSData();
    IMUData getIMUData();
    AttitudeData getAttitudeData();
    PressureData getPressureData();
    TimeData getTimeData();
    LocalizationData getLocalizationData();
    
    void recordData();

private:
    std::string url;
    std::unordered_map<std::string, std::vector<std::string>> recordedData;
    bool isRecording;
    std::future<void> recordingTask;
    void saveDataToCSV();
};

#endif // DATAMANAGER_H
