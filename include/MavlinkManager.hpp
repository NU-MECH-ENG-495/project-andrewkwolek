#ifndef MAVLINKMANAGER_H
#define MAVLINKMANAGER_H

#include <thread>
#include <iostream>
#include <common/mavlink.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <atomic>
#include <cstring>

#include "ImuTypes.h"

class MavlinkManager {
public:
    MavlinkManager();
    ~MavlinkManager();
    
    // Main functions
    int establish_mavlink_connection();
    void receive_mavlink_data();
    void stopReceiving();
    
    // IMU data handling
    double getCurrentTimestamp();
    std::vector<ORB_SLAM3::IMU::Point> getIMUVector();
    void resetIMUVector();

private:
    // Socket handling
    int socket_fd;
    struct sockaddr_in src_addr;
    socklen_t src_addr_len;
    bool src_addr_set;
    
    // IMU data storage
    std::vector<ORB_SLAM3::IMU::Point> imu_vec;
    double current_timestamp;
    
    // Thread control
    std::atomic<bool> keep_running;
    
    // Mutex for thread safety
    std::mutex data_mutex;
    
    // Helper function to convert MAVLink IMU to SLAM IMU point
    ORB_SLAM3::IMU::Point convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data);
    double getTimestampInSeconds(int64_t timestamp_us);
};

#endif // MAVLINKMANAGER_H