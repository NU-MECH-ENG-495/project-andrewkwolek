#ifndef MAVLINKMANAGER_H
#define MAVLINKMANAGER_H

#include <thread>
#include <future>
#include <iostream>
#include <common/mavlink.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <deque>
#include <mutex>
#include <optional>
#include <algorithm>
#include <cmath>

#include "ImuTypes.h"

// Buffer class (similar to SensorBuffer in Python)
template<typename T>
class SensorBuffer {
public:
    SensorBuffer(size_t max_size) : max_size_(max_size) {}
    
    void add_data(const T& data) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (buffer_.size() >= max_size_) {
            buffer_.pop_front();  // Maintain buffer size
        }
        buffer_.push_back(data);
    }

    bool is_empty() {
        if (buffer_.empty()) {
            return true;
        }
        else {
            return false;
        }
    }
    
    T get_latest_data() {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return buffer_.back();
    }
    
    T get_data_near_timestamp(int64_t target_time) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
    
        T closest_data;
        int64_t min_time_diff = std::numeric_limits<int64_t>::max();
    
        for (const auto& data : buffer_) {
            int64_t timestamp = data.timestamp;  // Assuming `T` has a `timestamp` field
            int64_t time_diff = std::abs(target_time - timestamp);
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                closest_data = data;
            }
        }
    
        return closest_data;
    }

private:
    std::deque<T> buffer_;
    size_t max_size_;
    std::mutex buffer_mutex_;
};

class MavlinkManager {
public:
    MavlinkManager();
    ~MavlinkManager();
    int establish_mavlink_connection();
    void receive_mavlink_data();
    ORB_SLAM3::IMU::Point MavlinkManager::convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data);
    double getTimestampInSeconds(int64_t timestamp_us);
    std::vector<ORB_SLAM3::IMU::Point> getIMUVector();
    void resetIMUVector();
    double getCurrentTimestamp();

    SensorBuffer<mavlink_raw_imu_t> imu_buffer;
    SensorBuffer<mavlink_attitude_t> attitude_buffer;
    SensorBuffer<mavlink_global_position_int_t> gps_buffer;
    SensorBuffer<mavlink_scaled_pressure_t> pressure_buffer;
    std::vector<ORB_SLAM3::IMU::Point> imu_vec;
    double current_timestamp;

    struct sockaddr* src_addr;
    socklen_t src_addr_len;
    bool src_addr_set;

private:
    int socket_fd;
    sockaddr_in gcAddr;
};

#endif

