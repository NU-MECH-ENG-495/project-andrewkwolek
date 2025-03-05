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

// Buffer class (similar to SensorBuffer in Python)
template<typename T>
class SensorBuffer {
public:
    SensorBuffer(size_t max_size);
    void add_data(const T& data);
    T& get_latest_data();
    T& get_data_near_timestamp(int64_t target_time);

private:
    std::deque<T> buffer_;
    size_t max_size_;
    std::mutex buffer_mutex_;
};

class MavlinkManager {
public:
    MavlinkManager();
    ~MavlinkManager();
    void establish_mavlink_connection();
    void receive_mavlink_data();

private:
    SensorBuffer<mavlink_raw_imu_t> imu_buffer;
    SensorBuffer<mavlink_attitude_t> attitude_buffer;
    SensorBuffer<mavlink_global_position_int_t> gps_buffer;
    SensorBuffer<mavlink_scaled_pressure_t> pressure_buffer;

    int sock;
    sockaddr_in gcAddr;
};

