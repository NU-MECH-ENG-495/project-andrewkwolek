#ifndef PROCESSOR_HPP
#define PROCESSOR_HPP

#include <deque>
#include <mutex>
#include <memory>
#include <mavlink.h>
#include "PingManager.hpp"
#include "DataManager.hpp"
#include "typedefs.hpp"

template <typename T>
class SensorBuffer {
public:
    SensorBuffer(size_t max_size, MavlinkMessage type);
    void add_data(const T& data);
    T get_latest_data();
    T get_data_near_timestamp(uint64_t target_time);

private:
    std::deque<T> buffer;
    std::mutex buffer_mutex;
    size_t max_size;
    MavlinkMessage type;
};

class Processor {
public:
    Processor(int baudrate, const std::string& device = "", const std::string& udp = "");

    void write_gps_buffer_rest();
    void write_imu_buffer_rest();
    void write_attitude_buffer_rest();
    void write_pressure_buffer_rest();
    void receive_mavlink_data();
    void receive_sonar_data();

private:
    void write_sensor_buffer(uint32_t msg_type, mavlink_message_t& msg);

    std::shared_ptr<SensorBuffer<MavlinkMessage>> imu_buffer;
    std::shared_ptr<SensorBuffer<MavlinkMessage>> attitude_buffer;
    std::shared_ptr<SensorBuffer<MavlinkMessage>> gps_buffer;
    std::shared_ptr<SensorBuffer<MavlinkMessage>> pressure_buffer;
    std::shared_ptr<SensorBuffer<MavlinkMessage>> servo_buffer;

    DataManager data_manager;
    PingManager ping_manager;
    mavutil::MavlinkConnection mav;
};

#endif  // PROCESSOR_H
