#include "Processor.hpp"
#include <iostream>
#include <limits>
#include <thread>
#include <chrono>
#include <future>
#include <mavlink.h>
#include "typedefs.hpp"

template <typename T>
SensorBuffer<T>::SensorBuffer(size_t max_size, MavlinkMessage type) : max_size(max_size), type(type) {}

template <typename T>
void SensorBuffer<T>::add_data(const T& data) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    if (buffer.size() >= max_size) {
        buffer.pop_front();
    }
    buffer.push_back(data);
}

template <typename T>
T SensorBuffer<T>::get_latest_data() {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    return buffer.empty() ? T() : buffer.back();
}

template <typename T>
T SensorBuffer<T>::get_data_near_timestamp(uint64_t target_time) {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    T closest_data;
    uint64_t min_time_diff = std::numeric_limits<uint64_t>::max();

    for (const auto& data : buffer) {
        uint64_t timestamp = data["timestamp"];
        uint64_t time_diff = std::abs(static_cast<int64_t>(target_time - timestamp));
        if (time_diff < min_time_diff) {
            min_time_diff = time_diff;
            closest_data = data;
        }
    }

    return closest_data;
}

Processor::Processor(int baudrate, const std::string& device, const std::string& udp)
    : data_manager(), ping_manager(device, baudrate, udp) {

    // Setup MAVLink connection
    mav = mavutil::mavlink_connection("udpin:0.0.0.0:14550");
    std::cout << "Mavlink connection established." << std::endl;

    // Initialize sensor buffers
    imu_buffer = std::make_shared<SensorBuffer<MavlinkMessage>>(10, MavlinkMessage::RAW_IMU);
    attitude_buffer = std::make_shared<SensorBuffer<MavlinkMessage>>(10, MavlinkMessage::ATTITUDE);
    gps_buffer = std::make_shared<SensorBuffer<MavlinkMessage>>(10, MavlinkMessage::GLOBAL_POSITION_INT);
    pressure_buffer = std::make_shared<SensorBuffer<MavlinkMessage>>(10, MavlinkMessage::SCALED_PRESSURE);
    servo_buffer = std::make_shared<SensorBuffer<MavlinkMessage>>(10, MavlinkMessage::SERVO_OUTPUT_RAW);
}

void Processor::write_gps_buffer_rest() {
    std::async(std::launch::async, [this]() {
        while (true) {
            auto data = data_manager.get_gps_data();
            gps_buffer->add_data(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::write_imu_buffer_rest() {
    std::async(std::launch::async, [this]() {
        while (true) {
            auto data = data_manager.get_imu_data();
            imu_buffer->add_data(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::write_attitude_buffer_rest() {
    std::async(std::launch::async, [this]() {
        while (true) {
            auto data = data_manager.get_attitude_data();
            attitude_buffer->add_data(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::write_pressure_buffer_rest() {
    std::async(std::launch::async, [this]() {
        while (true) {
            auto data = data_manager.get_pressure_data();
            pressure_buffer->add_data(data);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::receive_mavlink_data() {
    std::async(std::launch::async, [this]() {
        while (true) {
            mavlink_message_t msg;
            if (mav.recv_match(&msg)) {
                write_sensor_buffer(msg.msgid, msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::receive_sonar_data() {
    std::async(std::launch::async, [this]() {
        while (true) {
            ping_manager.gather_ping_data();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void Processor::write_sensor_buffer(uint32_t msg_type, mavlink_message_t& msg) {
    switch (msg_type) {
        case MAVLINK_MSG_ID_RAW_IMU:
            imu_buffer->add_data(msg);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            attitude_buffer->add_data(msg);
            break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            gps_buffer->add_data(msg);
            break;
        case MAVLINK_MSG_ID_SCALED_PRESSURE:
            pressure_buffer->add_data(msg);
            break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            servo_buffer->add_data(msg);
            break;
    }
}
