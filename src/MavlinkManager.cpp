#include "MavlinkManager.hpp"

template<typename T>
SensorBuffer<T>::SensorBuffer(size_t max_size) : max_size_(max_size) {}


template<typename T>
void SensorBuffer<T>::add_data(const T& data) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_.size() >= max_size_) {
        buffer_.pop_front();  // Maintain buffer size
    }
    buffer_.push_back(data);
}

template<typename T>
T& SensorBuffer<T>::get_latest_data() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return buffer_.back();
}

template<typename T>
T& SensorBuffer<T>::get_data_near_timestamp(int64_t target_time) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_.empty()) return std::nullopt;

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

// Constructor
MavlinkManager::MavlinkManager()
    : imu_buffer(10),
      attitude_buffer(10),
      gps_buffer(10),
      pressure_buffer(10) {
    establish_mavlink_connection();
    std::cout << "Mavlink connection established." << std::endl;
}

// Destructor
MavlinkManager::~MavlinkManager() {
    close(sock);
}

// Establish UDP connection
void MavlinkManager::establish_mavlink_connection() {
    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        perror("Error creating socket");
        exit(EXIT_FAILURE);
    }

    memset(&gcAddr, 0, sizeof(gcAddr));
    gcAddr.sin_family = AF_INET;
    gcAddr.sin_addr.s_addr = INADDR_ANY;
    gcAddr.sin_port = htons(14555);

    if (bind(sock, (struct sockaddr*)&gcAddr, sizeof(gcAddr)) < 0) {
        perror("Error binding socket");
        close(sock);
        exit(EXIT_FAILURE);
    }
}

// Function to receive MAVLink data and process it
void MavlinkManager::receive_mavlink_data() {
    uint8_t buf[1024];
    mavlink_message_t msg;
    mavlink_status_t status;
    while (true) {
        ssize_t recsize = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (recsize > 0) {
            for (int i = 0; i < recsize; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_RAW_IMU: {
                            mavlink_raw_imu_t imu;
                            mavlink_msg_raw_imu_decode(&msg, &imu);
                            imu_buffer.add_data(imu);
                            break;
                        }
                        case MAVLINK_MSG_ID_ATTITUDE: {
                            mavlink_attitude_t attitude;
                            mavlink_msg_attitude_decode(&msg, &attitude);
                            attitude_buffer.add_data(attitude);
                            break;
                        }
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                            mavlink_global_position_int_t gps;
                            mavlink_msg_global_position_int_decode(&msg, &gps);
                            gps_buffer.add_data(gps);
                            break;
                        }
                        case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                            mavlink_scaled_pressure_t pressure;
                            mavlink_msg_scaled_pressure_decode(&msg, &pressure);
                            pressure_buffer.add_data(pressure);
                            break;
                        }
                    }
                }
            }
        }
    }
}
