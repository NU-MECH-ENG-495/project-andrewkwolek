#include "MavlinkManager.hpp"

// Constructor
MavlinkManager::MavlinkManager()
    : imu_buffer(10),
      attitude_buffer(10),
      gps_buffer(10),
      pressure_buffer(10) {
    if (establish_mavlink_connection() < 0) {
        exit(1);
    }
    std::cout << "Mavlink connection established." << std::endl;
}

// Destructor
MavlinkManager::~MavlinkManager() {
    close(socket_fd);
}

// Establish UDP connection
int MavlinkManager::establish_mavlink_connection() {
    socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        printf("socket error: %s\n", strerror(errno));
        return -1;
    }

    // Bind to port
    struct sockaddr_in addr = {};
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
    addr.sin_port = htons(14555); // default port on the ground

    if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
        printf("bind error: %s\n", strerror(errno));
        return -2;
    }

    // We set a timeout at 100ms to prevent being stuck in recvfrom for too
    // long and missing our chance to send some stuff.
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        printf("setsockopt error: %s\n", strerror(errno));
        return -3;
    }

    *src_addr = {};
    src_addr_len = sizeof(*src_addr);
    src_addr_set = false;

    return 0;
}

// Function to receive MAVLink data and process it
void MavlinkManager::receive_mavlink_data() {
    std::cout << "Mavlink thread started, waiting for data..." << std::endl;
    char buffer[2048];  // Buffer to receive raw MAVLink data

    while (true) {
        ssize_t ret = recvfrom(socket_fd, buffer, sizeof(buffer), 0, src_addr, &src_addr_len);  // Receive data from socket
        if (ret < 0) {
            printf("recvfrom error: %s\n", strerror(errno));
        } else if (ret == 0) {
            // peer has done an orderly shutdown
            return;
        }
        else {
            src_addr_set = true;
            mavlink_message_t message;
            mavlink_status_t status;
            // Process each byte of received data to parse MAVLink messages
            for (ssize_t i = 0; i < ret; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {
                    // A valid MAVLink message has been parsed, now handle it
                    switch (message.msgid) {
                        case MAVLINK_MSG_ID_RAW_IMU: {
                            mavlink_raw_imu_t imu;
                            mavlink_msg_raw_imu_decode(&message, &imu);

                            std::lock_guard<std::mutex> lock(data_mutex);
                            current_timestamp = getTimestampInSeconds(imu.time_usec);
                            imu_vec.push_back(convertMavlinkToSLAM(imu));
                            break;
                        }
                    }
                }
            }
        }
    }
}

ORB_SLAM3::IMU::Point MavlinkManager::convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data) {
    // Convert raw IMU data to appropriate units (depends on your sensor calibration)
    // This is a simplified conversion - you'll need to adjust based on your specific sensor
    
    // Assuming accelerometer data is in mG and needs to be converted to m/s^2
    float acc_x = imu_data.xacc / 100.0;
    float acc_y = imu_data.yacc / 100.0;
    float acc_z = imu_data.zacc / 100.0;
    
    // Assuming gyroscope data is in mrad/s and needs to be converted to rad/s
    float gyro_x = imu_data.xgyro / 1000.0;
    float gyro_y = imu_data.ygyro / 1000.0;
    float gyro_z = imu_data.zgyro / 1000.0;
    
    // Convert timestamp from microseconds to seconds
    double timestamp = getTimestampInSeconds(imu_data.time_usec);
    
    return ORB_SLAM3::IMU::Point(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp);
}

double MavlinkManager::getTimestampInSeconds(int64_t timestamp_us) {
    return static_cast<double>(timestamp_us) / 1e6;
}

std::vector<ORB_SLAM3::IMU::Point> MavlinkManager::getIMUVector() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return imu_vec;
}

void MavlinkManager::resetIMUVector() {
    std::lock_guard<std::mutex> lock(data_mutex);
    imu_vec.clear();
}

double MavlinkManager::getCurrentTimestamp() {
    std::lock_guard<std::mutex> lock(data_mutex);
    return current_timestamp;
}