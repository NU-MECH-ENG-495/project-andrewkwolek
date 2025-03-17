#include "MavlinkManager.hpp"
#include <chrono>
#include <errno.h>

// Constructor
MavlinkManager::MavlinkManager()
    : current_timestamp(0.0),
      src_addr_set(false),
      socket_fd(-1),
      keep_running(true) {
    
    // Try to establish connection, but don't fail if it doesn't work
    if (establish_mavlink_connection() < 0) {
        std::cerr << "Warning: MAVLink connection failed, will operate without IMU data" << std::endl;
    } else {
        std::cout << "MAVLink connection established successfully" << std::endl;
    }
}

// Destructor
MavlinkManager::~MavlinkManager() {
    // Signal the receiving thread to stop
    stopReceiving();
    
    // Close socket if it was opened
    if (socket_fd >= 0) {
        close(socket_fd);
        socket_fd = -1;
    }
    
    std::cout << "MavlinkManager cleaned up" << std::endl;
}

void MavlinkManager::stopReceiving() {
    keep_running.store(false);
}

// Establish UDP connection
int MavlinkManager::establish_mavlink_connection() {
    // Initialize default values in case of error
    socket_fd = -1;
    src_addr_set = false;
    
    try {
        // Create UDP socket
        socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            std::cerr << "Socket error: " << strerror(errno) << std::endl;
            return -1;
        }

        // Bind to port
        struct sockaddr_in addr = {};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY); // listen on all network interfaces
        addr.sin_port = htons(14555); // default port on the ground

        if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
            std::cerr << "Bind error: " << strerror(errno) << std::endl;
            close(socket_fd);
            socket_fd = -1;
            return -2;
        }

        // Set socket timeout to 100ms
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            std::cerr << "Setsockopt error: " << strerror(errno) << std::endl;
            close(socket_fd);
            socket_fd = -1;
            return -3;
        }

        // Initialize src_addr
        memset(&src_addr, 0, sizeof(src_addr));
        src_addr_len = sizeof(src_addr);
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception in establish_mavlink_connection: " << e.what() << std::endl;
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
        }
        return -4;
    }
}

// Function to receive MAVLink data and process it
void MavlinkManager::receive_mavlink_data() {
    std::cout << "MAVLink thread started, waiting for data..." << std::endl;
    
    // Check if socket is valid
    if (socket_fd < 0) {
        std::cerr << "Cannot start MAVLink receiver: invalid socket" << std::endl;
        return;
    }
    
    // Reset the running flag
    keep_running.store(true);
    
    // Buffer for incoming data
    char buffer[2048];
    
    // Main reception loop
    while (keep_running.load()) {
        try {
            // Receive data with timeout
            ssize_t ret = recvfrom(socket_fd, buffer, sizeof(buffer), 0, 
                                  (struct sockaddr*)&src_addr, &src_addr_len);
            
            if (ret < 0) {
                // Handle timeout and other errors
                if (errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "recvfrom error: " << strerror(errno) << std::endl;
                }
                // Just a timeout, continue
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            } else if (ret == 0) {
                // Peer has done an orderly shutdown
                std::cout << "MAVLink peer has disconnected" << std::endl;
                break;
            } else {
                // Data received successfully
                src_addr_set = true;
                
                // Parse MAVLink messages
                mavlink_message_t message;
                mavlink_status_t status;
                
                for (ssize_t i = 0; i < ret; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {
                        // We're only interested in RAW_IMU messages for now
                        if (message.msgid == MAVLINK_MSG_ID_RAW_IMU) {
                            mavlink_raw_imu_t imu;
                            mavlink_msg_raw_imu_decode(&message, &imu);
                            
                            // Update the timestamp and add the IMU data to our vector
                            std::lock_guard<std::mutex> lock(data_mutex);
                            current_timestamp = getTimestampInSeconds(imu.time_usec);
                            imu_vec.push_back(convertMavlinkToSLAM(imu));
                        }
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception in receive_mavlink_data: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    std::cout << "MAVLink thread exiting" << std::endl;
}

ORB_SLAM3::IMU::Point MavlinkManager::convertMavlinkToSLAM(const mavlink_raw_imu_t& imu_data) {
    // Convert raw IMU data to appropriate units
    // Assuming accelerometer data is in mG and needs to be converted to m/s^2
    float acc_x = imu_data.xacc / 100.0f;
    float acc_y = imu_data.yacc / 100.0f;
    float acc_z = imu_data.zacc / 100.0f;
    
    // Assuming gyroscope data is in mrad/s and needs to be converted to rad/s
    float gyro_x = imu_data.xgyro / 1000.0f;
    float gyro_y = imu_data.ygyro / 1000.0f;
    float gyro_z = imu_data.zgyro / 1000.0f;
    
    // Convert timestamp from microseconds to seconds
    double timestamp = getTimestampInSeconds(imu_data.time_usec);
    
    return ORB_SLAM3::IMU::Point(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, timestamp);
}

double MavlinkManager::getTimestampInSeconds(int64_t timestamp_us) {
    return static_cast<double>(timestamp_us) / 1.0e6;
}

std::vector<ORB_SLAM3::IMU::Point> MavlinkManager::getIMUVector() {
    std::lock_guard<std::mutex> lock(data_mutex);
    // Return a copy of the vector
    return imu_vec;
}

void MavlinkManager::resetIMUVector() {
    std::lock_guard<std::mutex> lock(data_mutex);
    // Clear all accumulated IMU data
    imu_vec.clear();
}

double MavlinkManager::getCurrentTimestamp() {
    std::lock_guard<std::mutex> lock(data_mutex);
    
    // If there's no timestamp from IMU, use system time
    if (current_timestamp <= 0.0) {
        current_timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
    return current_timestamp;
}