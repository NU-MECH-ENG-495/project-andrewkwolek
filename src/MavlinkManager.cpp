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
                            imu_buffer.add_data(imu);  // Add IMU data to the buffer
                            break;
                        }
                        case MAVLINK_MSG_ID_ATTITUDE: {
                            mavlink_attitude_t attitude;
                            mavlink_msg_attitude_decode(&message, &attitude);
                            attitude_buffer.add_data(attitude);  // Add attitude data to buffer
                            break;
                        }
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                            mavlink_global_position_int_t gps;
                            mavlink_msg_global_position_int_decode(&message, &gps);
                            gps_buffer.add_data(gps);  // Add GPS data to the buffer
                            break;
                        }
                        case MAVLINK_MSG_ID_SCALED_PRESSURE: {
                            mavlink_scaled_pressure_t pressure;
                            mavlink_msg_scaled_pressure_decode(&message, &pressure);
                            pressure_buffer.add_data(pressure);  // Add pressure data to the buffer
                            break;
                        }
                    }
                }
            }
        }
    }
}
