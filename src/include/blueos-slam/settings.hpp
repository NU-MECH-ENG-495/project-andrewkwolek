#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <string>

// Define constants for file paths, IPs, and device locations
const std::string DATA_FILEPATH = "/app/slam_data";
const std::string DOCKER_HOST = "host.docker.internal";
const std::string VEHICLE_IP = "192.168.2.2";
const std::string PING_BRIDGE = "UDP 9092";
const std::string PING_DEVICE = "/dev/ttyUSB0";
const std::string VIDEO_STREAM = "udp://192.168.2.1:5600";
const std::string UDP_PORT = "192.168.2.2:9092";
const std::string VIDEO_PATH = "/dev/video2";

#endif // CONFIG_H
