#ifndef PING_MANAGER_HPP
#define PING_MANAGER_HPP

#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <atomic>
#include <ping-device-ping360.h>
#include <ping-message-all.h>
#include <link/desktop/abstract-link.h>
#include <loguru.hpp>

class PingManager {
public:
    PingManager(const std::string& device, int baudrate, const std::string& udp);
    ~PingManager();

    void getPingData();
    void shutdown();
    std::map<std::string, int> getData() const;

private:
    Ping360* myPing360;
    std::string device;
    int baudrate;
    std::string udp;
    std::map<std::string, int> currentData;
};

#endif // PING_MANAGER_H
