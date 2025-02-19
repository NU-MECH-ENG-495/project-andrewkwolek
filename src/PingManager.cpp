#include "PingManager.hpp"

PingManager::PingManager(const std::string& device, int baudrate, const std::string& udp)
    : device(device), baudrate(baudrate), udp(udp) {
    myPing360 = new Ping360();

    if (!device.empty()) {
        myPing360->connect_serial(device, baudrate);
    } else if (!udp.empty()) {
        size_t colon_pos = udp.find(':');
        if (colon_pos != std::string::npos) {
            std::string host = udp.substr(0, colon_pos);
            int port = std::stoi(udp.substr(colon_pos + 1));
            myPing360->connect_udp(host, port);
        }
    }

    try {
        myPing360->initialize();
    } catch (...) {
        LOG_F(ERROR, "Failed to initialize Ping!");
        exit(1);
    }

    currentData.clear();
}

PingManager::~PingManager() {
    delete myPing360;
}

void PingManager::getPingData() {
    int step = 372;

    while (true) {
        myPing360->control_transducer(
            1,  // mode
            0,  // gain_setting
            step,  // angle
            80,  // transmit_duration
            80,  // sample_period
            750,  // transmit_frequency
            1024,  // number_of_samples
            1,  // transmit
            0   // reserved
        );

        const auto* m = myPing360->waitMessage(Ping360Id::DEVICE_DATA, 8000);
        if (m) {
            currentData = {
                {"mode", m->mode()},
                {"gain_setting", m->gain_setting()},
                {"angle", m->angle()},
                {"transmit_duration", m->transmit_duration()},
                {"sample_period", m->sample_period()},
                {"transmit_frequency", m->transmit_frequency()},
                {"number_of_samples", m->number_of_samples()},
                {"data", static_cast<int>(m->data()[0])}  // Simplified
            };

            LOG_F(INFO, "Angle: %d", currentData["angle"]);
        }

        if (step == 27) {
            step = 372;
        } else {
            step = (step + 1) % 400;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void PingManager::shutdown() {
    if (!device.empty()) {
        myPing360->connect_serial(device, baudrate);
    }

    myPing360->control_motor_off();
    LOG_F(INFO, "Ping360 shutting down");
}

std::map<std::string, int> PingManager::getData() const {
    return currentData;
}
