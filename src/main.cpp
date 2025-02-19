#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <fstream>
#include <loguru.hpp>
#include <future>
#include <thread>
#include "Video.hpp"
#include "MonoVideoOdometry.hpp"
#include "Processor.hpp"
#include <httplib.h> // For the web server (cpp-httplib)

// Constants
const std::string SERVICE_NAME = "slam";
const std::string VIDEO_STREAM = "udp://192.168.2.1:5600";

// Declare Processor and Video objects
Processor data_processor(nullptr, 115200, "192.168.2.2:9092");

// Function to simulate asynchronous visual odometry
void v0() {
    double focal = 1188.0;
    cv::Point2d pp(960, 540);
    cv::TermCriteria lk_params(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    // Initialize Visual Odometry
    MonoVideoOdometry vo(focal, pp, lk_params);

    // Initialize trajectory visualization with white background
    cv::Mat traj = cv::Mat::ones(600, 800, CV_8UC3) * 255;

    // Scale factor for visualization
    double scale_factor = 5.0;

    Video video(VIDEO_STREAM);
    LOG_F(INFO, "Initializing stream...");
    while (!video.frameAvailable()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    LOG_F(INFO, "Stream initialized!");

    int save_interval = 100;  // Save trajectory image every N frames
    int frame_count = 0;

    while (true) {
        if (video.frameAvailable()) {
            cv::Mat frame = video.getFrame();

            // Process frame
            vo.processFrame(frame);
            cv::Point2d current_pos = vo.getMonoCoordinates();

            // Scale the coordinates and negate Y (right is positive)
            int draw_y = static_cast<int>(round(current_pos.y * scale_factor));
            int draw_x = static_cast<int>(round(current_pos.x * scale_factor));

            // Draw black trail
            cv::circle(traj, cv::Point(draw_y + 400, draw_x + 300), 1, cv::Scalar(0, 0, 0), 2);

            // Draw coordinate axes
            cv::Point origin(400, 300);
            int axes_length = 50;
            cv::Scalar red(0, 0, 255), green(0, 255, 0);

            // X-axis vertical (forward/back) in red
            cv::arrowedLine(traj, origin, cv::Point(origin.x, origin.y - axes_length), red, 2);
            // Y-axis horizontal (right is positive) in green
            cv::arrowedLine(traj, origin, cv::Point(origin.x + axes_length, origin.y), green, 2);

            // Add labels
            cv::putText(traj, "X (forward)", cv::Point(origin.x - 20, origin.y - axes_length - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
            cv::putText(traj, "Y (right)", cv::Point(origin.x + axes_length + 10, origin.y + 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

            // Show current position values (unscaled)
            std::string pos_text = "X: " + std::to_string(current_pos.x) + " Y: " + std::to_string(-current_pos.y);
            cv::putText(traj, pos_text, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);

            // Print position to console
            LOG_F(INFO, "Position - X: %.2f Y: %.2f", current_pos.x, -current_pos.y);

            // Save trajectory image periodically
            frame_count++;
            if (frame_count % save_interval == 0) {
                cv::imwrite("trajectory_" + std::to_string(frame_count) + ".png", traj);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Equivalent to asyncio.sleep(0.01)
    }
}

// Basic HTTP server using httplib to handle requests
void runServer() {
    httplib::Server svr;

    // GPS Data endpoint
    svr.Get("/gps", [](const httplib::Request&, httplib::Response& res) {
        std::string gps_data = data_processor.data_manager.getGpsData();
        res.set_content(gps_data, "application/json");
    });

    // IMU Data endpoint
    svr.Get("/imu", [](const httplib::Request&, httplib::Response& res) {
        std::string imu_data = data_processor.data_manager.getImuData();
        res.set_content(imu_data, "application/json");
    });

    // Start the server
    svr.listen("0.0.0.0", 9050);
}

// Main function
int main(int argc, char* argv[]) {
    loguru::init(argc, argv);
    LOG_F(INFO, "Starting SLAM system...");

    // Ensure root privileges
    if (geteuid() != 0) {
        LOG_F(ERROR, "You need root privileges to run this script.");
        return 1;
    }

    // Start visual odometry and web server asynchronously
    std::future<void> vo_future = std::async(std::launch::async, v0);
    std::future<void> server_future = std::async(std::launch::async, runServer);

    vo_future.get();
    server_future.get();

    return 0;
}
