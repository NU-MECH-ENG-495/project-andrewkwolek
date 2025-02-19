cmake_minimum_required(VERSION 3.10)
project(SLAM_Project)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS} ./include ./libs/httplib)

# Set source files
set(SOURCES
    src/main.cpp
    src/video_capture.cpp
    src/Processor.cpp
    src/PingManager.cpp
    src/DataManager.cpp
)

# Add executable
add_executable(slam_project ${SOURCES})

# Link libraries
target_link_libraries(slam_project ${OpenCV_LIBS})
