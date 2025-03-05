cmake_minimum_required(VERSION 3.10)
project(SLAM_Project)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Paths to ORB_SLAM3
set(ORB_SLAM3_INCLUDE_DIR "/home/kwolek/ws/ORB_SLAM3/include")
set(ORB_SLAM3_LIB_DIR "/home/kwolek/ws/ORB_SLAM3/build")
set(DBoW2_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/DBoW2")
set(g2o_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/g2o")
set(Sophus_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/Sophus")

# Find OpenCV
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${ORB_SLAM3_INCLUDE_DIR}
    ${DBoW2_DIR}/DBoW2
    ${g2o_DIR}/g2o
    ${Sophus_DIR}/Sophus
    ./include
    ./libs/httplib
)

# Set source files
set(SOURCES
    src/main.cpp
    src/CameraManager.cpp
    src/MavlinkManager.cpp
)

# Add executable
add_executable(slam_project ${SOURCES})

# Link libraries
target_link_libraries(slam_project
    ${OpenCV_LIBS}
    ${ORB_SLAM3_LIB_DIR}/libORB_SLAM3.so  # Link the ORB_SLAM3 library directly
)
