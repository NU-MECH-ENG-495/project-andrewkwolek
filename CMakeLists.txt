cmake_minimum_required(VERSION 3.10)
project(SLAM_Project)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Paths to ORB_SLAM3
set(ORB_SLAM3_DIR "/home/kwolek/ws/ORB_SLAM3")
set(ORB_SLAM3_INCLUDE_DIR "/home/kwolek/ws/ORB_SLAM3/include")
set(ORB_SLAM3_LIB_DIR "/home/kwolek/ws/ORB_SLAM3/lib")
set(DBoW2_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/DBoW2")
set(g2o_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/g2o")
set(Sophus_DIR "/home/kwolek/ws/ORB_SLAM3/Thirdparty/Sophus")
set(Eigen3_DIR "/usr/share/eigen3/cmake")
set(Pangolin_DIR "/home/kwolek/ws/Pangolin/build")


# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_INCLUDE_DIR}
    ${ORB_SLAM3_INCLUDE_DIR}/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${Pangolin_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
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
    ${Pangolin_LIBRARIES}
    ${GSTREAMER_LIBRARIES}
    ${GST_APP_LIBRARIES}
)
