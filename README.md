# Monocular Inertial SLAM using ORB-SLAM3

Insert video

## Overview

This project demonstrates the integration of ORB-SLAM3, a state-of-the-art open-source SLAM system, with the BlueROV2 by Blue Robotics. The result is a real-time, underwater SLAM application.
By utilizing multithreading, object-oriented programming, and seamless package integration, I successfully adapted ORB-SLAM3 to function with external hardware in an aquatic environment.

## Camera Calibration

Monocular SLAM requires precise camera calibration due to its inherent scale ambiguity. Unlike stereo cameras, which estimate depth, monocular cameras cannot directly measure scale, meaning small movements can be misinterpreted as large transformations without accurate calibration.
To address this, I used OpenCV and a checkerboard pattern to thoroughly calibrate the BlueROV2's onboard camera, ensuring reliable performance in underwater SLAM.

## Data Retrieval

To minimize latency when retrieving camera and IMU data, websocket connections over UDP are established to efficiently collect and process data from both sensors. The BlueROV2 uses Mavlink as its telemetry protocol, transmitting a wide range of messages about the vehicle's state.
However, to ensure optimal performance, only the raw IMU data is filtered and processed, while other telemetry messages are ignored.

## Sensor Integration

Monocular-Inertial SLAM leverages an IMU to scale feature displacements detected by the ORB feature detector. Since the IMU operates at a higher frequency than the camera, it continuously records multiple measurements while awaiting the next camera frame.
To manage this, the `CameraManager` and `MavlinkManager` classes run in separate threads, ensuring thread-safe data handling through the use of mutexes and RAII lock guards.

In the main loop, IMU measurements are collected into a vector, which is then accessed when a new frame is ready. The frame and the corresponding IMU data are time-synchronized and passed to ORB-SLAM3 for processing, enabling accurate SLAM execution.

## Testing

To ensure the functionality of this project, I conducted two types of testing. The first involved open-air operation, where I manually carried the ROV through a feature-rich environment to verify that the system could successfully identify and map features.
This allowed me to confirm that the code was functioning correctly before transitioning to underwater testing.

As anticipated, ORB-SLAM3 encountered significant challenges underwater due to the lack of distinguishable features, light attenuation, and the added latency from the ground control application needed to operate the ROV.
ORB-SLAM3 only displays the video feed if it detects enough features, and given that the Northwestern pool had very few identifiable features, the video feed was never displayed.

## Future Considerations

In the future, I plan to implement a feature enhancement algorithm to address the challenges of underwater vision. This could help restore vibrant colors like red, which tend to be washed out underwater, making them appear as vivid as they would in air.
I also aim to test the system in a natural aquatic environment, where the features are less uniform and more easily identifiable, providing a more suitable testing ground than a controlled pool environment.
