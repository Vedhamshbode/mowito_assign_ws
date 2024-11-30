# Image Converter Node for ROS 2

This ROS 2 package implements an **Image Converter Node** that subscribes to an input image topic, converts the image between color and grayscale based on a service request, and publishes the processed image to an output topic.

---

## Features

- Subscribes to an input image topic.
- Provides a service to toggle between colored and grayscale images.
- Publishes the processed image to an output topic.
- Configurable topics via launch files.

---

## Prerequisites

Ensure you have the following installed:
- ROS 2 (tested on Humble)
- OpenCV
- cv_bridge

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Vedhamshbode/mowito_assign_ws.git
   cd mowito_assign_ws
   ```
2. Build and source the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```
3. launch the image_converter launch file with parameters:
   ```bash
   ros2 launch image_conversion image_conversion.launch.py camera_topic:="/desired_name" input_topic:="/desired_name/image_raw" output_topic:="/desired_name"
   ```
4. visualize the image in Rviz:
   ```bash
   rviz2
   ```
5. Service call:
```bash
   ros2 service call /Color_BnW_Switch example_interfaces/srv/SetBool "data: false"
   ros2 service call /Color_BnW_Switch example_interfaces/srv/SetBool "data: true"
```

Here, the camera_topic can be varied, but the input_topic should be the same as the camera_topic but with "/image_raw" added to the camera_topic. output_topic can be varied by the user.
