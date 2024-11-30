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
- ROS 2 (tested on Humble/Foxy)
- OpenCV
- cv_bridge

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Vedhamshbode/mowito_assign_ws.git
   cd mowito_assign_ws
