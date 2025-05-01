# Camera
Updated: Jan 21, 2024
---
**Current state:** Currently have a working raw camera node, need to fix the compressed image portion, and the launch file.

### How does the camera package work?
This package has the following dependencies:
- rclpy
- numpy
- cv2
- cvbridge
- sensor_msgs.msg

This package basically creates an `/image` topic which can be subscribed to using RVIZ2.

### ROS Parameters
These parameters can be passed at runtime:

**camera_node** 
camera_port: integer value specifying virtual device to be used, default 0
display_output: bool value, decides whether to publish annotated image to image_annotated topic

**camera_node_raw**
camera_port: ^^
fps: integer value, framerate