# Camera
Updated: May 2, 2025
---
**Current state:** raw camera node which publishes image data from a specific camera port, camera node that provides the DetectObjects service for any currently running camera

### How does the camera package work?
This package has the following dependencies:
- rclpy
- numpy
- cv2
- cvbridge
- sensor_msgs.msg

### Overview

**camera_node_raw**

publishes images to a topic corresponding to a specific camera

**camera_node** 

pulls image data from all running camera_node_raw, and provides DetectObjects service for any one of them


### ROS Parameters (IMPORTANT!!)
These parameters can be passed at runtime:

**camera_node_raw**

camera_port: integer value, /dev/video device to read feed from
fps: integer value, framerate
camera_id: string, one of a set of camera id's. Each id is mapped to it's own topic. In DetectObjects.srv, this is passed in the request.camera_id field. Makes sure a consistent API is exposed to client nodes requesting the DetectObjects service for a specific camera port. Possible values (subject to change): front, rear, top, bottom

**camera_node** 

display_output: bool value, decides whether to publish annotated image to camera specific image_annotated topic

### Troubleshooting

- Make sure the correct custom_interfaces are sourced; the DetectObjects service as a request field that needs to be filled!

- camera_node only publishes when the service is called and the proper parameters are passed, resulting in the fps of the image_annotated topics being limited by the frequency of requests
