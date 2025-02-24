#!/bin/bash

ros2 launch motor_control motor_control.launch.xml
ros2 launch dvl_data dvl_launch.xml
ros2 launch chimera_camera camera_launch.xml