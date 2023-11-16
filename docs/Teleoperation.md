# Teleoperation
Updated: Nov 13, 2023
---
**Current state:** We are at a point where when running a joystick node, we subscribe to that joystick node and convert the values to PWM
values that can be sent to a singular motor. We need to:
- Map forward/backward motion to our forward/backward motors (pending motor configuration once the sub is built).
- Map side-to-side motion to our side-to-side motors.
- Map forward-backward pwm commands to forward-backward motors depending on the twist command
- Map depth motion to depth motors
- Wrap it all up in a bow so that you can roslaunch the teleop operations
  - Launch ros2 teleop node `ros2 launch teleop_twist_joy teleop-launch.py`
  - Run joyListener.py

### How does the teleoperation package work?
This package has the following dependencies:
- pyserial
- maestro (how do we get maestro?)
- pytime
- rclpy
- sensor_msgs.msg
- ROS2 [teleop_twist_joy_package](https://index.ros.org/r/teleop_twist_joy/)

This package runs on three general functions:
- Subscribe to the `/joy` node (ROS2 teleop_twist_joy package) to take data from the joystick (floating point value for each axis from -1 to 1)
- Convert the floating point data to a PWM value (neutral position is 1490, see Maestro for more details)
- Based on the axis sending the data, send the calculated PWM value to either the forward-back, left-right, or up-down motors

Motor axis configurations:
```/* to be implemented */```

PWM values:
- Max forward thrust: 1900
- Max backward thrust: 1080
- Neutral thrust: 1490

Note, when sending PWM values to the motors through the Pololu Mini Maestro, we need to multiply the values by four. More documentation on the Maestro can be found [here](Maestro.md).
