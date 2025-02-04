# Gazebo Sim #

Launch Gazebo (in gazebo dir)

``` bash
ign gazebo sim_test_world.sdf
```

Ros GZ Bridge (source ROS)

``` bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./gazebo/bridge_config.yaml
```

Publish to ROS (source ROS)

``` bash
ros2 topic pub cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}"
```

## Useful Links ##

[Gazebo System Plugins](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo_1_1systems.html)

[ROS <-> Gazebo Topic Conversion](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-gazebo-transport-talker-and-ros-2-listener)

[SDF Spec](http://sdformat.org/spec)

## Holonomic Control Sim ##

```bash
# ros sourced - build
colcon build

# overlay sourced - launch
ros2 launch motor_control motor_control_launch.xml

# nothing sourced - run gazebo
ign gazebo sim_holonomic.sdf

# ros sourced - run ros_gz bridge
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./gazebo/sim_holonomic_bridge.yaml

# ros sourced - publish cmd_force
ros2 topic pub cmd_force geometry_msgs/msg/Twist "{linear: {x: 1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}"

# ros sourced - echo the output
ros2 topic echo thrusters/force_FL
```
