# Gazebo Sim #

Launch Gazebo

``` bash
ign gazebo gazebo/sim_test_world.sdf
```

Ros GZ Bridge (source ROS)

``` bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./gazebo/bridge_config.yaml
```

Publish to ROS (source ROS)

``` bash
ros2 topic pub cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}"
```
