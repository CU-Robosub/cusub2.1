# Gazebo Sim #

We use Gazebo Fortress.

**Note:** after this version of Gazebo, there was a name change in lots of the commands. For our version, most of the packages/commands are prefaced `ignition` or `ign`. All modern packages/commands (and corresponding documentation) uses `gazebo` or `gz`. Make sure to look at documentation for Fortress only.

## Install ##

[Installation instructions](https://gazebosim.org/docs/fortress/ros_installation/)

For a normal install, you just need run this command:

``` bash
sudo apt-get install ros-humble-ros-gz
```

## Launch Thruster Sim ##

Run launch script (requires tmux)

``` bash
gazebo/launch_thruster.sh
```

This script will launch create a tmux session and automatically build and launch ROS, and launch Gazebo. To switch between the terminal windows, hit `^b` (ctrl-b) and type the window number (as seen on the bottom of the screen). To To kill the tmux session, hit `^b`  and type `:kill-session`.

## Manual Thruster Sim Launch ##

Launch Gazebo (must be in /gazebo dir!)

``` bash
cd gazebo
ign gazebo sim_thruster.sdf
```

Launch ROS-Gazebo Bridge (source local ROS). The bridge connects ROS topics to Gazebo topics. These connections are defined in `bridge_thruster.yaml`.

``` bash
source install/setup.sh
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./gazebo/bridge_thruster.yaml
```

Launch ROS (source local ROS)

``` bash
source install/setup.sh
ros2 launch simulation thruster_sim_launch.xml
```

## Publishing to ROS (source local ROS) ##

``` bash
ros2 source install/setup.sh
ros2 topic pub cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}"
```

## Publishing to Gazebo ##

``` bash
ign topic -t /doubleTopic -m ignition.msg.Double -p "data: 1.0"
```

## Useful Links ##

[Gazebo System Plugins](https://gazebosim.org/api/gazebo/6/namespaceignition_1_1gazebo_1_1systems.html)

[ROS <-> Gazebo Topic Conversion](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md#example-1a-gazebo-transport-talker-and-ros-2-listener)

[SDF Spec](http://sdformat.org/spec)
