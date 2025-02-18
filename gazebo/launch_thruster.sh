#!/bin/bash

SCRIPT_DIR=$(realpath "$(dirname "$0")")
cd $SCRIPT_DIR/..

TMUX_INSTALLED=$(which tmux)
if ["$TMUX_INSTALLED" == ""]; then
    echo tmux is not installed
else
    tmux new-session -d -s thruster_sim

    tmux rename-window -t thruster_sim:0 'Bridge'
    tmux send-keys -t thruster_sim:0 'source install/setup.sh' C-m
    tmux send-keys -t thruster_sim:0 'ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=./gazebo/bridge_thruster.yaml' C-m

    tmux new-window -t thruster_sim -n 'MotorControl'
    tmux send-keys -t thruster_sim:1 'source /opt/ros/humble/setup.sh' C-m
    tmux send-keys -t thruster_sim:1 'colcon build' C-m
    tmux send-keys -t thruster_sim:1 'source install/setup.sh' C-m
    tmux send-keys -t thruster_sim:1 'ros2 launch motor_control motor_control_launch.xml' C-m

    tmux new-window -t thruster_sim -n 'Gazebo'
    tmux send-keys -t thruster_sim:2 'cd gazebo' C-m
    tmux send-keys -t thruster_sim:2 'ign gazebo sim_thruster.sdf' C-m
fi
