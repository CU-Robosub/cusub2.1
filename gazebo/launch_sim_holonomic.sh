#!/bin/bash

SCRIPT_DIR=$(realpath "$(dirname "$0")")

cd $SCRIPT_DIR

TMUX_INSTALLED=$(which tmux)
if ["$TMUX_INSTALLED" == ""]; then
    echo tmux is not installed
else
    session="sim_holo"

    # Start a new tmux session (no window attached)
    tmux new-session -d -s $session

    # Rename the window to "gazebo"
    tmux rename-window -t $session:0 "gazebo"

    # Split the window into two panes side by side
    tmux split-window -h

    # Run the first command in the first pane (left side)
    tmux send-keys -t $session:0.0 "ign gazebo sim_holonomic.sdf" C-m

    # Run the second command in the second pane (right side)
    tmux send-keys -t $session:0.1 "source /opt/ros/humble/setup.sh" C-m "ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=sim_holonomic_bridge.yaml" C-m

    tmux new-window -t $session:1 -n "ros"

    tmux split-window -t $session:1 -h

    tmux send-keys -t $session:1.0 "source /opt/ros/humble/setup.sh" C-m "ros2 topic pub cmd_force geometry_msgs/msg/Twist \"{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}\" -1"

    tmux send-keys -t $session:1.1 "source /opt/ros/humble/setup.sh" C-m "ros2 topic echo thrusters/force_FL"

    cd ..

    tmux new-window -t $session:2 -n "build"

    tmux split-window -t $session:2 -h

    tmux send-keys -t $session:2.0 "source /opt/ros/humble/setup.sh" C-m "colcon build" C-m

    tmux send-keys -t $session:2.1 "source install/setup.sh" C-m "ros2 launch motor_control motor_control_launch.xml"

    tmux attach -t $session

fi