#!/bin/bash

# check to see if the package has already been installed
if test -f "../../install/behaviortree_ros2" && test  -f "../../install/btcpp_ros2_interfaces"; then
  echo "Packages already installed..."
else
  echo "Installing packages: behaviortree_ros2, btcpp_ros2_interfaces..."

  # clone the git repo
  git clone https://github.com/BehaviorTree/BehaviorTree.ROS2.git --branch 0.2.0

  # copy the packages to the src directory
  cp BehaviorTree.ROS2/behaviortree_ros2 .. -R
  cp BehaviorTree.ROS2/btcpp_ros2_interfaces .. -R

  # build the packages
  cd ../.. && colcon build --packages-select behaviortree_ros2 btcpp_ros2_interfaces

  # clean up
  cd src/bt_trees && rm BehaviorTree.ROS2 -R -f
  rm ../behaviortree_ros2 -R -f
  rm ../btcpp_ros2_interfaces -R -f
fi

echo "Done!"

