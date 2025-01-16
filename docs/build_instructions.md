# Build Instructions #

## Clone Repository ##

``` bash
git clone https://github.com/CU-Robosub/cusub2.1.git  # HTTPS
git clone git@github.com:CU-Robosub/cusub2.1.git      # SSH

cd cusub2.1
```

## Set Up Build Environment ##

Whenever working with `ros` commands, you need to source ROS:

``` bash
source /opt/ros/humble/setup.bash # or wherever your ROS2 installation is
```

Resolve Dependencies

Use the rosdep tool to install the dependencies

``` bash
rosdep install -i --from-path src --rosdistro humble -y
```

## Build and Run ##

Build the project using colcon

``` bash
colcon build
```

When we build the project, files for a ROS2 overlay workspace will be generated. We can activate this workspace in a new terminal:

Source ros2 and go to workspace directory

``` bash
source /opt/ros/humble/setup.bash
cd /path/to/cusub2.1
```

Source the overlay

``` bash
. install/local_setup.bash
```

Now we can run from the overlay workspace

``` bash
ros2 run package node
```

## Set Up Development Environment ##

[Intellisense](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)

Settings > Python > Analysis: Extra Paths

``` json
"python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
],
```
