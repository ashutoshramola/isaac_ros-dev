#!/bin/bash

# Setup ROS2 environment
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /workspaces/isaac_ros-dev/install/setup.bash" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash

# Restart udev daemon
sudo service udev restart
sudo chmod 777 /dev/ttyACM0

# Source setup so we can ros2 immediately
source install/setup.bash

# do nothing -- we just start the container so that other services can use it
#tail -f

$@

