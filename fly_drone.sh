#!/bin/bash

ros2 launch px4_vslam run_vio.launch.xml &
vio_pid=$!

MicroXRCEAgent serial -D /dev/ttyACM0 -b 3000000 &
agent_pid=$!

ros2 launch lidar_bridge lidar_cp.launch.py &  # for collision prevention using lidar.
lidar_pid=$!

# Function to clean up processes when Ctrl+C is pressed
function cleanup() {
  echo "Stopping isaac ros commands..."
  kill $vio_pid $agent_pid $lidar_pid
  exit 0
}

# Trap the Ctrl+C signal (SIGINT) and call the cleanup function
trap cleanup INT

# Wait for any of the background processes to finish
wait 
echo "Stopped all processes!!"

