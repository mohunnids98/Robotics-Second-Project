#!/bin/bash

# # Optional: launch your odometry node or other necessary nodes
roslaunch second_project mapping.launch &
echo "Launching second_project..."

# # Give everything a moment to spin up
sleep 5

# Start rosbag recording
# rosbag record -O ../data/my_drive.bag /odom /gps_odom /tf /ekf &
# echo "Recording started..."

# sleep 2

# Start rosbag playback 
rosbag play --clock ../data/robotics2.bag &
echo "Playing back project.bag..."

# # Optional: launch your odometry node or other necessary nodes
# roslaunch second_project mapping.launch &
# echo "Launching second_project..."

# Give everything a moment to spin up
sleep 5

# Optional: wait for user to press [ENTER] to stop
read -p "Press ENTER to stop recording..."

# Kill the rosbag process cleanly
pkill -f "rosbag record"
