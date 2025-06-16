#!/bin/bash

# # Optional: launch your odometry node or other necessary nodes
roslaunch second_project mapping2.launch &
echo "Launching second_project..."

# # Give everything a moment to spin up
sleep 5



## Start Recording
Start rosbag recording
rosbag record -O ../data/project.bag /merged_scan /filtered_front_scan /filtered_back_scan /tf /map &
echo "Recording started..."

sleep 2



# Start rosbag playback 
rosbag play --clock ../data/robotics2.bag 
echo "Playing back project.bag..."

# Wait a bit so map is built
sleep 15


## Start the map server 

rosrun  map_server map_saver -f maps/my_map

echo "Started Map Server..."

sleep 2
# # Optional: launch your odometry node or other necessary nodes
# roslaunch second_project mapping.launch &
# echo "Launching second_project..."

# Give everything a moment to spin up
sleep 5

# Optional: wait for user to press [ENTER] to stop
read -p "Press ENTER to stop recording..."

# Kill the rosbag process cleanly
pkill -f "rosbag record"
