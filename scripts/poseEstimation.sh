#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash;" & 
sleep 1
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=\"$(rospack find home_service)/worlds/simple.world\"" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=\"$(rospack find home_service)/maps/my_map.yaml\""