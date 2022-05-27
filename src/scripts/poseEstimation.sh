#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash;" & 
sleep 1
xterm  -e  " export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/worlds/simple.world; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/maps/my_map.yaml"