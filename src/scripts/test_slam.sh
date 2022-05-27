#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash;" & 
sleep 1
xterm  -e  " source /home/workspace/catkin_ws/devel/setup.bash;" & 
sleep 1
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch" &