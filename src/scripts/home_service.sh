#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash;" & 
sleep 1
xterm  -e  " export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/worlds/simple.world; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/maps/my_map.yaml" &
sleep 5
xterm  -e  " rosrun rviz rviz -d /home/workspace/catkin_ws/src/rvizConfig/home_service.rviz" &
sleep 5
xterm  -e  " rosrun add_markers add_markers" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects"