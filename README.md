# Udacity Nanodegree - Robotics Software Engineer - Project3

This is my project submission for Project5: Home Service Robot of [Udacity Nanodegree - Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209?irclickid=U9u1PgV1xxyIROOV3m3wlTMuUkD0yqTMORvH3A0&irgwc=1&utm_source=affiliate&utm_medium=&aff=2298976&utm_term=&utm_campaign=__&utm_content=&adid=786224).

## Write-up
### Simulatenous Localization and Mapping (SLAM)
Mapping is about estimating the map with given measurements and robot's pose whereas Localization is about estimating the robot's pose with given motion control, map, and measurements. When the robot doesn't have both map and robot's pose, SLAM is what the robot needs. The inputs to the SLAM problem are measurements and controls as inputs, whereas the outputs are maps and trajectory/pose.

[gmapping](http://wiki.ros.org/gmapping) ROS package is used in this project. Underlying this package, it provides a ROS node called slam_gmapping which takes in laser scan and tf data as inputs, and then generates a 2-D occupancy grid map. It is based on the Grid-based FastSLAM algorithm. Grid-based FastSLAM algorithm is able to model the environment using grid maps without predefining any landmark position, this explains why the slam_gmapping node is able to perform mapping without any predefined landmarks.

The robot has to move around to update the map, this can be done either through manual teleoperation or by using the '2D Nav Goal' feature on RVIZ.

Once the mapping is done, the map_saver command-line utility of [map_server](http://wiki.ros.org/map_server) ROS package can be used to save the map.

Side note: Turtlebot doesn't have a laser scanner, it only has an RGB-D camera. Howewer, its launch file does have a depthimage_to_lasercan node which converts the depthimage into laser scan data. Hence, gmapping package can be used for Turtlebot.

### Localization
Localization is about estimating the robot's pose with given motion control, map, and measurements. With the map generated and saved using gmapping and map_server packages, the robot can now perform localization using the [amcl](http://wiki.ros.org/amcl) package.

AMCL stands for Adaptive Monte Carlo Localization, which is a variant of Monte Carlo Localization(MCL). MCL is able to solve both local and global localization problems. It uses particles to localize the robot's pose. If you look at the RVIZ, initially the green arrows are scattered around, but after moving the robot around, the arrows start to converge and more accurately estimating the robot's pose.

AMCL offers a significant computational advantage over MCL as it is able to dynamically adjust the number of particles over a period of time as the robot navigates around in the map. 

The inputs required for this amcl package are laser-based map, laser scans, and transform messages. Hence, the amcl launch file normally also runs the [map_server](http://wiki.ros.org/map_server) node to provide map data as a ROS service. Again, since the turtlebot has the fake laser scanner data, amcl package can be used for Turtlebot as well.

### Navigation
The [move_base](http://wiki.ros.org/move_base) ROS package is used to perform navigation. With this package, we can define a navigation goal position on the map, and then the robot will navigate to it.

This move_base ROS node has a global planner that finds the optimal path with a prior knowledge of the environment and static obstacles through global costmap. Meanwhile it also has a local planner that recalculates the path to avoid dynamic obstacles noticed by local costmap.

Besides that, the move_base ROS node is also able to perform recovery behaviors when the robot perceives itself as stuck. 

The move_base node provides an implementation of the SimpleActionServer. Hence, we can use SimpleActionClient to publish our goal to that server, then the ROS node will start the navigation. To learn about ROS Action and how to write the SimpleActionClient, refer to [actionlib roswiki](http://wiki.ros.org/actionlib).

## Usage
### Mapping
1. `cd` into the scripts folder and run the mapping.sh
    ```
    ./mapping.sh
    ```
2. Navigate the robot around to perform mapping
3. Run the following to save the map
    ```
    rosrun map_server map_saver -f my_map
    ```
4. Move the my_map.pgm and my_map.yaml to correct position and edit the my_map.yaml if necessary.

### Selecting locations of interest
1. `cd` into the scripts folder and run the poseEstimation.sh
    ```
    ./poseEstimation.sh
    ```
2. Use the '2D Pose Estimate' feature of rviz to estimate the location that you want to use.
3. Echo the /amcl_pose topic to see the location and orientation of the robot:
    ```
    rostopic echo /amcl_pose
    ```
4. Jot down the position and orientation for each location of interest, add them to your navigation node afterwards.

### Running the home service demo
1. `cd` into the scripts folder and run the home_service.sh
    ```
    ./home_service.sh
    ```