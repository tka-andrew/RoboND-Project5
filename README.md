# Udacity Nanodegree - Robotics Software Engineer - Project3

This is my project submission for Project5: Home Service Robot of [Udacity Nanodegree - Robotics Software Engineer](https://www.udacity.com/course/robotics-software-engineer--nd209?irclickid=U9u1PgV1xxyIROOV3m3wlTMuUkD0yqTMORvH3A0&irgwc=1&utm_source=affiliate&utm_medium=&aff=2298976&utm_term=&utm_campaign=__&utm_content=&adid=786224).

## Requirements
1. Git clone this workspace.
2. Install xterm: `sudo apt-get install xterm`
3. Clone the following packages into the workspace and catkin make:
    ```
    git clone https://github.com/ros-perception/slam_gmapping
    git clone https://github.com/turtlebot/turtlebot
    git clone https://github.com/turtlebot/turtlebot_interactions
    git clone https://github.com/turtlebot/turtlebot_simulator
    cd ../
    catkin_make
    ```
4. ROS navigation stack, map_server
5. Update the image path in my_map.yaml if necessary

## Overall process
1. Simulation setup
    - turtlebot packages are git cloned and used in this project
    - the simple world file in the `worlds` folder is used
2. Mapping
    - gmapping package is used to perform SLAM
    - the turtlebot_teleop package is used to navigate the turtlebot around during mapping process
    - the map file is then saved using map_server
    - the map files are stored in the `maps` folder
3. Navigation and Localization
    - move_base package and amcl package are used to perform navigation and localization
4. Navigation Goal node
    - with the amcl and rviz, we can get the poses of turtlebot on the map
    - the locations of interests are then recorded
    - a pick_objects C++ node is written to autonomously navigate the robot to the 2 goals
    - upon reaching each goal, the pick_objects C++ node has a publisher internally that will publish its current checkpoint (either 'A' or 'B'), which will be subscribed by add_markers node
5. Mimic of item pick-up and drop-off process
    - a add_markers C++ node is written to mimic an item being picked up and dropped off
    - in order to view the marker on rviz, it has to be added to the rviz, which is already done by the home_service.rviz stored in the `rvizConfig` folder
    - initially, the virtual marker will be displayed at position A
    - in the add_markers node, there is a subscriber subcribing to the checkpoint of the turtlebot
    - when it was informed that the current checkpoint of robot is A: hide the marker to mimic pick up process
     - when it was informed that the current checkpoint of robot is B: hide the marker to mimic drop off process
6. Shell scripts
    - shell scripts are written to launch multiple ROS nodes at once
    - the shell scripts are stored in the `scripts` folder


## Usage
### Mapping
1. `cd` into the scripts folder and run the turtlebot_mapping.sh
    ```
    ./turtlebot_mapping.sh
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