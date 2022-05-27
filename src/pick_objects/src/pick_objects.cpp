#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool moveToGoal(geometry_msgs::Pose pose)
{

    //define a client for to send goal requests to the move_base server through a SimpleActionClient
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //set up the frame parameters
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    /* moving towards the goal*/

    goal.target_pose.pose = pose;

    ROS_INFO("Sending goal location ...");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle nh;
    ros::Publisher checkpoint_pub = nh.advertise<std_msgs::String>("checkpoint", 10);
    ros::spinOnce();
  	ros::Rate loop_rate(1);

    bool goalReached = false;

    // Predefined locations
    geometry_msgs::Pose A;
    A.position.x = -3.031790724522435;
    A.position.y = 1.8504067625438276;
    A.position.z = 0.0;
    A.orientation.x = 0.0;
    A.orientation.y = 0.0;
    A.orientation.z = -0.9999779299754936;
    A.orientation.w = 0.006643761128066824;

    geometry_msgs::Pose B;
    B.position.x = -2.874548707258525;
    B.position.y = -1.2700492347463364;
    B.position.z = 0.0;
    B.orientation.x = 0.0;
    B.orientation.y = 0.0;
    B.orientation.z = -0.9999199405860978;
    B.orientation.w = 0.012653553583660455;

    geometry_msgs::Pose C;
    C.position.x = 3.907423171741299;
    C.position.y = -0.12776213860412772;
    C.position.z = 0.0;
    C.orientation.x = 0.0;
    C.orientation.y = 0.0;
    C.orientation.z = 0.7107860191875328;
    C.orientation.w = 0.7034082988759375;
  
    geometry_msgs::Pose origin;
    origin.position.x = 0.0;
    origin.position.y = 0.0;
    origin.position.z = 0.0;
    origin.orientation.x = 0.0;
    origin.orientation.y = 0.0;
    origin.orientation.z = 0.0;
    origin.orientation.w = 0.05;

    ROS_INFO("Moving to first goal...");
    goalReached = moveToGoal(A);
    if (goalReached)
    {
        ROS_INFO("Reached goal A.");
 	    std_msgs::String msg;
        msg.data = "A";
        checkpoint_pub.publish(msg);
      
     	ROS_INFO("Picked up item.");
        ros::Duration(5).sleep(); // pause 5 seconds simulate pick up
        
        ROS_INFO("Moving to second goal...");
        goalReached = moveToGoal(B);
    } else {
        ROS_INFO("Not able to reach goal A.");
    }
    if (goalReached)
    {
        ROS_INFO("Reached goal B.");
        std_msgs::String msg;
        msg.data = "B";
        checkpoint_pub.publish(msg);
     	ROS_INFO("Dropped off item.");
      	
      	ros::Duration(5).sleep(); // pause 5 seconds simulate drop off
    } else {
        ROS_INFO("Not able to reach goal B.");
    }
  
  	ROS_INFO("Mission Complete!");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}