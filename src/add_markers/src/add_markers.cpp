#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>

// global marker
visualization_msgs::Marker marker;

// global publisher
ros::Publisher marker_pub;

void checkpointCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Current checkpoint: [%s]", msg->data.c_str());
  if (msg->data == "A")
  {
    marker.action = visualization_msgs::Marker::DELETE;
   	marker_pub.publish(marker);
  } else if (msg->data == "B")
  {
    geometry_msgs::Pose B;
    B.position.x = -2.874548707258525;
    B.position.y = -1.2700492347463364;
    B.position.z = 0.0;
    B.orientation.x = 0.0;
    B.orientation.y = 0.0;
    B.orientation.z = -0.9999199405860978;
    B.orientation.w = 0.012653553583660455;
    marker.pose = B; // update to new pose
  	
  	marker.action = visualization_msgs::Marker::ADD;
  	marker_pub.publish(marker);
  } else {
    ROS_INFO("Unknown checkpoint!");
  }
    
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ros::Subscriber sub = n.subscribe("checkpoint", 10, checkpointCallback);

  uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    geometry_msgs::Pose A;
    A.position.x = -3.031790724522435;
    A.position.y = 1.8504067625438276;
    A.position.z = 0.0;
    A.orientation.x = 0.0;
    A.orientation.y = 0.0;
    A.orientation.z = -0.9999779299754936;
    A.orientation.w = 0.006643761128066824;
    marker.pose = A;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Waiting for subscriber...");
      sleep(1);
    }
  
  	ROS_INFO("Subscriber found!");
  	
    marker_pub.publish(marker);
  
  	while (ros::ok())
  	{
      ros::spinOnce();
      loop_rate.sleep();
    }
}