#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ngws_msgs/WayPointArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
nav_msgs::Odometry odom_msg;

ngws_msgs::WayPointArray waypoints_msg;

void chatterCallback1(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_msg=*msg;
  ROS_INFO("I heard odom: %f", odom_msg.pose.pose.position.x);
}

void chatterCallback2(const ngws_msgs::WayPointArray::ConstPtr& msg)
{
  waypoints_msg=*msg;
  ROS_INFO("I heard waypoints: %f", waypoints_msg.waypoints[0].waypoint.position.x);

  ROS_INFO("I heard waypoints: %f", waypoints_msg.waypoints[1].waypoint.position.x);
 
 ROS_INFO("I heard waypoints: %f", waypoints_msg.waypoints[2].waypoint.position.x);

}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  
  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("odom_fused", 1000, chatterCallback1);
  ros::Subscriber sub2 = n.subscribe("cmd_waypoints", 1000, chatterCallback2);

 
  ros::spin();

  return 0;
}
