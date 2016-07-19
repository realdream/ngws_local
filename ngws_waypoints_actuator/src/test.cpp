#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ngws_msgs/WayPointArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
//int biaozhi=1;
int main(int argc, char **argv)
{ 
  int biaozhi=1;
  ros::init(argc, argv, "test1");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("odom_fused", 1000);
 
//  ros::Publisher position_pub = n.advertise<ngws_msgs::WayPointArray>("cmd_waypoints", 1000);
nav_msgs::Odometry msg;
//ngws_msgs::WayPointArray waypoint_msg[3];
while((ros::ok()) ){
if(biaozhi==1){
 
 // nav_msgs::Odometry msg;
 
  msg.pose.pose.position.x=0.0;
  msg.pose.pose.position.y=0.0;
  msg.pose.pose.orientation.z=0.0;

//  ngws_msgs::WayPointArray waypoint_msg;
/* waypoint_msg.waypoints[0].waypoint.position.x=0;
 waypoint_msg.waypoints[0].waypoint.position.y=0;
 
 waypoint_msg.waypoints[1].waypoint.position.x=0;
 waypoint_msg.waypoints[1].waypoint.position.y=1;
  
 waypoint_msg.waypoints[2].waypoint.position.x=1;
 waypoint_msg.waypoints[2].waypoint.position.x=1;
  

 
       position_pub.publish(msg);*/
       chatter_pub.publish(msg);
       
  	biaozhi=0;
    }
      }
    ros::spinOnce();


}
