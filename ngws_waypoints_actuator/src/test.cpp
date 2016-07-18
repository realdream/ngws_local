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
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("odom_fused", 1000);

  nav_msgs::Odometry msg
  msg.pose.pose.position.x=0.0;
  msg.pose.pose.position.y=0.0;
  msg.pose.pose.orientation.z=0.0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if(biaozhi==1)
    { 
   	chatter_pub.publish(msg);
   	biaozhi=0;
     }   
    ros::spinOnce();
    loop_rate.sleep();  
  }
}
