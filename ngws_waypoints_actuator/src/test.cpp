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

 ros::Publisher position_pub = n.advertise<ngws_msgs::WayPointArray>("cmd_waypoints", 1000);
ros::Rate loop_rate(0.01);
nav_msgs::Odometry msg;
ngws_msgs::WayPoint msg1;
ngws_msgs::WayPointArray msg2;

     msg1.waypoint.position.x=0;
     msg1.waypoint.position.y=1;
     msg1.waypoint.orientation.z=90;
       msg2.waypoints.push_back(msg1);

     msg1.waypoint.position.x=1;
     msg1.waypoint.position.y=1;
     msg1.waypoint.orientation.z=0;
       msg2.waypoints.push_back(msg1);

    msg1.waypoint.position.x=1;
    msg1.waypoint.position.y=2;
    msg1.waypoint.orientation.z=0;
       msg2.waypoints.push_back(msg1);
//ros::Rate loop_rate(0.01);

while(ros::ok()) {
//if(biaozhi==1){

 // nav_msgs::Odometry msg;
 
//  msg.pose.pose.position.x=0.0;
 // msg.pose.pose.position.y=0.0;
 // msg.pose.pose.orientation.z=0.0;


  
//     chatter_pub.publish(msg);
 
     position_pub.publish(msg2);
   chatter_pub.publish(msg);    
     ros::spinOnce();
    loop_rate.sleep(); 
//     biaozhi=0;
  //  }
    }

}
