/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ngws_msgs/WayPointArray.h>

ros::Publisher * cmd_pub_ptr;

nav_msgs::Odometry odom_msg;

ngws_msgs::WayPointArray waypoints_msgs;

ros::Time last_get_odom_time;
ros::Time last_get_waypoints_time;
 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

odom_msg=*msg;
last_get_odom_time=ros::Time::now();


}

 void waypointsCallback(const ngws_msgs::WayPointArray::ConstPtr& msg)
{

waypoints_msgs=*msg;
last_get_waypoints_time=ros::Time::now();


}

void main_loop()
{



  geometry_msgs::Twist cmd;
  cmd_pub_ptr->publish(cmd);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoints_actuator");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub =
      nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  cmd_pub_ptr = &cmd_pub;

  ros::Subscriber odom_sub = nh.subscribe("odom_fused", 1000, odomCallback);
  
  ros::Subscriber waypoints_sub = nh.subscribe("cmd_waypoints", 1000, waypointsCallback);

  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&main_loop));

  ROS_INFO("start waypoints_actuator");
  
  last_get_odom_time=ros::Time::now();
  last_get_waypoints_time=ros::Time::now();


  ros::spin();
}
