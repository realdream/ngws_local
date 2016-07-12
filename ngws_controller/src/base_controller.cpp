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

ros::Publisher * cmd_pub_ptr;
ros::Publisher * odom_pub_ptr;

geometry_msgs::Twist vel_msg;

ros::Time last_get_vel_time;
 void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

vel_msg=*msg;
last_get_vel_time=ros::Time::now();


}

void main_loop()
{
  if(ros::Time::now()-last_get_vel_time>ros::Duration(0.5))
  {
    vel_msg.linear.x=0;
    vel_msg.linear.y=0;
    vel_msg.linear.z=0;
    vel_msg.angular.x=0;
    vel_msg.angular.y=0;
    vel_msg.angular.z=0;
  }
  nav_msgs::Odometry odom;
  std_msgs::Float64MultiArray cmd;

 cmd.data.resize(4);




  odom_pub_ptr->publish(odom);
  cmd_pub_ptr->publish(cmd);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub =
      nh.advertise<std_msgs::Float64MultiArray>("wheel_group_controller/command", 10);
  cmd_pub_ptr = &cmd_pub;

  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("odom", 10);
  odom_pub_ptr = &odom_pub;

  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cmdCallback);

  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&main_loop));

  ROS_INFO("start base_controller");
  
  last_get_vel_time=ros::Time::now();


  ros::spin();
}
