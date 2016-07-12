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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>

ros::Publisher * odom_pub_ptr;

nav_msgs::Odometry odom_msg;
geometry_msgs::PoseStamped pose_msg;
std_msgs::Int8 pose_status_msg;
 
ros::Time last_get_odom_time;
ros::Time last_get_pose_time;
ros::Time last_get_pose_status_time;

 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

odom_msg=*msg;
last_get_odom_time=ros::Time::now();


}

 void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

pose_msg=*msg;
last_get_pose_time=ros::Time::now();


}

void poseStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{

pose_status_msg=*msg;
last_get_pose_status_time=ros::Time::now();


}

void main_loop()
{
  nav_msgs::Odometry odom;


  /*add tf boardcast from world to base_footprint*/



  odom_pub_ptr->publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("odom_fused", 10);
  odom_pub_ptr = &odom_pub;

  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomCallback);

  ros::Subscriber tracker_pose_sub = nh.subscribe("tag_tracker/object_position", 1000, poseCallback);

  ros::Subscriber tracker_status_sub = nh.subscribe("tag_tracker/status", 1000, poseStatusCallback);

  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&main_loop));

  ROS_INFO("start pose_estimation");
  
  last_get_odom_time=ros::Time::now();
  last_get_pose_time=ros::Time::now();
  last_get_pose_status_time=ros::Time::now();



  ros::spin();
}
