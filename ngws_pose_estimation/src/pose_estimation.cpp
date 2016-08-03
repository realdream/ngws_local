/*
 *
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
#include "pose_estimation.hpp"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iostream>

namespace pose_estimation
{

poseEstimation::poseEstimation(std::shared_ptr<ros::Publisher> p_odom_pub_ptr, ros::Time p_time) : odom_pub_ptr(p_odom_pub_ptr), last_get_odom_time(p_time), last_get_pose_time(p_time), last_get_pose_status_time(p_time), last_get_tag_msg_time(p_time)
{
  offsetOfYaw = 0.0;
  offsetOfX = 0.0;
  offsetOfY = 0.0;
  doffsetOfYaw = M_PI/60;
  doffsetOfX = 0.1/30;
  doffsetOfY = 0.1/30;
}

void poseEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

odom_msg=*msg;
last_get_odom_time=ros::Time::now();


}

double poseEstimation::convertAngleInTwoPI(double p_angle)
{
  if(p_angle < (0 - M_PI))return convertAngleInTwoPI(p_angle + (2 * M_PI));
  if(p_angle > M_PI)return convertAngleInTwoPI(p_angle - (2 * M_PI));
  return p_angle;
}

double poseEstimation::getYawFromQuat(geometry_msgs::Quaternion p_quat)
{
  double l_yaw = tf::getYaw(p_quat);
  return convertAngleInTwoPI(l_yaw);

}

void poseEstimation::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double l_offsetOfYaw = 0.0;
  double l_offsetOfY = 0.0;
  double l_offsetOfX = 0.0;

  pose_msg=*msg;
  last_get_pose_time=ros::Time::now();

  if(pose_status_msg.data != 3)return;
  if(ros::Time::now() - last_get_pose_status_time > ros::Duration(0.5))return;
  if(ros::Time::now() - last_get_tag_msg_time > ros::Duration(0.5)) return;
  if(pose_msg.pose.position.x == 0 || pose_msg.pose.position.x == 0.0)return;

  updateQrcodePosition();	

  geometry_msgs::Quaternion quatOfTag = pose_msg.pose.orientation;
  double yawOfTag = tf::getYaw(quatOfTag);
  double realYaw = convertAngleInTwoPI(yawOfTag - M_PI / 2);
  double realX = pose_msg.pose.position.y + qrcodePositionX;
  double realY = pose_msg.pose.position.x + qrcodePositionY;

  geometry_msgs::Quaternion quatOfVehicle = odom_msg.pose.pose.orientation;
  l_offsetOfYaw = convertAngleInTwoPI(realYaw - getYawFromQuat(quatOfVehicle)); 
  l_offsetOfX = realX - odom_msg.pose.pose.position.x;
  l_offsetOfY = realY - odom_msg.pose.pose.position.y;

  if(l_offsetOfX > offsetOfX)
  {
    offsetOfX += doffsetOfX;
    if(l_offsetOfX <= offsetOfX)offsetOfX = l_offsetOfX;
  }
  else
  {
    offsetOfX -= doffsetOfX;
    if(l_offsetOfX >= offsetOfX)offsetOfX = l_offsetOfX;
  }

  if(l_offsetOfY > offsetOfY)
  {
    offsetOfY += doffsetOfY;
    if(l_offsetOfY <= offsetOfY)offsetOfY = l_offsetOfY;
  }
  else
  {
    offsetOfY -= doffsetOfY;
    if(l_offsetOfY >= offsetOfY)offsetOfY = l_offsetOfY;
  }

  if(convertAngleInTwoPI(l_offsetOfYaw - offsetOfYaw) > 0.0)
  {
    offsetOfYaw += doffsetOfYaw;
    offsetOfYaw = convertAngleInTwoPI(offsetOfYaw);
    if(convertAngleInTwoPI(l_offsetOfYaw - offsetOfYaw) <= 0.0)offsetOfYaw = l_offsetOfYaw;
  }
  else
  {
    offsetOfYaw -= doffsetOfYaw;
    offsetOfYaw = convertAngleInTwoPI(offsetOfYaw);
    if(convertAngleInTwoPI(l_offsetOfYaw - offsetOfYaw) >= 0.0)offsetOfYaw = l_offsetOfYaw;
  }

//  std::cout << "offsetOfX: " << offsetOfX << std::endl;
//  std::cout << "offsetOfYaw: " << offsetOfYaw << std::endl;
//  double angleOfTag = (yawOfTag/M_PI)*180;
//  std::cout << "Angle of tag: " << angleOfTag << std::endl;
}

void poseEstimation::updateQrcodePosition()
{
  std::string msgData = tag_msg.data.data(); 
  std::stringstream sStream;
  int pos = msgData.find(',');
  if(pos == -1)return;
  std::string tmp = msgData.replace(pos, 1, 1, ' ');	
  sStream << tmp;
  sStream >> qrcodePositionX;
  sStream >> qrcodePositionY;
//	std::cout << "x: " << qrcodePositionX << "y: " << qrcodePositionY<< std::endl;
}

void poseEstimation::poseStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{

pose_status_msg=*msg;
last_get_pose_status_time=ros::Time::now();


}

void poseEstimation::buildOdomfusedMsg(nav_msgs::Odometry & odom)
{
  if(odom_msg.pose.pose.orientation.x == 0 && odom_msg.pose.pose.orientation.y ==0 && odom_msg.pose.pose.orientation.z ==0 &&odom_msg.pose.pose.orientation.w ==0 ) return;

  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time = ros::Time::now();

  geometry_msgs::Quaternion quadOfVehicle = odom_msg.pose.pose.orientation;


  double x = odom_msg.pose.pose.position.x + offsetOfX;
  double y = odom_msg.pose.pose.position.y + offsetOfY;
  double th = convertAngleInTwoPI(tf::getYaw(quadOfVehicle) + offsetOfYaw);
  std::cout << "Angle of vehicle: " << th << "  " << (th/M_PI)*180.0 << std::endl;

  double vx = odom_msg.twist.twist.linear.x;
  double vy = odom_msg.twist.twist.linear.y;
  double vth = odom_msg.twist.twist.angular.z;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

}

void poseEstimation::main_loop()
{
  nav_msgs::Odometry odom;


  /*add tf boardcast from world to base_footprint*/

  buildOdomfusedMsg(odom);


  odom_pub_ptr->publish(odom);
}

void poseEstimation::tagMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  tag_msg = *msg;




  last_get_tag_msg_time = ros::Time::now();	

}


poseEstimation::~poseEstimation()
{
}


}//end of namespace
int main(int argc, char** argv)
{
  using namespace pose_estimation;

  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;

  std::shared_ptr<ros::Publisher> odom_pub = std::make_shared<ros::Publisher>(
      nh.advertise<nav_msgs::Odometry>("odom_fused", 10));
//  odom_pub_ptr = &odom_pub;
  std::shared_ptr<IPoseEstimation> l_poseEstimator = std::make_shared<poseEstimation>(std::move(odom_pub), ros::Time::now());

  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, &IPoseEstimation::odomCallback, l_poseEstimator.get());

  ros::Subscriber tracker_pose_sub = nh.subscribe("tag_tracker/object_position", 1000, &IPoseEstimation::poseCallback, l_poseEstimator.get());

  ros::Subscriber tracker_status_sub = nh.subscribe("tag_tracker/status", 1000, &IPoseEstimation::poseStatusCallback, l_poseEstimator.get());

  ros::Subscriber tag_msg_sub = nh.subscribe("tag_tracker/code_message", 1000, &IPoseEstimation::tagMsgCallback, l_poseEstimator.get());

  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&IPoseEstimation::main_loop, l_poseEstimator.get()));

  ROS_INFO("start pose_estimation");


  ros::spin();
}
