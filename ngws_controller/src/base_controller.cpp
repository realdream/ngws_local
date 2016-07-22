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
#include "base_controller.hpp"
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

namespace ngws_local
{

odomCreator::odomCreator(std::shared_ptr<ros::Publisher> p_cmd_ptr, std::shared_ptr<ros::Publisher> p_odom_ptr)
{
  cmd_pub_ptr = p_cmd_ptr;
  odom_pub_ptr = p_odom_ptr;
  currentPosition.pose.pose.position.x = 0;
  currentPosition.pose.pose.position.y = 0;
  currentPosition.pose.pose.position.z = 0;
  lastTime = ros::Time::now();
  last_get_vel_time = ros::Time::now();
//TODO trans from launch file	
  wheelDis = 0.6; 
  wheelR = 0.08;
}

void odomCreator::cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  vel_msg=*msg;
  last_get_vel_time=ros::Time::now();


}

void odomCreator::main_loop()
{
  currentTime = ros::Time::now();
  ros::Duration timeBetweenLastVelMsg = ros::Time::now() - last_get_vel_time;
  if(timeBetweenLastVelMsg > ros::Duration(0.5))
  {
    vel_msg.linear.x=0.0;
    vel_msg.linear.y=0.0;
    vel_msg.linear.z=0.0;
    vel_msg.angular.x=0.0;
    vel_msg.angular.y=0.0;
    vel_msg.angular.z=0.0;
  }
  
  tf::TransformBroadcaster odomBroadcaster;

  geometry_msgs::TransformStamped odomTrans;
  odomTrans.header.stamp = currentTime;
  odomTrans.header.frame_id = "odom";
  odomTrans.child_frame_id = "base_link";

  nav_msgs::Odometry odom;
  odom.header.stamp = currentTime;
  odom.header.frame_id = "odom";


  std_msgs::Float64MultiArray cmd;

 // cmd.data.resize(4);

//	cmd.data[0] = 0.1;

  double pi;
  pi = M_PI;

  odom.twist.twist = vel_msg;
  double dt = (currentTime - lastTime).toSec();

  currentPosition.pose.pose.position.x += dt * (vel_msg.linear.x * cos(th) - vel_msg.linear.y * sin(th));
  currentPosition.pose.pose.position.y += dt * (vel_msg.linear.y * cos(th) + vel_msg.linear.x * sin(th));
  currentPosition.pose.pose.position.z = 0.0;

  


  th += dt * vel_msg.angular.z;
  geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);

  odomTrans.transform.translation.x = currentPosition.pose.pose.position.x;
  odomTrans.transform.translation.y = currentPosition.pose.pose.position.y;
  odomTrans.transform.translation.z = 0.0;
  odomTrans.transform.rotation = odomQuat;

  odomBroadcaster.sendTransform(odomTrans);

  odom.child_frame_id = "base_link";
  odom.pose.pose.position = currentPosition.pose.pose.position;
  odom.pose.pose.orientation = odomQuat;	


  buildCmdMsgForNormalWheel(cmd);

  odom_pub_ptr->publish(odom);
  cmd_pub_ptr->publish(cmd);
  lastTime = currentTime;
}

void odomCreator::buildCmdMsgForNormalWheel(std_msgs::Float64MultiArray &cmd)
{
  double xVel = vel_msg.linear.x;
  double velDiff = vel_msg.angular.z * (wheelDis / 2);
  double leftWheelVel = (xVel - velDiff) / wheelR;
  double rightWheelVel = (xVel + velDiff) / wheelR;	
  cmd.data.push_back(leftWheelVel);
  cmd.data.push_back(rightWheelVel);
  cmd.data.push_back(leftWheelVel);
  cmd.data.push_back(rightWheelVel);
}


odomCreator::~odomCreator()
{
}

}//end of ngws_local

int main(int argc, char** argv)
{
  using namespace ngws_local;

  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;


  std::shared_ptr<ros::Publisher> cmd_pub = std::make_shared<ros::Publisher>(
      nh.advertise<std_msgs::Float64MultiArray>("wheel_group_controller/command", 10));

  std::shared_ptr<ros::Publisher> odom_pub = std::make_shared<ros::Publisher>(
      nh.advertise<nav_msgs::Odometry>("odom", 10));

  std::shared_ptr<Ipublisher> l_MsgSender = std::make_shared<odomCreator>(std::move(cmd_pub), std::move(odom_pub));


  ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, &Ipublisher::cmdCallback, l_MsgSender.get());



  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&Ipublisher::main_loop, l_MsgSender.get()));

  ROS_INFO("start base_controller");
  

  ros::spin();

  return 1;
}




