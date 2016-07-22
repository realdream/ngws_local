#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <memory>

namespace pose_estimation
{


class IPoseEstimation
{
public:
  IPoseEstimation() = default;
  virtual void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) = 0;
  virtual void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) = 0;
  virtual void poseStatusCallback(const std_msgs::Int8::ConstPtr& msg) = 0;
  virtual void main_loop() = 0;
  virtual	~IPoseEstimation() = default;
};

class poseEstimation : public IPoseEstimation
{
public:
  poseEstimation(std::shared_ptr<ros::Publisher> p_odom_pub_ptr);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void poseStatusCallback(const std_msgs::Int8::ConstPtr& msg);
  void main_loop();
  ~poseEstimation();
private:
  std::shared_ptr<ros::Publisher>	odom_pub_ptr;

  ros::Time last_get_odom_time;	
  ros::Time last_get_pose_time;
  ros::Time last_get_pose_status_time;

  nav_msgs::Odometry odom_msg;
  geometry_msgs::PoseStamped pose_msg;
  std_msgs::Int8 pose_status_msg;

};




}//end namespace
