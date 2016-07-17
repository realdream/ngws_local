#pragma once

#include <ros/ros.h>
#include <memory>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace ngws_local
{

class Ipublisher
{
public:
	Ipublisher() = default;
	virtual void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) = 0;
	virtual void main_loop() = 0;
	virtual ~Ipublisher() = default;

};

class odomCreator : public Ipublisher
{
public:
	odomCreator(ros::Publisher &p_cmd_pub, ros::Publisher &p_odom_pub);
  void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void main_loop();
	~odomCreator();

private:
	std::unique_ptr<ros::Publisher> cmd_pub_ptr;
	std::unique_ptr<ros::Publisher> odom_pub_ptr;

	geometry_msgs::Twist vel_msg;

	ros::Time last_get_vel_time;
	ros::Time lastTime;
	ros::Time currentTime;

	nav_msgs::Odometry currentPosition;	
	double th = 0.0;

};



}  // end of ngws_local
