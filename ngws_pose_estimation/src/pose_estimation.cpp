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

namespace pose_estimation
{
/*
ros::Publisher * odom_pub_ptr;

nav_msgs::Odometry odom_msg;
geometry_msgs::PoseStamped pose_msg;
std_msgs::Int8 pose_status_msg;
 
ros::Time last_get_odom_time;
ros::Time last_get_pose_time;
ros::Time last_get_pose_status_time;*/
poseEstimation::poseEstimation(std::shared_ptr<ros::Publisher> p_odom_pub_ptr) : odom_pub_ptr(p_odom_pub_ptr), last_get_odom_time(ros::Time::now()), last_get_pose_time(ros::Time::now()), last_get_pose_status_time(ros::Time::now())
{

}

void poseEstimation::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

odom_msg=*msg;
last_get_odom_time=ros::Time::now();


}

void poseEstimation::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  pose_msg=*msg;
  last_get_pose_time=ros::Time::now();

  geometry_msgs::Quaternion quatOfTag = pose_msg.pose.orientation;
  double yawOfTag = tf::getYaw(quatOfTag);
  double angleOfTag = (yawOfTag/M_PI)*180;
  std::cout << "Angle of tag: " << angleOfTag << std::endl;

}

void poseEstimation::poseStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{

pose_status_msg=*msg;
last_get_pose_status_time=ros::Time::now();


}

void poseEstimation::main_loop()
{
  nav_msgs::Odometry odom;


  /*add tf boardcast from world to base_footprint*/



  odom_pub_ptr->publish(odom);
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
  std::shared_ptr<IPoseEstimation> l_poseEstimator = std::make_shared<poseEstimation>(std::move(odom_pub));

  ros::Subscriber odom_sub = nh.subscribe("odom", 1000, &IPoseEstimation::odomCallback, l_poseEstimator.get());

  ros::Subscriber tracker_pose_sub = nh.subscribe("tag_tracker/object_position", 1000, &IPoseEstimation::poseCallback, l_poseEstimator.get());

  ros::Subscriber tracker_status_sub = nh.subscribe("tag_tracker/status", 1000, &IPoseEstimation::poseStatusCallback, l_poseEstimator.get());

  ros::Timer cmd_timer =nh.createTimer(ros::Duration(0.01), boost::bind(&IPoseEstimation::main_loop, l_poseEstimator.get()));

  ROS_INFO("start pose_estimation");


  ros::spin();
}
