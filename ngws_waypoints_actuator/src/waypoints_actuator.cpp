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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <ngws_msgs/WayPoint.h>
#include <tf/transform_broadcaster.h>

ros::Publisher * cmd_pub_ptr;

nav_msgs::Odometry odom_msg;

ngws_msgs::WayPointArray waypoints_msgs;

ros::Time last_get_odom_time;
ros::Time last_get_waypoints_time;
float wspeed=5;
float aspeed=0.5;
float maxv=0.1;
float maxv2;
float length;
float t0,t1,t2,t3;
float t; //jishiqi
int biaozhi;
int jihao=0;
long unsigned int wplength; //shuzuchangdu
int i;//di ji ge dian

 void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  odom_msg=*msg;
  last_get_odom_time=ros::Time::now();
}

 float juli()
{
  float xlength;
  float ylength;
  float length;

  xlength=odom_msg.pose.pose.position.x-waypoints_msgs.waypoints[i].waypoint.position.x;
  ylength=odom_msg.pose.pose.position.y-waypoints_msgs.waypoints[i].waypoint.position.y;

  length=sqrt(xlength*xlength+ylength*ylength);

  return length;

}



 float jiajiao()
{
  float xlength;
  float ylength;
  float th;
  float th1;
  float th2;
  float th3;

  xlength=waypoints_msgs.waypoints[i].waypoint.position.x-odom_msg.pose.pose.position.x;
  ylength=waypoints_msgs.waypoints[i].waypoint.position.y-odom_msg.pose.pose.position.y;
  th1=xlength/sqrt(xlength*xlength+ylength*ylength);
  th2=acos(th1);
  th2=th2*180/3.1415926;
 
  if(ylength<0)
  {
   th2=-th2;
  }

geometry_msgs::Quaternion quatOfTag=odom_msg.pose.pose.orientation;
 th=tf::getYaw(quatOfTag);
 th=th*180/3.1415926;
// th=odom_msg.pose.pose.orientation.z;
  
  if(th>180)
  {
   th=th-360;
  }
  
  th3=th2-th;

  ROS_INFO("th3:%f",th3);

  return th3;

}


 float spcont(float length )
{

  float temp;
  int fangan;

  temp=maxv*maxv/aspeed;

  if(temp<length)
  {
     fangan=1;
  }
  else
  {	
  fangan=2;
  }

  return fangan;

}



 float jdtime()
{
 
  float th;
  float wtime;

  th=jiajiao();

  if(fabs(th)<=180)
  {
   if(th>0)
   {
    wspeed=5;
   }
   else
   {
    wspeed=-5;
   }
  }
  else
  {
   
   if(th>0)
   {
    wspeed=-5;
    th=360-fabs(th);
   }
   else
   {
    wspeed=5;
    th=360-fabs(th);
   }
  }
  wtime=fabs(th/wspeed);
  ROS_INFO("wtime:%f",wtime);
  return wtime;
}


 void wpinit()
{
  length=juli();
  t0=jdtime();
  biaozhi=spcont(length);

  if(biaozhi==1)
  {
   t1=maxv/aspeed;
   t2=(length-(maxv*maxv/aspeed))/maxv;
  }

  if(biaozhi==2)
  {
   t3=sqrt(length/aspeed);
   maxv2=aspeed*t3;
  }
  jihao=2;
  t=0;
}

 void waypointsCallback(const ngws_msgs::WayPointArray::ConstPtr& msg)
{

  waypoints_msgs=*msg;
  wplength=waypoints_msgs.waypoints.size();

  ROS_INFO("length:%d",wplength);
  jihao=1;
  i=0;
  last_get_waypoints_time=ros::Time::now();
}

void main_loop()
{
    geometry_msgs::Twist cmd;
    if(jihao==1)
    {
	if(i<wplength)
	{
		wpinit();
	}
   	else
	{
	return;
	}
    }


  if(jihao==2)
{
   t+=0.01;

   if(biaozhi==1)
   { 
    if (t<t0)
    {
     cmd.linear.x=0;
     cmd.linear.y=t;
     cmd.angular.z=wspeed;
    } 
    else if(t<t0+t1)
    {
     cmd.linear.x=aspeed*(t-t0);
     cmd.linear.y=t;
     cmd.angular.z=0;

    }
    else if(t<t0+t1+t2)
    {
     cmd.linear.x=maxv;
     cmd.linear.y=t;
     cmd.angular.z=0;
    }
    else if(t<t0+t1+t2+t1)
    {
     cmd.linear.x=maxv-aspeed*(t-t0-t1-t2);
     cmd.linear.y=t;
     cmd.angular.z=0;
    }

    else if(t<t0+t1+t2+t1+waypoints_msgs.waypoints[i].halt_time)
    {

     cmd.linear.x=0;
     cmd.linear.y=t;
     cmd.angular.z=0;
    }

    else
    {
     jihao=1;
     i++;
    }
   }
   if(biaozhi==2)
   {
    if (t<t0)
    {
     cmd.linear.x=0;
     cmd.linear.y=0;
     cmd.angular.z=wspeed;

    }
    if(t<t0+t3)
    {
     cmd.linear.x=aspeed*(t-t0);
     cmd.linear.y=0;
     cmd.angular.z=0;

    }
    else if(t<t0+t3+t3)
    {
     cmd.linear.x=maxv2-aspeed*(t-t0-t3);
     cmd.linear.y=0;
     cmd.angular.z=0;

    }
    else if(t<t0+t3+t3+waypoints_msgs.waypoints[i].halt_time)
    {
     cmd.linear.x=0;
     cmd.linear.y=0;
     cmd.angular.z=0;

    }
    else
    {
     jihao=1;
     i++;
    }
   }
  cmd_pub_ptr->publish(cmd);
}

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
