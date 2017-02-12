/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Shanghai University, Strive@Home
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  Haiqin Sun
* 				 Di Yang
* 				 Kairen Ye
*********************************************************************/
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <cstdlib> 
#include <vector>
#include <cmath> 
ros::Publisher *pose_pub;

void gettrace(const geometry_msgs::PoseStamped::ConstPtr& input)
{
	nav_msgs::Odometry pose;
	pose.header.stamp=ros::Time::now();
	pose.header.frame_id="odom";
	
	pose.pose.pose=input->pose;
	pose_pub->publish(pose);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_user_gaze");
  ros::NodeHandle n;

 ros::Publisher pub0 = n.advertise<nav_msgs::Odometry>("/user_gaze", 3);
 pose_pub=&pub0;

 ros::Subscriber sub = n.subscribe("/HL_move_base/gaze", 10, gettrace);
 ros::spin();
 ros::Rate r(30);	
  while(n.ok())
  {
	  r.sleep();
  }
  return 0;
}
