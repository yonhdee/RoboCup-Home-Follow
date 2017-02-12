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
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
using namespace std;
using namespace tf;
char car_info_flag=0;
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

double pit = 0.0;
ros::Time current_time, last_time;

void car_CTL(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_time = current_time;
    current_time = ros::Time::now();
    car_info_flag=1;
    vx=msg->linear.x;
    vy=msg->linear.y;
    vth=msg->angular.z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "base_controller_fake");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(50.0);
    //ros::Subscriber sub=n.subscribe("car_info_topic",5,car_info_handle);
    ros::Subscriber vel_sub = n.subscribe("cmd_vel",1,car_CTL);
    ROS_INFO("odom runing...");

    while(n.ok()){

		last_time = current_time;
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th))*dt ;
		double delta_y = (vx * sin(th) + vy * cos(th))*dt ;
		double delta_th= vth * dt;
		
		x += delta_x;
		y += delta_y;
		th += delta_th;
        // check for incoming messages
        //compute odometry in a typical way given the velocities of the robot
        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = createQuaternionMsgFromRollPitchYaw(0,0,th);    //pit,th
        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
		odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);
        //
		ros::spinOnce();  
        r.sleep();
    }
}
