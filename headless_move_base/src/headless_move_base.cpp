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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sound_play/sound_play.h>
#include <math.h>
geometry_msgs::PoseStamped temp_gaze,temp_goal;
nav_msgs::Odometry temp_odom;
ros::Publisher cmd_pub;
ros::Publisher dist_pub;
ros::Publisher state_pub;
geometry_msgs::Twist vel_out;//to cmd_vel
tf::TransformListener *listener;
static int state=0;
static int atpoint=0;
static int waiTenSeconds=0;//0 ready 1 wait 2 stop
static int voice_ctr=0;//0 wait voice control 1 start 2 stop
ros::Time time0;
ros::Time timerForWait;
ros::Time Wait;
sound_play::SoundClient* sc;
void gazeCB (const geometry_msgs::PoseStampedConstPtr& gaze)//
{
        geometry_msgs::PoseStamped _gaze,base_gaze;//xframeid to base footprint
			_gaze=*gaze;
			ROS_INFO("get gaze");
			try
			{
				listener->transformPose("odom", _gaze, base_gaze);
			}
			catch(tf::TransformException& ex)
			{
				 	ROS_ERROR("gaze %s", ex.what());
			return;  					
			}
		atpoint=0;
		state=1;
		temp_gaze=base_gaze;
}
void voiceCtr(const std_msgs::String::ConstPtr& voice)//
{
	//voice command
}
void goalCB (const geometry_msgs::PoseStampedConstPtr& goal)
{
        geometry_msgs::PoseStamped _goal,base_goal;//xframeid to base footprint
			_goal=*goal;
			ROS_INFO("get goal");
			//_gaze.header.stamp=ros::Time(0);
			try
			{
				listener->transformPose("odom", _goal, base_goal);
			}
			catch(tf::TransformException& ex)
			{
				 	ROS_ERROR("goal %s", ex.what());
			return;  					
			}
		atpoint=0;
		state=1;
		temp_goal=base_goal;
}
void odomCB (const nav_msgs::OdometryConstPtr& odom)
{
	temp_odom=*odom;
}
void main_loop (const ros::TimerEvent& event)
{


	if(state==1)
	{
			geometry_msgs::PoseStamped base_gaze,base_goal;//xframeid to base footprint
			try
			{
				listener->transformPose("odom", temp_gaze, base_gaze);
			}
			catch(tf::TransformException& ex)
			{
				 	ROS_ERROR("gaze %s", ex.what());
			return;  					
			}
			try
			{
				listener->transformPose("odom", temp_goal, base_goal);
			}
			catch(tf::TransformException& ex)
			{
				 	ROS_ERROR("goal %s", ex.what());
			return;  					
			}

			double yaw=atan2(2*temp_odom.pose.pose.orientation.z*temp_odom.pose.pose.orientation.w,1-2*(temp_odom.pose.pose.orientation.z*temp_odom.pose.pose.orientation.z));
			double x=temp_odom.pose.pose.position.x;
			double y=temp_odom.pose.pose.position.y;
			double gaze_x=base_gaze.pose.position.x;
			double gaze_y=base_gaze.pose.position.y;
			double goal_x=base_goal.pose.position.x;
			double goal_y=base_goal.pose.position.y;
			double vx=goal_x-x;
			double vy=goal_y-y;
			double vv=sqrt(vx*vx+vy*vy)*2;
			double goal_dist=vv;

			if(vv==0.0)vv=0.4;
			double vx_n=vx/vv;
			double vy_n=vy/vv;
			double vyaw=atan2(gaze_y-y,gaze_x-x)-yaw;
			double yaw_=yaw+temp_odom.twist.twist.angular.z*0.03;
			if(vyaw>M_PI)
			vyaw-=2*M_PI;
			if(vyaw<-M_PI)
			vyaw+=2*M_PI;
			vyaw=vyaw*6;
			if(vyaw>0.4)
			vyaw=0.4;
			if(vyaw<-0.4)
			vyaw=-0.4;
			if(vv>0.4)
			vv=0.4;
			if(vv<-0.4)
			vv=-0.4;
			if(fabs(vyaw)<0.05)
				vyaw=0;
			if(vyaw>0.05)
				vyaw-=0.05;
			if(vyaw<-0.05)
				vyaw+=0.05;
			double vx_old=vel_out.linear.x;
			double vy_old=vel_out.linear.y;
			double vx_new=cos(yaw_)*vx_n*vv+sin(yaw_)*vy_n*vv;
			double vy_new=-sin(yaw_)*vx_n*vv+cos(yaw_)*vy_n*vv;
			vx_new=vx_old+(vx_new-vx_old)/50.0;
			vy_new=vy_old+(vy_new-vy_old)/50.0;
			vel_out.angular.x=0.0;
			vel_out.angular.y=0.0;
			vel_out.angular.z=vyaw;
			vel_out.linear.x=vx_new;
			vel_out.linear.y=vy_new;
			vel_out.linear.z=0.0;
			std_msgs::Float64 dist_out;
			dist_out.data=goal_dist;
			dist_pub.publish(dist_out);
			//std::cout<<goal_dist<<"QW\n";
						//printf("%f-%f-%f-%f\n",x,y,gaze_x,vyaw);
			if(goal_dist<0.05&&fabs(vyaw)<0.08&&atpoint==0)
			{
				time0=ros::Time::now();
				atpoint=1;
			}
			if(atpoint==1)
			{
				if(ros::Time::now()-time0>ros::Duration(5))
				{
					atpoint=0;
					state=3;
					vel_out.linear.x=0.0;
					vel_out.linear.y=0.0;
					vel_out.linear.z=0.0;
					vel_out.angular.x=0.0;
					vel_out.angular.y=0.0;
					vel_out.angular.z=0.0; 
				}
			}
			cmd_pub.publish(vel_out);	
	}//if state==1

	std_msgs::Int16 state_out;
	state_out.data=state;
	state_pub.publish(state_out);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "headless_move_base");
	ros::NodeHandle n;
	temp_gaze.header.frame_id="odom";
	temp_gaze.pose.position.x=0;
	temp_gaze.pose.position.y=0;
	temp_gaze.pose.position.z=0;
	temp_gaze.pose.orientation.x=0;
	temp_gaze.pose.orientation.y=0;
	temp_gaze.pose.orientation.z=0;
	temp_gaze.pose.orientation.w=1;

	temp_goal.header.frame_id="odom";
	temp_goal.pose.position.x=0;
	temp_goal.pose.position.y=0;
	temp_goal.pose.position.z=0;
	temp_goal.pose.orientation.x=0;
	temp_goal.pose.orientation.y=0;
	temp_goal.pose.orientation.z=0;
	temp_goal.pose.orientation.w=1;

	tf::TransformListener listener0(ros::Duration(1.0/100.0));
	listener=&listener0; 

	timerForWait=ros::Time::now();	
	sound_play::SoundClient sc0;
	sc=&sc0;

	ros::Subscriber sub0 = n.subscribe("/HL_move_base/gaze",50,gazeCB);
	ros::Subscriber sub1 = n.subscribe("/HL_move_base/goal",50,goalCB);
	ros::Subscriber sub2 = n.subscribe("/odom",50,odomCB);
	
	ros::Subscriber sub_voice = n.subscribe("/follow_wait",1,voiceCtr);
	
	ros::Timer timer1 = n.createTimer(ros::Duration(1.0/100.0), main_loop);

    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    dist_pub = n.advertise<std_msgs::Float64>("/HL_move_base/goal_dist", 50);
    state_pub = n.advertise<std_msgs::Int16>("/HL_move_base/state", 50);
    ROS_INFO("headless_move_base runing...");
    ros::spin ();
return 0;
}
