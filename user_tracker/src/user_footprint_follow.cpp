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
/**************************************
******* 机器人跟随 **********************
***************************************/
#include <user_tracker/user_tracker.h>

/********************************
**** 获取两点间直线距离 **********
*********************************/
double get_distance(geometry_msgs::PoseStamped & pose1, geometry_msgs::PoseStamped & pose2)
{
	double x=pose1.pose.position.x-pose2.pose.position.x;
	double y=pose1.pose.position.y-pose2.pose.position.y;
	return sqrt(x*x+y*y);
}
/***********************************
* 距离机器人最近的点相对于激光的坐标 *
************************************/
void get_blindage(const user_tracker_msgs::Blindage &blindage)
{
	double range=blindage.range;
	double ang=blindage.angle;
	goal_blind.x=range*cos(ang)-cos(M_PI/3-ang);
	goal_blind.y=range*sin(ang)+sin(M_PI/3-ang);
	goal_blind.z=0;
}
/***********************************
* 距离 *
************************************/
void get_goal_dist(const std_msgs::Float64 &dist_input)
{
	dist=dist_input.data;
}
/***********************************
*********** 获取odom ***************
************************************/
void get_odom(const nav_msgs::Odometry &odom_input)
{
	odom=odom_input;
}
/***********************************
*********** 语音指令 ***************
************************************/
void get_voice_cmd(const std_msgs::String &voice_cmd)
{
	//Set status and command by voice string
}
/***********************************
*********** 手势认人 ***************
************************************/
void getNEO(const geometry_msgs::PoseStamped &NEO_pose)
{
	flag.trackedNEO=1;
}
/***********************************
*********** 目标轨迹 ***************
************************************/
void footprint_painter(const geometry_msgs::PoseStamped &gaze_input)
{
	geometry_msgs::PoseStamped gaze=gaze_input;//目标信息
	geometry_msgs::PoseStamped base;
	base.pose.position=odom.pose.pose.position;//机器人odom信息

	if(flag.nav == 0)//follow
	{
		if(posecloud_ptr_w==posecloud_ptr_r)//轨迹点列表为空
		{
			posecloud[posecloud_ptr_w]=gaze;	
			posecloud_ptr_w++;	
			posecloud_ptr_w=posecloud_ptr_w%VOLUMN;
			posecloud_length++;
		}
		else//posecloud not empty
		{
			//目标距离前一个轨迹的距离大于0.2，则增加一个新的轨迹点
			if(get_distance(posecloud[(posecloud_ptr_w-1)%VOLUMN],gaze)>0.2)
			{
				posecloud[posecloud_ptr_w]=gaze;	
				posecloud_ptr_w++;
				posecloud_ptr_w=posecloud_ptr_w%VOLUMN;
				posecloud_length++;
			}
		}
		
		if((get_distance(base,gaze) < 2.0) || posecloud_length<1)//stop
		{
			goal.pose.position=odom.pose.pose.position;
		}	
		else //跟随
		{
			//机器人到达下一个轨迹点0.2m以内，则删除该轨迹点
			if(get_distance(base,posecloud[posecloud_ptr_r])<0.2)
			{
				posecloud_ptr_r++;	
				posecloud_ptr_r=posecloud_ptr_r%VOLUMN;
				posecloud_length--;
			}
			goal=posecloud[posecloud_ptr_r];	//更新目标点
			if(get_distance(base,gaze)>3.5)//wait me
			{
				ROS_WARN("WAIT ME !!!!");
				if(ros::Time::now()-soundplay_timer>ros::Duration(2))
				{
					sc->say("Wait Me! Wait Me!");
					ros::spinOnce();
					soundplay_timer=ros::Time::now();
				}
			}
		}
	}//normal follow
	
	goal.header.stamp=ros::Time::now();
	goal.header.frame_id="odom";
	goal.pose.orientation.x=0;
	goal.pose.orientation.y=0;
	goal.pose.orientation.z=0;
	goal.pose.orientation.w=1;
	pub0.publish(goal);

	geometry_msgs::TransformStamped goal_tf;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);    //pit,th]
    goal_tf.header.stamp = ros::Time::now();   
    goal_tf.header.frame_id = "odom";
    goal_tf.child_frame_id = "/HL_move_base_tf/goal";

	goal_tf.transform.translation.x = goal.pose.position.x;
	goal_tf.transform.translation.y = goal.pose.position.y;
	goal_tf.transform.translation.z = 0.0;
	goal_tf.transform.rotation = odom_quat;
    broadcaster->sendTransform(goal_tf);
    ROS_INFO("length:%d",posecloud_length);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "user_footprint_follow");
  ros::NodeHandle nh;
  tf::TransformListener listener0(ros::Duration(10));
  listener=&listener0; 
  tf::TransformBroadcaster broadcaster0;
  broadcaster=&broadcaster0;
  goal.header.frame_id="odom";
  goal.header.stamp=ros::Time::now();
  sound_play::SoundClient sc0;
  sc=&sc0;

  ros::Subscriber sub0 = nh.subscribe ("/HL_move_base/gaze", 1,footprint_painter);
  ros::Subscriber sub1 = nh.subscribe ("/HL_move_base/goal_dist", 1,get_goal_dist);
  ros::Subscriber sub2 = nh.subscribe ("/odom", 1,get_odom);
  ros::Subscriber sub3 = nh.subscribe ("/my_recognizer/output", 1,get_voice_cmd);
  ros::Subscriber sub4 = nh.subscribe ("/user_tracker/blindage", 1,get_blindage);
  ros::Subscriber sub5 = nh.subscribe ("/HL_move_base/gaze_theone", 1,getNEO);

  pub0 = nh.advertise<geometry_msgs::PoseStamped> ("/HL_move_base/goal", 1);

  // Spin
  soundplay_timer=ros::Time::now();
  ros::spin ();
  return 0;
}
