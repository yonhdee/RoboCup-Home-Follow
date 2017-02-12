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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>

#include <user_tracker_msgs/UserPoseCloud.h>
#include <user_tracker_msgs/Blindage.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#define DEBUG

ros::Publisher pub;
ros::Publisher pub2;
geometry_msgs::PoseStamped gaze;
geometry_msgs::Point gaze_blind;
tf::TransformListener *listener;
tf::TransformBroadcaster *broadcaster;
ros::Time timer1;

char userFound=0;

//检测人体
int human_detect(const sensor_msgs::LaserScan &laser , int user_pose, int user_width,float min_fat,float max_fat, double &range)
{
	range=(laser.ranges[user_pose-2]+laser.ranges[user_pose-1]+laser.ranges[user_pose]+laser.ranges[user_pose+1]+laser.ranges[user_pose+2])/5.0;
	float fat= 2*tan(laser.angle_increment*user_width/2)*range;
	if(fat>min_fat&&fat<max_fat)
		return 1;
	else
		return 0;
}
//距离最近的物体的坐标
void get_blindage(const user_tracker_msgs::Blindage &blindage)
{
	double range=blindage.range;
	double ang=blindage.angle;
	gaze_blind.x=range*cos(ang)-cos(M_PI/3-ang);
	gaze_blind.y=range*sin(ang)+sin(M_PI/3-ang);
	gaze_blind.z=0;
}
//重新找人 //node:user_trackNEO
void getNEO(const geometry_msgs::PoseStamped &NEO_pose)
{
	gaze=NEO_pose;
}
void lasertracker(const sensor_msgs::LaserScan &input)
{
	#ifdef DEBUGx
		printf("%f|%f|%f|%f|%f|%f|%f|%d\n",input.angle_min,
		input.angle_max,
		input.angle_increment,
		input.time_increment,
		input.scan_time,
		input.range_min,
		input.range_max,
		input.ranges.size());
	#endif

	int size=input.ranges.size();
	int size_d2=size/2;
	float range_diff[size-5];
	int user_pose[size];
	int user_width[size];
	int user_count=0;
    sensor_msgs::LaserScan laser=input;

    //大于5.4的激光信息改为5.4
    for(int i=0;i<size;i++)
	{
		if(laser.ranges[i]>5.4)
		{
			laser.ranges[i]=5.4;
		}
		if(isnan(laser.ranges[i]))
			laser.ranges[i]=5.4;
	}

	laser.intensities.resize(size);

	for(int i=0;i<size-1;i++)
	{
		range_diff[i]=laser.ranges[i+1]-laser.ranges[i];
		//laser.intensities[i]=(range_diff[i]+5.6)/11.2;
	}

	if(userFound)
	{
		geometry_msgs::PointStamped source_point;
		geometry_msgs::PointStamped object_point;

		try
		{
			source_point.header.frame_id="odom";
			source_point.header.stamp=ros::Time();
			source_point.point.x=gaze.pose.position.x;
			source_point.point.y=gaze.pose.position.y;
			source_point.point.z=0;
			listener->transformPoint("base_footprint", source_point, object_point);


			#ifdef DEBUGx
				ROS_INFO("source_point: (%.2f, %.2f. %.2f) -----> object_point: (%.2f, %.2f, %.2f) at time %.2f",
				source_point.point.x, source_point.point.y, source_point.point.z,
				object_point.point.x, object_point.point.y, object_point.point.z, object_point.header.stamp.toSec());
			#endif
		}

		catch(tf::TransformException& ex)
		{
			ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
			return;
		}

		double range=sqrt(object_point.point.y*object_point.point.y+object_point.point.x*object_point.point.x);
		double ang=atan2(object_point.point.y,object_point.point.x);
		int pose=ang/laser.angle_increment+340;
		int edge_l=pose-40,edge_r=pose+40;

		for(int j=0;j<40;j++)
		{
			if(range_diff[pose-j]>0.2||range_diff[pose-j]<-0.2)
			{
				edge_l=pose-j;
				break;
			}
		}

		for(int k=1;k<=40;k++)
		{
			if(range_diff[pose+k]>0.2||range_diff[pose+k]<-0.2)
			{
				edge_r=pose+k;
				break;
			}
		}

		int pose_new=(edge_l+edge_r)/2;
		laser.intensities[edge_l]=0.25;
		laser.intensities[edge_r]=0.25;
		laser.intensities[pose_new]=0.75;
		int width_new=edge_r-edge_l;
		double range_new;

		if(pose_new>size_d2-170&&pose_new<size_d2+170)//寻找范围±60°
		{
			//if()//
			int ishuman=human_detect(laser, pose_new, width_new,0.05,0.9,range_new);//human_detect(激光,user位置,宽度,最小宽度,最大宽度,ran)
			//{
			//ROS_INFO("track you!!!");//%d||%d||%d||%d",pose,edge_l,edge_r,pose_new);
			//gaze.header=input.header;
			try
			{
				double ang=laser.angle_increment*(pose_new-340);

				source_point.header.frame_id="base_footprint";
				source_point.header.stamp=ros::Time();
				source_point.point.x=range_new*cos(ang);
				source_point.point.y=range_new*sin(ang);
				source_point.point.z=0;
				listener->transformPoint("odom", source_point, object_point);

				#ifdef DEBUGx
					ROS_INFO("source_point: (%.2f, %.2f. %.2f) -----> object_point: (%.2f, %.2f, %.2f) at time %.2f",
					source_point.point.x, source_point.point.y, source_point.point.z,
					object_point.point.x, object_point.point.y, object_point.point.z, object_point.header.stamp.toSec());
				#endif
			}

			catch(tf::TransformException& ex)
			{
				ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
				return;
			}

			double error_x=object_point.point.x-gaze.pose.position.x;
			double error_y=object_point.point.y-gaze.pose.position.y;
			double error_mo=sqrt(error_x*error_x+error_y*error_y);

			if(error_mo<0.5&&ishuman)
			{
				gaze.pose.position.x=object_point.point.x;
				gaze.pose.position.y=object_point.point.y;
				gaze.pose.position.z=0;
				//ROS_INFO("track you!!!%7.4lf||%7.4lf||%7.4lf||%7.4lf||%7.4lf",error_x,error_y,error_mo,range_diff[edge_l],range_diff[edge_r]);
				timer1=ros::Time::now();
			}

			else
			{
				if(ros::Time::now()-timer1>ros::Duration(6))
				{
					userFound=0;
					//ROS_INFO("lost!!!");
				}
					//ROS_INFO("not the one!!!%2.4lf||%2.4lf||%2.4lf",error_x,error_y,error_mo);
			}
			//}
			//else not human
			//	{
			//		userFound=0;
			//		ROS_INFO("not human!!!");
			//	}
		}

		else //not in place
		{
			userFound=0;
			//ROS_INFO("not in place!!!%d||%d||%d||%d",pose,edge_l,edge_r,pose_new);
		}

	}//userFound==1

	else
	{
		char user_scan_flag=0;
		int edge_down,edge_up;
		for(int i=0;i<size-1;i++)
		{
			switch(user_scan_flag)
			{
				case 0://none
					if(range_diff[i]<-0.5)
					{
						user_scan_flag=1;
						edge_down=i;
					}
					break;
				case 1://got down edge
					if(range_diff[i]<-0.5)
					{
						user_scan_flag=1;
						edge_down=i;
					}
					if(range_diff[i]>0.5)
					{
						user_scan_flag=0;
						edge_up=i;
						user_pose[user_count]=(edge_down+edge_up)/2;
						user_width[user_count]=edge_up-edge_down;
						user_count++;
					}
					break;
				case 2://got up edge
					break;
			}
		}//for

		for(int i=0;i<user_count;i++)
		{
			double range;

			if(user_pose[i]>size_d2-40&&user_pose[i]<size_d2+40)
			if(human_detect(laser, user_pose[i], user_width[i],0.18,0.6,range))
			{
				//ROS_INFO("found you!!!");
				//gaze.header=input.header;
				geometry_msgs::PointStamped source_point;
				geometry_msgs::PointStamped object_point;

				try
				{
					double ang=laser.angle_increment*(user_pose[i]-340);

					source_point.header.frame_id="base_footprint";
					source_point.header.stamp=ros::Time();
					source_point.point.x=range*cos(ang);
					source_point.point.y=range*sin(ang);
					source_point.point.z=0;
					listener->transformPoint("odom", source_point, object_point);

					#ifdef DEBUG
						ROS_INFO("source_point: (%.2f, %.2f. %.2f) -----> object_point: (%.2f, %.2f, %.2f) at time %.2f",
						source_point.point.x, source_point.point.y, source_point.point.z,
						object_point.point.x, object_point.point.y, object_point.point.z, object_point.header.stamp.toSec());
					#endif
				}

				catch(tf::TransformException& ex)
				{
					ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
					return;
				}
				gaze.pose.position.x=object_point.point.x;
				gaze.pose.position.y=object_point.point.y;
				gaze.pose.position.z=0;
				userFound=1;
				timer1=ros::Time::now();
			}

			//else
			//ROS_INFO("not human!!!");
			//ROS_INFO("not in place!!!");
		}//for

	}//userFound==0

	////////////////////////////
	gaze.header=input.header;
	gaze.header.frame_id="odom";
	gaze.pose.orientation.x=0;
	gaze.pose.orientation.y=0;
	gaze.pose.orientation.z=0;
	gaze.pose.orientation.w=1;
	pub.publish(gaze);
	geometry_msgs::TransformStamped gaze_tf;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);    //pit,th]
    gaze_tf.header.stamp = ros::Time::now();
    gaze_tf.header.frame_id = "odom";
    gaze_tf.child_frame_id = "/HL_move_base_tf/gaze";

    gaze_tf.transform.translation.x = gaze.pose.position.x;
    gaze_tf.transform.translation.y = gaze.pose.position.y;
    gaze_tf.transform.translation.z = 0.0;
    gaze_tf.transform.rotation = odom_quat;
	broadcaster->sendTransform(gaze_tf);
	pub2.publish(laser);

}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "user_tracker_laser");
	ros::NodeHandle nh;
	tf::TransformListener listener0(ros::Duration(10));
	listener=&listener0;
	tf::TransformBroadcaster broadcaster0;
	broadcaster=&broadcaster0;
	gaze.header.frame_id="odom";
	gaze.header.stamp=ros::Time::now();
	//std::cout<<dotransform(Eigen::Vector3f (1,1,2),Eigen::Vector3f (1.0,0.0,1.0))<<std::endl;
	// Create a ROS subscriber for the input point cloud
	//ros::spin ();
	ros::Subscriber sub0 = nh.subscribe ("/scan_user", 1,lasertracker);
	ros::Subscriber sub1 = nh.subscribe ("/user_tracker/blindage", 1,get_blindage);
	ros::Subscriber sub2 = nh.subscribe ("/HL_move_base/gaze_theone", 1,getNEO);

	pub = nh.advertise<geometry_msgs::PoseStamped> ("/HL_move_base/gaze", 1);
	pub2 = nh.advertise<sensor_msgs::LaserScan> ("lasertest", 1);
	// Spin
	ros::spin ();
	return 0;

}
