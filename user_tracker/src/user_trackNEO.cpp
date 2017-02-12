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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16.h>

#include <user_tracker_msgs/UserPoseCloud.h>

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#define DEBUG
ros::Publisher pub;

geometry_msgs::PoseStamped gaze;
geometry_msgs::PoseStamped gaze_temp;
tf::TransformListener *listener;
tf::TransformBroadcaster *broadcaster;
ros::Time timer1;

int gaze_count;
int state=0;

double get_distance(geometry_msgs::PoseStamped & pose1, geometry_msgs::PoseStamped & pose2)
{
double x=pose1.pose.position.x-pose2.pose.position.x;
double y=pose1.pose.position.y-pose2.pose.position.y;
return sqrt(x*x+y*y);
}

void get_state(const std_msgs::Int16 &state_input)
{
state=state_input.data;
}
char userFound=0;
int human_detect(const sensor_msgs::LaserScan &laser , int user_pose, int user_width,float s_fat,float l_fat, double &range)//级联分类器
{
int hand_count_inner=0;
int hand_count_outer=0;
int hand_flag=0;
double hand_fat_inner=0;
double hand_fat_outer=0;

//param
float s_fat_hand_inner=0.08;
float l_fat_hand_inner=0.48;
float s_fat_hand_outer=0.026;
float l_fat_hand_outer=0.18;
float hand_error_i=0.4;
float hand_error_o=0.15;
//param

range=(laser.ranges[user_pose-2]+laser.ranges[user_pose-1]+laser.ranges[user_pose]+laser.ranges[user_pose+1]+laser.ranges[user_pose+2])/5.0;

float fat= 2*tan(laser.angle_increment*user_width/2)*range;
if(fat<s_fat||fat>l_fat)//is human?
return 0;//not human
	for(int i=user_width/2;i<user_width*2;i++)//left hand check
	{
		switch(hand_flag)
		{
			case 0:
				if((laser.ranges[user_pose-i]-range-0.15)>hand_error_i||(laser.ranges[user_pose-i]-range-0.15)<-hand_error_i)
				{
					hand_count_inner++;
				}
				else
				{
					hand_count_outer++;
					hand_flag=1;
				}
			break;
			case 1:
				if((laser.ranges[user_pose-i]-range-0.15)<hand_error_o&&(laser.ranges[user_pose-i]-range-0.15)>-hand_error_o)
				{
					hand_count_outer++;
				}
				else
				{
					hand_flag=2;
				}
			break;

		}
	}
	hand_fat_inner= 2*tan(laser.angle_increment*hand_count_inner/2)*range;
	hand_fat_outer= 2*tan(laser.angle_increment*hand_count_outer/2)*range;
	//printf("%f||%f\n",hand_fat_inner,hand_fat_outer);
	if(hand_fat_inner<s_fat_hand_inner||hand_fat_inner>l_fat_hand_inner)
	return 0;//not left hand inner
	if(hand_fat_outer<s_fat_hand_outer||hand_fat_outer>l_fat_hand_outer)
	return 0;//not left hand
//reinit
	hand_count_inner=0;
	hand_count_outer=0;
	hand_flag=0;
	for(int i=user_width/2+1;i<user_width*2;i++)//right hand check
	{
		switch(hand_flag)
		{
			case 0:
				if((laser.ranges[user_pose+i]-range-0.15)>hand_error_i||(laser.ranges[user_pose+i]-range-0.15)<-hand_error_i)
				{
					hand_count_inner++;
				}
				else
				{
					hand_count_outer++;
					hand_flag=1;
				}
			break;
			case 1:
				if((laser.ranges[user_pose+i]-range-0.15)<hand_error_o&&(laser.ranges[user_pose+i]-range-0.15)>-hand_error_o)
				{
					hand_count_outer++;
				}
				else
				{
					hand_flag=2;
				}
			break;

		}
	}
	hand_fat_inner= 2*tan(laser.angle_increment*hand_count_inner/2)*range;
	hand_fat_outer= 2*tan(laser.angle_increment*hand_count_outer/2)*range;
	//printf("%f||%f\n",hand_fat_inner,hand_fat_outer);
	if(hand_fat_inner<s_fat_hand_inner||hand_fat_inner>l_fat_hand_inner)
	return 0;//not left hand inner
	if(hand_fat_outer<s_fat_hand_outer||hand_fat_outer>l_fat_hand_outer)
	return 0;//not left hand


return 1;//found NEO!!

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

    //laser.ranges.resize(input.ranges.size());
    for(int i=0;i<size;i++)
		{
			if(laser.ranges[i]>5.4)
			{
				laser.ranges[i]=5.4;
			}
			if(isnan(laser.ranges[i]))
			laser.ranges[i]=5.4;
			//else
			//laser.ranges[i]=input.ranges[i];
		}
	laser.intensities.resize(size);

	for(int i=0;i<size-1;i++)
		{
			range_diff[i]=laser.ranges[i+1]-laser.ranges[i];
			//laser.intensities[i]=(range_diff[i]+5.6)/11.2;
		}
if(state==1)
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
			//if(user_pose[i]>size_d2-40&&user_pose[i]<size_d2+40)
				if(human_detect(laser, user_pose[i], user_width[i],0.18,0.6,range))
				{
					ROS_INFO("found you!!!");
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
						geometry_msgs::PoseStamped obpose;
						obpose.pose.position=object_point.point;
						if(get_distance(gaze_temp,obpose)<0.15)
						{
							if(gaze_count<50)
							{
							gaze_count++;
							}
						}
						else
						{
							gaze_count=0;
						}
						gaze_temp.pose.position=object_point.point;
						userFound=1;
						timer1=ros::Time::now();
				}
	//	else
			//	ROS_INFO("not human!!!");
			//else
			//ROS_INFO("not in place!!!");

		}//for
	printf("%d\n",gaze_count);
	if(gaze_count>6)
	{
	gaze.pose.position=gaze_temp.pose.position;
	gaze.header=input.header;
	gaze.header.frame_id="odom";
	pub.publish(gaze);
	geometry_msgs::TransformStamped gaze_tf;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);    //pit,th]
        gaze_tf.header.stamp = ros::Time::now();
	    gaze_tf.header.frame_id = "odom";
	    gaze_tf.child_frame_id = "/HL_move_base_tf/gaze_theone";

	        gaze_tf.transform.translation.x = gaze.pose.position.x;
	        gaze_tf.transform.translation.y = gaze.pose.position.y;
	        gaze_tf.transform.translation.z = 0.0;
        gaze_tf.transform.rotation = odom_quat;
	broadcaster->sendTransform(gaze_tf);
	}
}
//pub2.publish(laser);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "user_trackNEO");
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
  ros::Subscriber sub1 = nh.subscribe ("/user_tracker/state", 1,get_state);
  pub = nh.advertise<geometry_msgs::PoseStamped> ("/HL_move_base/gaze_theone", 1);
  //pub2 = nh.advertise<sensor_msgs::LaserScan> ("lasertest", 1);
  // Spin
  ros::spin ();
return 0;
}
