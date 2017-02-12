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
#include "sensor_msgs/LaserScan.h"

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_R 0x72
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6F
#define KEYCODE_P 0x70
#define KEYCODE_N 0x6E

struct termios cooked, raw;sensor_msgs::LaserScan scan;
int laser_state = 0;
int kfd=0;
int state=0;
struct people
{
	int left;
	int right;
};
enum{
	NORMAL,
	FORWARD,
	BACK,
	LEFT,
	RIGHT,
	RESET
};
people people1;

void initLaserData(sensor_msgs::LaserScan &scan)
{
	for(int i=0;i<680;i++)
	{
		scan.ranges[i]=4.3;
	}
	people1.left = 313;
	people1.right = 367;
}

void resetLaserData(sensor_msgs::LaserScan &scan)
{
	people1.left = 313;
	people1.right = 367;
	for(int i=0;i<680;i++)
	{
		scan.ranges[i]=4.3;
	}

	for(int i=people1.left;i<people1.right;i++)
	{
		scan.ranges[i]=1.7;
	} 
}

void laserDataFoward(sensor_msgs::LaserScan &scan)
{
	for(int i= people1.left;i<people1.right;i++)
	{
		scan.ranges[i] += 0.1;
	} 
}

void laserDataBack(sensor_msgs::LaserScan &scan)
{
	for(int i=people1.left ; i < people1.right;i++)
	{
		scan.ranges[i] -= 0.1;
	} 
}

void laserDataLeft(sensor_msgs::LaserScan &scan)
{
	people1.left -= 5;
	people1.right -= 5;
	people1.left = (people1.left + 680)%680;
	people1.right = (people1.right + 680)%680;
	
	float old_scan[5];
	for(int i = 0; i < 5; i++)
	{
		old_scan[i] = scan.ranges[i];
	}
	for(int i = 0; i < 675; i++)
	{
		scan.ranges[i] = scan.ranges[i + 5];
	}
	for(int i = 675; i < 680; i++)
	{
		scan.ranges[i] = old_scan[i - 675];
	}
}

void laserDataRight(sensor_msgs::LaserScan &scan)
{
	people1.left += 5;
	people1.right += 5;
	people1.left = (people1.left)%680;
	people1.right = (people1.right)%680;
	
	float old_scan[5];
	for(int i = 675; i < 680; i++)
	{
		old_scan[i - 675] = scan.ranges[i];
	}
	for(int i = 679; i >= 5; i--)
	{
		scan.ranges[i] = scan.ranges[i - 5];
	}
	for(int i = 0; i < 5; i++)
	{
		scan.ranges[i] = old_scan[i];
	}
}

void keyboardLoop()
{
    char c;    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    puts("Reading from keyboard");
    puts("W ↑ \nA ← \nS ↓\nD →\n");
    puts("R Reset people\nN No people\n");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(;;)
    {
        boost::this_thread::interruption_point();
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            continue;
        }
        
        switch(c)
        {
            case KEYCODE_W:
				state = FORWARD;
                break;
            case KEYCODE_S:
				state = BACK;
                break;
            case KEYCODE_A:
				state = LEFT;
                break;
            case KEYCODE_D:
				state = RIGHT;
                break;
            case KEYCODE_R:
				state = RESET;
                break;
            case KEYCODE_N:
				state = NORMAL;
                break;
            default:
				break;
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserscan_fake_follow");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan_user", 50);

	ros::Rate r(10.0);
	sensor_msgs::LaserScan scan;
	scan.header.stamp = ros::Time::now();
	scan.header.frame_id = "laser";
	scan.angle_min = -M_PI*2/3.0;
	scan.angle_max = M_PI*2/3.0;
	scan.angle_increment = (scan.angle_max - scan.angle_min) / 680;
	scan.time_increment = (1 / 10) / 680;
	scan.range_min = 0.0;
	scan.range_max = 5.6;
	scan.ranges.resize(680);
	
	initLaserData(scan);

	boost::thread t = boost::thread(keyboardLoop);
    printf("laserscan_fake: running\n");
     while(n.ok()){
		switch(state)
        {
            case FORWARD:
				laserDataFoward(scan);
                break;
            case BACK:
				laserDataBack(scan);
                break;
            case LEFT:	
				laserDataLeft(scan);
                break;
            case RIGHT:
				laserDataRight(scan);
                break;
            case RESET:
				resetLaserData(scan);
                break;
            case NORMAL:
				initLaserData(scan);
                break;
            default:
				break;
        }
//        if(state != 11)
//			printf("%d    %d\n",people1.left,people1.right);
        state = 11;
		scan.header.stamp = ros::Time::now();
	    scan_pub.publish(scan);
	    r.sleep();
	 }
	t.interrupt();
	t.join();
	tcsetattr(kfd, TCSANOW, &cooked);
	printf("laserscan_fake: stopped\n");
	return 0;
}
