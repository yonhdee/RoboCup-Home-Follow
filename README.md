# RoboCup-Home-Follow

### RoboCup@Home 比赛项目的 Follow ROS包

Follow 的ROS包不依赖任何库，但运行需要ROS的基本环境。

- 编译

	```bash
	cd workspace
	catkin_make --pkg user_tracker_msgs
	source devel/setup.bash
	catkin_make
	```

- 运行

	运行一个仿真实例：

	```bash
	roslaunch bringup follow_simulation.launch
	roslaunch bringup rviz.launch
	```

	说明：

	- 程序启动后在`follow_simulation.launch`终端内按`R`键，给定被跟随者的位置
	- 在`follow_simulation.launch`终端内按`S、W、A、D`键会改变模拟器中激光状态，模拟运动的人的位置。*按W使人由静止变为移动，6秒后开始跟随*
	- 在`follow_simulation.launch`终端内按`R`键，重置人的位置
	- 在`follow_simulation.launch`终端内按`N`键，清除人
	- 机器人底盘的数据由 robot_fake 节点模拟，人的距离过远或者人突然以很快的速度跑开会造成机器人跟丢，此时机器人会发出 “Wait me” 的声音，人走回原位即可重新锁定。
