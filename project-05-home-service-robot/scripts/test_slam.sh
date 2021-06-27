#!/bin/sh

xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../src/my_robot/worlds/rothl.world.sdf" &
#xterm  -e  "roslaunch my_robot world.launch" &
sleep 5
            
xterm  -e  "
 rosparam set /slam_gmapping/iterations 100;
 rosparam set /slam_gmapping/linearUpdate 0.05;
 rosparam set /slam_gmapping/angularUpdate 0.05;
 rosparam set /slam_gmapping/map_update_interval 0.1;
 rosparam set /slam_gmapping/srr 0.01;
 rosparam set /slam_gmapping/srt 0.01;
 rosparam set /slam_gmapping/str 0.02;
 rosparam set /slam_gmapping/stt 0.02;
 roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

xterm  -e  "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

