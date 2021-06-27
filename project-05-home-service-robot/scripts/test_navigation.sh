#!/bin/sh

#xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../src/my_robot/worlds/rothl.world.sdf" &
sleep 5
 
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../src/map/new2/map.yaml " &
sleep 5

xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5



