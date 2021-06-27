#!/bin/sh

#xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/../src/my_robot/worlds/rothl.world.sdf" &
sleep 5
 
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../src/map/new/map.yaml " &
sleep 5

#xterm  -e  "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm  -e  "roslaunch add_markers add_markers.launch rviz_config_file:=$(pwd)/../src/rvizConfig/home_service.rviz;" &
sleep 5

xterm  -e  "rosparam load $(pwd)/../src/config/home_service.yaml;
            rosrun add_markers add_markers_test" &
sleep 5
