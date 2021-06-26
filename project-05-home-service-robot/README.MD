# Home Service Robot

<p align="center">
  <img src="https://github.com/miharothl/lab-robotics/blob/master/project-05-home-service-robot/images/home-service.gif?raw=true" width="800" />
</p>


## Prerequisites

* [ROS Development Rig](https://github.com/miharothl/nvidia-docker-novnc)
* [Git Large File Storage](https://git-lfs.github.com/)

## Repository and Dependencies

Initialize sub-modules. 

``` bash
git submodule init
git submodule update
```

Install dependencies.
```
cd ~/project-05-home-service-robot
catkin_make
sudo apt-get update

source devel/setup.bash
rosdep -i install gmapping
rosdep -i install turtlebot_teleop
rosdep -i install turtlebot_rviz_launchers
rosdep -i install turtlebot_gazebo

catkin_make
source devel/setup.bash
```

In this project I used [turtlebot](https://www.turtlebot.com/).

![BOT](https://github.com/miharothl/lab-robotics/blob/master/project-05-home-service-robot/images/turtlebot.png?raw=true)

## Launch Scripts

Prepare environment in terminal 1:

``` bash
cd ~/project-05-home-service-robot
catkin_make
source devel/setup.bash
```

### Run Tests

``` bash
# Test Launch Script
./scripts/test_launch.sh

# Test SLAM
./scripts/test_slam.sh

# Test Virtual Objects
./scripts/test_add_marker.sh

# Test Navigational Goal
./scripts/test_pick_objects.sh
``` 

### Run Home Service Robot Demo

``` bash
./scripts/home_service.sh
```

## Map

While testing the SLAM save the map and use it later during navigation.

``` bash
cd ~/project-05-home-service-robot/src/map
rosrun map_server map_saver
```

![MAP](https://github.com/miharothl/lab-robotics/blob/master/project-05-home-service-robot/images/map.png?raw=true)

## Packages

### Mapping & Localization 

[Gmapping](http://wiki.ros.org/gmapping) package used to map and localize the robot in the environment. The gmapping package provides laser-based SLAM
(Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create
a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

### Navigation

Robot is navigating the environment using the [ROS Navigation Stack](http://wiki.ros.org/navigation), which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm.
Using the algorithm the robot plans a trajectory from current to target position. The ROS navigation stack permits the robot to avoid any obstacle on its path
by re-planning a new trajectory once your robot encounters them. 

### Home Service

Turtle bot in the environment.

![WORLD](https://github.com/miharothl/lab-robotics/blob/master/project-05-home-service-robot/images/world.png?raw=true)

SLAM and navigation in action.

![SLAM](https://github.com/miharothl/lab-robotics/blob/master/project-05-home-service-robot/images/slam-and-navigation.png?raw=true)
