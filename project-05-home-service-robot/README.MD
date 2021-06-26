# Home Service Robot

## Prerequisites

* [ROS Development Rig](https://github.com/miharothl/nvidia-docker-novnc)
* [Git Large File Storage](https://git-lfs.github.com/)

## Repo and Dependencies

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

## Launch

In terminal 1:

``` bash
cd ~/project-05-home-service-robot
catkin_make
source devel/setup.bash
```

### Test Shell Script

``` bash
./scripts/test_launch.sh
```

### Test SLAM

```
./scripts/test_slam.sh
```

### Test Virtual Objects

``` bash
./scripts/test_add_marker.sh
```

### Test Navigational Goal

``` bash
./scripts/test_pick_objects.sh
```

### Home Service Robot Demo

``` bash
./scripts/home_service.sh
```

## Packages

### Mapping & Localization 

[Gmapping](http://wiki.ros.org/gmapping) package used to map and localize the robot in the environment. The gmapping package provides laser-based SLAM
(Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create
a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

### Navigation

Robot is navigating the environment using the ROS Navigation Stack, which is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm.
Using the algorithm the robot plans a trajectory from current to target position. The ROS navigation stack permits the robot to avoid any obstacle on its path
by re-planning a new trajectory once your robot encounters them. 

## Localization

Environment has to have clear features and geometric shapes to perform the mapping.
![SLAM 1](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/the-world.png?raw=true)

Robot, explores the world, SLAM in action.
![SLAM 2](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/slam-in-action.png?raw=true)

Explore the mapping database.
![SLAM 3](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/slam-features.png?raw=true)

