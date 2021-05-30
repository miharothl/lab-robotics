# Map My World

## Prerequisites

* [ROS Development Rig](https://github.com/miharothl/nvidia-docker-novnc)
* [Git Large File Storage] (https://git-lfs.github.com/)

## Clone Repo

Initialize sub-modules. 

``` bash
git submodule init
git submodule update
```

## Install Rtabmap

Follow https://github.com/introlab/rtabmap_ros#build-from-source

## Launching

Start Gazebo and RViz in terminal 1:

``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch
```

Start SLAM mapping in  terminal 2:
``` bash
source devel/setup.bash
roslaunch my_robot mappling.launch
```

Start teleop in terminal 3:
``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot teleop.launch
```

Check SLAM mapping database
``` bash
rtabmap-databaseViewer db/rtabmap.db 
```

## Localization

Environment has to have clear features and geometric shapes to perform the mapping.
![SLAM 1](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/the-world.png?raw=true)

Robot, explores the world, SLAM in action.
![SLAM 2](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/slam-in-action.png?raw=true)

Explore the mapping database.
![SLAM 3](https://github.com/miharothl/lab-robotics/blob/master/project-04-map-my-world/images/slam-features.png?raw=true)

