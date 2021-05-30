# Where Am I

![The Robot](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/the_robot.png?raw=true)

## Prerequisites

* [ROS Development Rig](https://github.com/miharothl/nvidia-docker-novnc)

## Clone Repo

Initialize sub-modules.

``` bash
git submodule init
git submodule update
```

## Launching

Start Gazebo and RViz in terminal 1:

``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch
```

Start localization in  terminal 2:
``` bash
source devel/setup.bash
roslaunch my_robot amcl.launch
```

Start teleop in terminal 3:
``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot teleop.launch
```



rtabmap-databaseViewer db/rtabmap.db 

## Localization

Robot is kidnapped. Location is unknown.
![Kidnapped Robot Step 1](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_1.png?raw=true)

Kidnapped robot, explores. Step 1...
![Kidnapped Robot Step 2](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_2.png?raw=true)

Kidnapped robot, explores. Step 2...
![Kidnapped Robot Step 3](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_3.png?raw=true)

Kidnapped robot, explores. Step 3...
![Kidnapped Robot Step 4](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_4.png?raw=true)

Kidnapped robot is localized.
![Kidnapped Robot Step 5](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_5.png?raw=true)
