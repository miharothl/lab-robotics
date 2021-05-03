# Where Am I

## Prerequisites

* [ROS Development Rig](https://github.com/miharothl/nvidia-docker-novnc)

## Clone Repo

Initialize sub-modules.

``` bash
git submodule init
submodule update
```

## Launching

Terminal 1:

``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch
```

Terminal 2:
``` bash
catkin_make
source devel/setup.bash
roslaunch my_robot amcl.launch
```

Terminal 3:
``` bash
catkin_make
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Localization

Kidnapped Robot

[Kidnapped Robot Step 1](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_1.png?raw=true)
[Kidnapped Robot Step 2](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_2.png?raw=true)
[Kidnapped Robot Step 3](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_3.png?raw=true)
[Kidnapped Robot Step 4](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_4.png?raw=true)
[Kidnapped Robot Step 5](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_5.png?raw=true)


