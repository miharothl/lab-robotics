# Where Am I

Kidnapped robot
![The Robot](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/the_robot_1.png?raw=true)


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

Kidnapped robot.
![Kidnapped Robot Step 1](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_1.png?raw=true)

Kidnapped robot - step 2 
![Kidnapped Robot Step 2](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_2.png?raw=true)

Kidnapped robot - step 3 
![Kidnapped Robot Step 3](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_3.png?raw=true)


Kidnapped robot - step 4 
![Kidnapped Robot Step 4](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_4.png?raw=true)


Kidnapped robot is localized.
![Kidnapped Robot Step 5](https://github.com/miharothl/lab-robotics/blob/master/project-03-where-am-i/images/kidnapped_robot_5.png?raw=true)


