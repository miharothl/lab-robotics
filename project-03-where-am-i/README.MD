# Where Am I

## Prerequisites

* ROS Development Rig

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

( )


