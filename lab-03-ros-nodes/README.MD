# ROS Nodes Lab

## Pull Repository and Start roslaunch 

In terminal 1: pull repo and start roslaunch

```bash
cd ~/scm/r/hub

git clone https://github.com/miharothl/lab-robotics.git
cd lab-robotics
git submodule init
git submodule update

cd lab-03-ros-nodes
catkin_make
source devel/setup.bash
roslaunch simple_arm robot_spawn.launch
```

## Start Camera Viewer

In terminal 2.

```bash
cd scm/r/hub/lab-robotics/lab-03-ros-nodes/
source devel/setup.bash
rqt_image_view /rgb_camera/image_raw
```

## Call Service

In terminal 3.

```bash
cd scm/r/hub/lab-robotics/lab-03-ros-nodes/
source devel/setup.bash
rosservice call /arm_mover/safe_move "joint_1: 0
joint_2: 0"
```
