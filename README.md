# UR3 simulation - ERC 2023

The repository contains a simulation of a UR3 robot, prepared for training before the ERC 2022 competition. The simulation uses a UR3 arm model with a repository [ROS-Industrial Universal Robot repository] (https://github.com/ros-industrial/universal_robot). Cell model, IMU box, box with lid, buttons and a gripper with camera have been added. The gripper also uses a gazebo plugin to create mimic joints ([roboticsgroup_gazebo_plugins] (https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins)). The files necessary for motion planning have also been modified to take into account the added elements. All changed and added files are located in this repository.

The gripper used was taken from the [Printables website](https://www.printables.com/pl/model/165722-robotic-gripper?fbclid=IwAR14jhZnuyvgtlFUA-Fm_h8lI08LHBNPQ0fYph930b-ZGCJd9EBcdEk6IPQ).

## UPDATE!!!
A block has been added to the gripper in the place where the plugs are. This will avoid collisions with plugs.

## Install on the host system

### Requirements

The simulation is mainly developed and tested on [Ubuntu 20.04 Focal Fossa](https://releases.ubuntu.com/20.04/) with [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu), so it is a recommended setup.

For the simulation to work properly, you must install dependencies and download repositories by running the following commands:
```
source /opt/ros/noetic/setup.bash
sudo apt-get update && apt-get upgrade -y && apt-get install -y lsb-core g++
sudo apt-get install git
rosdep init && rosdep update
sudo apt install ros-noetic-moveit -y
sudo apt install ros-noetic-ros-controllers* -y
mkdir -p /catkin_ws/src
cd /catkin_ws/src
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
sudo rm -r universal_robot/ur_msgs
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
git clone https://github.com/Michal-Bidzinski/UR3_sim.git
cd /catkin_ws
catkin_make
source devel/setup.bash
```
### Run simulation
To run UR3 simulation in Gazebo with MoveIt!, and RVzi GUI, including an example cell:
```
$ roslaunch ur3_sim simulation.launch
```
To run UR3 simulation in Gazebo with MoveIt!, and RVzi GUI, containing only a robot with a grapple and a camera (as per real setup):
```
$ roslaunch ur3_sim real_station.launch
```

### Control the robot
To control the arm can by used MoveIt!. Planning robot movement can be performed using for example the Move Group Interface. Tutorials written in [Python](https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py) and [C++](https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp) can be a hint (note the robot group name and number of joints).


### Control the gripper
The gripper is controlled by publishing appropriate commands on topic /gripper command. Message type: std msgs/String. To control the gripper, send the one of following commands:
- open
- semi_open (for catching the IMU box)
- semi_close (for catching the lid of the box)
- close

### Camera
The simulation includes a camera placed on a gripper that detects aruco tags and other items.
The following topics are published:
- /camera\_image/camera\_info
- /camera\_image/image\_raw
- /camera\_image/image\_raw/compressed
- /camera\_image/image\_raw/compressed/parameter\_descriptions
- /camera\_image/image\_raw/compressed/parameter\_updates
- /camera\_image/image\_raw/compressedDepth
- /camera\_image/image\_raw/compressedDepth/parameter\_descriptions
- /camera\_image/image\_raw/compressedDepth/parameter\_updates
- /camera\_image/image\_raw/theora
- /camera\_image/image\_raw/theora/parameter\_descriptions
- /camera\_image/image\_raw/theora/parameter\_updates

Although the topics with the word Depth in the name are published by defalut ros node,
these cameras do not capture any depth data.
