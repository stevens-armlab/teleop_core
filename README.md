# teleop_core

This is a ROS package for Teleoperation capabilities using the 3D Systems Touch Haptic Device.

This package must be installed in the same workspace as the 3DS Touch Haptic Device. 

## Tested Environment
|Software| Version|
|:---:|:---:|
|Operating System          |     Ubuntu 18.04  |
|ROS Distro                |     Melodic       |
|3DS Touch connection      |     USB           |
|3DS Touch Driver          |     Version 2019  |
|Open Haptics              |     Version 3.4   |
---

**Required: Install the following ROS packages**
1. [jhu-saw/sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom)
2. [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
3. [universal_robot](https://github.com/ros-industrial/universal_robot)

Clone these repositories into your ros workspace src/ directory.

Use `catkin build` to compile everything

## How to use this codebase
### 0. Launch UR5 in Gazebo
In all new terminal windows, source ROS + your workspace

In a new ternimal:
```
roscore
```
In a new ternimal:
```
roslaunch ur_gazebo ur5_bringup.launch
```
This will launch a gazebo window with the UR5 robot

### 1. follow_traj
This ROS Node will move a gazebo simulation of the UR5 using a configuration file consisting of the joint position trajectory of the robot.

[Trajectory Generation](https://github.com/stevens-armlab/teleop_python_utils) : The linked repo has instructions to generate a configuration file that is needed to use this ROS Node. This file must be saved in [config/](config/)

A sample file called ```traj1.yaml``` is provided.
```Shell
rosrun teleop_core follow_traj.py --config traj1
```
### Notes
This package is still under development.