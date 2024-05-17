# teleop_core

This is a ROS package for Teleoperation capabilities using the 3D Systems Touch Haptic Device.

Dependency: [jhu-saw/sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom) 

## Tested Environment
|Software| Version|
|:---:|:---:|
|Operating System          |     Ubuntu 18.04 / 20.04|
|ROS Distro                |     Melodic      / Noetic |
|3DS Touch connection      |     USB           |
|3DS Touch Driver          |     Version 2019  / 2022|
|Open Haptics              |     Version 3.4   |
---

## Using this package

This package is meant to be the hub for sending teleoperation commands to different systems. 

The scripts in this package rely on messages from the follower robots, to compute and relay back messages to the same robot(s).

**UVMS:** For bimanual teleoperation of Underwater Vehicle Manipulator System (RexROV) - *[Paper Accepted for Publication](https://arxiv.org/abs/2404.03790)*

**Franka Research 3:** For teleoperation of Franka Research 3 (panda) mounted on a Husky UGV, for mobile-manipulation research activities

**xArm7:** For teleoperation of xArm7 robot for minimally invasive surgery research activities

### Disclaimer

More additions continue to be made to this package.

## Publications
The following are publications by the ARMLAB that utilize this teleoperation package:

1. Sitler, J., Sowrirajan, S., and Wang, L. (2024). "A Bimanual Teleoperation Framework for Light Duty Underwater Vehicle-Manipulator Systems." Accepted to *21st International Conference on Ubiquitous Robots* ([Preprint](https://arxiv.org/abs/2404.03790))

<!-- **Required: Install the following ROS packages** 
1. [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
2. [universal_robot](https://github.com/ros-industrial/universal_robot)

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
``` -->