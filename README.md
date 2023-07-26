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

**[IMPORTANT] The following instructions must be completed before installing this package**

Install the following device and ros drivers: https://github.com/jhu-saw/sawSensablePhantom

Download this repository into the same catkin_ws/src folder used above.

Use `catkin build` to compile everything

## How to Run

To run any of the scripts in ROS:

```Shell
rosrun teleop_core <file-name>.py
```
### Notes
This package is still under development.