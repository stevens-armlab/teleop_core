# teleop_core

ROS 2 Jazzy package for teleoperation using the 3D Systems Touch haptic device.

It is the hub that turns Touch stylus motion into robot commands. While the
clutch button is held, each node maps the stylus motion since the press
(the "anchor") onto the robot pose captured at the same moment. It also applies
viewer/robot-base frame rotations and scaling.

## Tested Environment
| Software | Version |
|:---:|:---:|
| Operating System   | Ubuntu 24.04 |
| ROS Distro         | Jazzy |
| 3DS Touch connection | USB |
| 3DS Touch Driver   | 2025_12_10 |
| OpenHaptics        | 3.4 |

## Dependencies

- A ROS 2 driver for the Touch that publishes the
  [CRTK](https://github.com/collaborative-robotics/crtk-docs)-style topics below, e.g.
  [jhu-saw/sawSensablePhantom](https://github.com/jhu-saw/sawSensablePhantom) (ROS 2 build).
  With the default single-device config, the device is named `arm`:

  | Topic | Type |
  |---|---|
  | `/arm/measured_cp` | `geometry_msgs/PoseStamped` |
  | `/arm/button1`, `/arm/button2` | `sensor_msgs/Joy` (`buttons[0]` = pressed) |

  The bimanual node expects two devices named `left` and `right`.
- `rclpy`, `PyKDL` (`python_orocos_kdl_vendor`), `numpy`. Install with rosdep:

```bash
cd ~/research/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/teleop_core --ignore-src -y
```

## Build

```bash
cd ~/research/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_core
source install/setup.bash
colcon test --packages-select teleop_core && colcon test-result --verbose
```

> If you use pyenv, make sure `/usr/bin/python3` (the Jazzy interpreter) builds
> and runs the package. A pyenv interpreter cannot import `rclpy` or `PyKDL`.

## Nodes

All topic names are parameters (or can be remapped with `-r old:=new`).

### `teleop_xarm7` — xArm7, minimally invasive surgery
```bash
ros2 run teleop_core teleop_xarm7
```
- Button 2 (hold): clutch, which drives the arm. When released, the current robot
  pose is republished so the arm holds still.
- Button 1 (press): toggle the RCM position lock (rotation only while locked).
- Subscribes: `/arm/*`, `/rcm_pos` (`PoseStamped`). Publishes: `/go_to_pose` (`PoseStamped`).
- Parameters: `rate` (100), `position_scale` (5), `orientation_scale` (1),
  `viewer_R_robotbase` (row-major 3x3, default identity), `*_topic`.

### `teleop_fr3` — Franka Research 3 on Husky
```bash
ros2 run teleop_core teleop_fr3
```
- Button 2 (hold): clutch.
- Subscribes: `/arm/*`, `/direct_transform` (`Pose`). Publishes: `/go_to_pose` (`Pose`).
- Parameters: `rate` (10), `position_scale` (1), `orientation_scale` (1),
  `viewer_R_robotbase`, `*_topic`.

### `teleop_cmd` — bimanual UVMS (RexROV)
```bash
ros2 run teleop_core teleop_cmd
```
- Per device: hold button 2 to drive that arm. Press button 1 while holding
  button 2 to toggle that side's gripper.
- Both devices, button 1 only: pilot the vehicle by moving both styluses together.
- Subscribes: `/{left,right}/{button1,button2,measured_cp}`, `/arm1/pose`, `/arm2/pose`
  (`Float32MultiArray` `[x,y,z,rx,ry,rz]`).
- Publishes: `/Xd1`, `/Xd2`, `/grippers`, `/rexrov/cmd_vel` (parameter `cmd_vel_topic`).

Example with a remapped robot topic and a different desk orientation:
```bash
ros2 run teleop_core teleop_xarm7 --ros-args \
  -r /rcm_pos:=/my_robot/pose \
  -p position_scale:=2.0 \
  -p viewer_R_robotbase:="[-1.0,0.0,0.0, 0.0,-1.0,0.0, 0.0,0.0,1.0]"
```

## Config

`config/*.yaml` hold UR5 joint-space trajectories that were used by the earlier
`follow_traj` node (not included in this version). They are installed to
`share/teleop_core/config`.

## Publications
The following are publications by the ARMLAB that use this teleoperation package:

1. Sitler, J., Sowrirajan, S., and Wang, L. (2024). "A Bimanual Teleoperation Framework
   for Light Duty Underwater Vehicle-Manipulator Systems." *21st International Conference
   on Ubiquitous Robots* ([Preprint](https://arxiv.org/abs/2404.03790))
