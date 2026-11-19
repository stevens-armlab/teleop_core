# AGENTS.md — teleop_core

Rules for developing this package, for people and coding agents alike.
`README.md` covers how to use the package; this file covers how to change it.

## Safety first

These nodes command physical robots (FR3, xArm7, UVMS arms). A clean build or a
passing test does not mean a motion is safe.

- Never test a behavior change on hardware first. Validate it with simulated
  topics (see *Build and test*) or in simulation before connecting a real robot.
- A node must publish **nothing** (or hold the current pose, as `teleop_xarm7` does)
  until it has received both a haptic pose and a robot pose. Never publish a
  command computed from `None` or default-initialized state.
- Motion commands are only produced while the clutch (button 2) is held.
  Releasing the clutch must clear the anchor so the next press re-anchors.
  Do not change this contract without the maintainer's explicit approval.
- Do not raise default scale factors (`position_scale`, `orientation_scale`) or
  publish rates without stating why in the change description.
- Keep the hardware emergency stop reachable whenever a robot is powered.

## Platform

- Target: **ROS 2 Jazzy** on Ubuntu 24.04, Python 3.12 (`/usr/bin/python3`).
- This is an **`ament_python`** package. Do not add `CMakeLists.txt`, catkin,
  `rospy`, `tf_conversions`, or any other ROS 1 APIs.
- The machine has a pyenv Python on `PATH` that cannot import `rclpy` or `PyKDL`.
  Always use `/usr/bin/python3` (or `ros2 run`, `colcon`) to run or test code.
  See *Haptic driver setup → step 0*.
- Haptic input comes from a 3D Systems Touch (OpenHaptics 3.4). The driver is
  external to this package. Do not vendor driver or OpenHaptics code here.

## Haptic driver setup (3D Systems Touch → ROS 2)

Follow these steps in order on a fresh Ubuntu 24.04 + ROS 2 Jazzy machine. The
result is the `sensable_phantom` node publishing `/arm/measured_cp`,
`/arm/button1`, and `/arm/button2`, which this package consumes.
Last verified: 2026-11-19, 30-package workspace build.

### 0. Use the system Python in every terminal

VS Code's Python extension can auto-activate a pyenv interpreter
(`VIRTUAL_ENV=~/.pyenv/versions/3.12.12`). That breaks every CMake package with
`No module named 'catkin_pkg'`. Before building or running anything:

```bash
which python3        # must print /usr/bin/python3
# if not, either select /usr/bin/python3 via "Python: Select Interpreter" and open a
# new terminal, or strip pyenv from the current shell:
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v pyenv | paste -sd:)
unset VIRTUAL_ENV PYENV_SHELL
```

For a permanent fix, add `"python.terminal.activateEnvironment": false` to the
workspace `.vscode/settings.json` (the folder is gitignored).

### 1. Touch device driver (vendor)

Download from 3D Systems' Touch driver page: `TouchDriver_2025_12_10.tgz` and
`openhaptics_3.4-0-developer-edition-amd64.tar.gz`. Put both in `~/Downloads`.

```bash
# Touch driver: libPhantomIOLib42 / libPhantomManagerLite + udev rule
tar -xzf ~/Downloads/TouchDriver_2025_12_10*.tgz -C ~/Desktop
cd ~/Desktop/TouchDriver_2025_12_10 && ./install_haptic_driver

# The device appears as /dev/ttyACM0 (root:dialout, 660). The user must be in dialout.
sudo usermod -aG dialout $USER
```

### 2. OpenHaptics 3.4 SDK

```bash
W=$(mktemp -d) && tar -xzf ~/Downloads/openhaptics_3.4-0-developer-edition-amd64.tar.gz -C $W
S=$W/openhaptics_3.4-0-developer-edition-amd64
sudo cp -R $S/opt/* /opt && sudo cp -R $S/usr/lib/* /usr/lib && sudo cp -R $S/usr/include/* /usr/include
for lib in HD HL QH QHGLUTWrapper; do
  sudo ln -sfn /usr/lib/lib$lib.so.3.4.0 /usr/lib/lib$lib.so.3.4
  sudo ln -sfn /usr/lib/lib$lib.so.3.4.0 /usr/lib/lib$lib.so
done
sudo chmod -R a+rX /opt/OpenHaptics
echo 'export OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0' | sudo tee /etc/profile.d/openhaptics.sh
# TouchCheckup needs libglut.so.3
[ -e /usr/lib/x86_64-linux-gnu/libglut.so.3 ] || \
  sudo ln -s /usr/lib/x86_64-linux-gnu/libglut.so.3.12 /usr/lib/x86_64-linux-gnu/libglut.so.3
sudo ldconfig && rm -rf $W
grep -q GTDD_HOME ~/.bashrc || echo 'export GTDD_HOME=$HOME/.3dsystems' >> ~/.bashrc
```

`~/setup_touch_libs.sh` on the lab machine does the same thing (`sudo bash ~/setup_touch_libs.sh`).

**Reboot**, so the dialout group, udev rule, and environment take effect. Then
calibrate and create the default device config:

```bash
cd ~/Desktop/TouchDriver_2025_12_10/bin && ./TouchCheckup
```

This writes `~/.3dsystems/config/Default Device.config`. The ROS driver uses
that default device, which it names `arm`.

### 3. Build dependencies (apt)

```bash
sudo apt install qtbase5-dev libqt5opengl5-dev libqt5xmlpatterns5-dev swig \
  libgraphviz-dev libxml2-dev libjsoncpp-dev libraw1394-dev libncurses-dev \
  libglu1-mesa-dev gfortran python3-dev python3-numpy
```

Easy to miss: `libqt5xmlpatterns5-dev` (cisst won't find Qt5 without it), `swig`
(cisst Python bindings), and `libgraphviz-dev` (cisst includes `cgraph.h`
unconditionally).

### 4. Clone the ROS driver stack

Use this exact layout in the workspace `src/`. **sawSensablePhantom must be on `devel`.**
cisst `main` removed the `cisstQt` library (commit 255d9b8b, June 2026), and only
sawSensablePhantom `devel` (commit c06b376) has the matching CMake update. On `main`,
`sensable_phantom` fails with *"cisst_FOUND to FALSE"*. Check whether upstream `main`
has caught up before switching back.

```bash
cd ~/research/colcon_ws/src
mkdir -p cisst-saw crtk
git -C cisst-saw clone https://github.com/jhu-cisst/cisstNetlib.git
git -C cisst-saw clone https://github.com/jhu-cisst/cisst.git
git -C cisst-saw clone https://github.com/jhu-cisst/cisst-ros.git
git -C cisst-saw clone https://github.com/jhu-saw/sawControllers.git
git -C cisst-saw clone https://github.com/jhu-saw/sawKeyboard.git
git -C cisst-saw clone -b devel https://github.com/jhu-saw/sawSensablePhantom.git
git -C crtk clone https://github.com/collaborative-robotics/crtk_msgs.git
git -C crtk clone https://github.com/collaborative-robotics/crtk_python_client.git
```

Known-good commits: cisst `5ffd602c`, cisstNetlib `e8eb1f9`, cisst-ros `014c9ab`,
sawControllers `b3b78f9`, sawKeyboard `c374200`, sawSensablePhantom `c06b376`,
crtk_msgs `14b04fc`, crtk_python_client `d002e28`. If a fresh clone of `main`
breaks, check out these commits.

### 5. Build

```bash
cd ~/research/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Expected noise (ignore these):
- rosdep cannot resolve `rviz`/`catkin`. Those are ROS 1 keys in the saw packages.
- CMake dev warnings (OpenGL CMP0072, FetchContent).
- `sawSensablePhantomIGTL` skips itself because `sawOpenIGTLink` is not present.

After any failed cisst configure, `rm -rf build/cisst` before rebuilding, because
CMake caches "not found" results.

### 6. Verify

```bash
ros2 run sensable_phantom sensable_phantom          # terminal 1, opens a Qt window
ros2 topic list | grep arm                          # terminal 2
ros2 topic echo /arm/measured_cp --once
ros2 run teleop_core teleop_xarm7                   # or teleop_fr3 / teleop_cmd
```

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No module named 'catkin_pkg'` in every CMake package | pyenv venv active. See step 0 |
| `CISST_HAS_QT is ON but no usable Qt version was found` | `libqt5xmlpatterns5-dev` missing. Then `rm -rf build/cisst` |
| `cgraph.h: No such file or directory` | `libgraphviz-dev` missing. Then `rm -rf build/cisst` |
| `cisst library "cisstQt" doesn't exist` / `cisst_FOUND to FALSE` | sawSensablePhantom on `main`. Check out `devel` and `rm -rf build/sawSensablePhantomCore build/sensable_phantom` |
| Driver can't open the device | Run `TouchCheckup` first. Check `echo $GTDD_HOME`, `ls -l /dev/ttyACM0`, and `id -nG` (needs `dialout`) |
| `ros2 run` fails importing `rclpy` / `PyKDL` | Wrong Python. See step 0 |

## Package layout

```text
teleop_core/
├── package.xml, setup.py, setup.cfg, resource/teleop_core
├── teleop_core/
│   ├── kdl_utils.py      # shared frame conversions + anchoring math
│   ├── teleop_fr3.py     # Franka Research 3
│   ├── teleop_xarm7.py   # xArm7 (RCM / surgical)
│   └── teleop_cmd.py     # bimanual UVMS (RexROV)
├── config/               # installed to share/teleop_core/config
└── test/
```

- **One node per robot.** `teleop_fr3` is only for the Franka Research 3, and
  `teleop_xarm7` is only for the xArm7. Support a new robot (e.g. Kinova Gen3) with
  a new `teleop_<robot>.py` node, not by repurposing an existing one.
- Every new node needs an entry in `setup.py` `console_scripts`
  (`teleop_<robot> = teleop_core.teleop_<robot>:main`) and a section in `README.md`.
- New files that must be installed (config, launch) need a `data_files` entry in
  `setup.py`. Otherwise they are silently left out of `install/`.

## Code rules

- **Shared math lives in `kdl_utils.py`.** Frame conversions, scaling, and the
  anchor / relative-command computation must not be re-implemented inside a node.
  Any change to `kdl_utils.py` affects every robot, so add or update a test with it.
- Use PyKDL for frame math. Convert messages only through `kdl_utils`
  (`pose_to_frame`, `frame_to_pose`, `frame_from_rotvec`).
- Nodes are `rclpy.node.Node` subclasses with a `main()` that spins, handles
  `KeyboardInterrupt`, and calls `destroy_node()` and `rclpy.try_shutdown()`.
- Use a timer for the control loop. Do not use `while` loops with `sleep`.
- **Everything robot- or desk-specific is a parameter:** topic names, scale
  factors, rate, and the `viewer_R_robotbase` rotation (row-major 3×3). Keep defaults
  equal to the robot's known working setup. Do not hard-code per-person desk setups.
  Document alternatives in a comment or the README instead.
- Default topics follow the sawSensablePhantom / CRTK convention:
  `/<device>/measured_cp` (`PoseStamped`) and `/<device>/button1|button2`
  (`sensor_msgs/Joy`, `buttons[0]`). The device is `arm` for single-device nodes,
  and `left`/`right` for bimanual.
- Button messages arrive on change. Toggle logic (e.g. RCM lock, gripper) must act
  on the press edge only.
- Keep the existing frame names and their meaning: `VIEWER_HAPTIC`
  (haptic base → viewer) and `viewer_R_robotbase` (robot base → viewer).
  If you change a frame convention, update the test that checks it.
- Log with `self.get_logger()`. Throttle anything inside the control loop
  (`throttle_duration_sec`). Never log at `info` every cycle.
- Follow PEP 8 with a 100-character line limit and single quotes, matching the
  existing code.

## Dependencies

- Declare every runtime import in `package.xml` using a rosdep key
  (`rclpy`, `python_orocos_kdl_vendor`, `python3-numpy`, ...). Check it with
  `rosdep check --from-paths src/teleop_core --ignore-src`.
- Prefer apt/rosdep binaries. Do not `pip install` into the system Python.

## Build and test

From the workspace root (`~/research/colcon_ws`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select teleop_core
colcon test --packages-select teleop_core
colcon test-result --test-result-base build/teleop_core --verbose
```

- All tests must pass before a change is considered done.
- Math changes need unit tests in `test/` (plain pytest, no ROS graph required).
- Node changes need a smoke test with simulated topics: publish fake
  `measured_cp`, button, and robot-pose messages, press the clutch, move the
  stylus, and check that the published command moves the expected direction and
  amount. Use a private `ROS_DOMAIN_ID` so the test doesn't reach a live robot.

## Git

- This package is its own repository, separate from the workspace repo. Work on the
  `ros2-jazzy` branch. `devel` holds the ROS 1 version.
- Keep commits focused: one robot or one concern per commit.
- Do not commit `build/`, `install/`, `log/`, `__pycache__/`, or recorded data
  (`*.bag`, `*.db3`, `*.mcap`).
