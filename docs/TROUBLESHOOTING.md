# Common problems and fixes

This page records common setup, build, controller, and Gazebo problems for the
OMRON Cobra s600 ROS 2 package.

## 1. Package not found

Example:

```text
Package 'omron_cobra_s600_description' not found
searching: ['/opt/ros/jazzy']
```

ROS 2 can see Jazzy, but the workspace overlay is not sourced. Build from the
workspace root and source the result:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix omron_cobra_s600_description
```

Run `colcon build` from `~/ros2_ws`, not `~/ros2_ws/src`. Source the
workspace again in every new terminal.

## 2. Repository directory not found

The GitHub repository is normally cloned locally with the ROS package name:

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git pull --ff-only origin main
```

The GitHub name `OMRON-Cobra-S600-ROS2-URDF` may not be the local directory
name. Check with:

```bash
ls ~/ros2_ws/src
```

Also, `cd /src` means a directory directly under the filesystem root.
Use `cd ~/ros2_ws/src` for this workspace.

## 3. Wrong controller manager is discovered

If the controller list contains `fr3_arm_controller` or
`franka_robot_state_broadcaster`, ROS 2 is discovering the Franka controller
manager. Isolate the Cobra simulation:

```bash
export ROS_DOMAIN_ID=42
```

Set this before starting Gazebo and in every Cobra command terminal. Restart
Gazebo after changing it. Both terminals must return `42` from:

```bash
echo $ROS_DOMAIN_ID
```

Do not activate real FR3 controllers while troubleshooting this simulation.

## 4. Controller-manager service is unavailable

Example:

```text
waiting for service /controller_manager/list_controllers to become available
```

Gazebo is not running, has shut down, or uses a different ROS domain. Keep the
launch terminal running and check that both terminals use the same
`ROS_DOMAIN_ID`.

## 5. Trajectory goal is rejected

Check:

```bash
ros2 control list_controllers
```

The original trajectory command requires:

```text
cobra_controller  joint_trajectory_controller/JointTrajectoryController  active
```

Launch position-control mode:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```

Do not send `FollowJointTrajectory` goals when
`effort_controller` is the selected controller.

## 6. Nonlinear demo opens Gazebo but does not move

Launch the complete nonlinear demo:

```bash
ros2 launch omron_cobra_s600_description nonlinear_demo.launch.py
```

In a second terminal using the same domain:

```bash
ros2 control list_controllers
ros2 node list | grep nonlinear
ros2 topic echo /joint_states --once
timeout 5 ros2 topic hz /effort_controller/commands
```

Expected conditions:

- `joint_state_broadcaster` is active.
- `effort_controller` is active.
- `/nonlinear_control_demo` exists.
- Effort commands are published near 250 Hz.

## 7. control_demo.py is missing from libexec

Example:

```text
executable 'control_demo.py' not found on the libexec directory
```

The script was not installed, usually because the local repository or package
installation is stale. Update and rebuild:

```bash
pkill -f gz

cd ~/ros2_ws/src/omron_cobra_s600_description
git switch main
git pull --ff-only origin main
chmod +x scripts/control_demo.py

cd ~/ros2_ws
rm -rf build/omron_cobra_s600_description
rm -rf install/omron_cobra_s600_description
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source ~/ros2_ws/install/setup.bash
```

Verify:

```bash
ls -l ~/ros2_ws/install/omron_cobra_s600_description/lib/omron_cobra_s600_description/control_demo.py
```

## 8. Effort controller is not loaded

Install the controller package and rebuild:

```bash
sudo apt update
sudo apt install ros-jazzy-forward-command-controller

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

Check that the controller type was loaded from YAML:

```bash
ros2 param get /controller_manager effort_controller.type
```

Expected value:

```text
forward_command_controller/ForwardCommandController
```

If Gazebo remains running, load it manually:

```bash
ros2 run controller_manager spawner effort_controller \
  --controller-manager /controller_manager
```

## 9. /joint_states produces no message

The broadcaster must be active:

```bash
ros2 control list_controllers
ros2 control set_controller_state joint_state_broadcaster active
```

If activation fails, inspect:

```bash
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
```

## 10. Gazebo mesh files cannot be resolved

Test using primitive visuals:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py use_meshes:=false
```

If primitives work, update and rebuild the package so the mesh resources and
Gazebo resource path are installed correctly.

## 11. Gazebo remains after Ctrl+C

First close the launch with `Ctrl+C`. If a process remains:

```bash
pkill -f gz
```

Restart Gazebo only after the old process has exited.

## 12. libEGL or DRI graphics warnings

Warnings such as `libEGL warning` or `failed to create dri2 screen` are
usually graphics-driver warnings. They are not the cause when the real launch
error says that a controller or executable is missing. Diagnose the first
specific ROS 2 `ERROR` line before changing graphics settings.
