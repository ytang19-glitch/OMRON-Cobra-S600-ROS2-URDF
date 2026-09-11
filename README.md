# OMRON / Adept Cobra s600 ROS 2 + Gazebo

ROS 2 description and Gazebo Harmonic simulation of the OMRON/Adept Cobra s600 SCARA robot.

The robot is represented as an R-R-P-R chain:

`world -> base_link -> joint_1 -> inner_link -> joint_2 -> outer_link -> joint_3 -> quill_slide -> joint_4 -> quill_link`

## What is included

- Xacro robot description with collision geometry
- SolidWorks mass, centre-of-mass, and inertia values for Links 1-3
- Gazebo Harmonic integration through `gz_ros2_control`
- Joint-state broadcaster
- Four-joint trajectory controller
- Committed STL visuals for the original robot appearance
- Optional primitive fallback visuals

## Model assumptions that still require verification

- Joint 1 to Joint 2: 325 mm
- Joint 2 to quill axis: 275 mm
- Joint 1 limit: +/-105 degrees
- Joint 2 limit: +/-150 degrees
- Joint 3 stroke: 0-210 mm along negative Z
- Joint 4 limit: +/-360 degrees
- Base inertia is a fixed-base approximation inferred from the documented 41 kg total mass
- Effort limits, damping, and the tiny inertia on virtual `quill_slide` are simulation placeholders
- The SolidWorks output coordinate system for every link must match its URDF link frame. If it does not, transform the COM and inertia tensor before using the model for dynamics research.

Do not use this draft to command the physical robot until these values have been checked against official documentation or measurements.

## ROS 2 Jazzy / Ubuntu 24.04 setup

Install Gazebo Harmonic and the ROS 2 control packages:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2
```

Clone and build:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ytang19-glitch/OMRON-Cobra-S600-ROS2-URDF.git omron_cobra_s600_description

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select omron_cobra_s600_description
source install/setup.bash
```


## Updating after GitHub changes

When files have changed on GitHub, follow the dedicated guide to pull the
latest revision and rebuild the ROS 2 package:

[Pull GitHub changes and rebuild](docs/update_and_rebuild.md)

## Start Gazebo

The four STL files are committed and enabled by default:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```


## Isolating Cobra from an FR3 controller manager

If `ros2 control list_controllers` shows controllers such as
`fr3_arm_controller` or `franka_robot_state_broadcaster`, the command is
discovering the Franka controller manager instead of the Cobra simulation.
Use a separate ROS domain for Cobra. Every Cobra terminal must use the same
domain ID, and Gazebo must be restarted after changing it.

Stop the old Gazebo process, then start Cobra in Terminal 1:

```bash
pkill -f gz
ros2 daemon stop

export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch omron_cobra_s600_description gazebo.launch.py
```

Keep Terminal 1 running. In Terminal 2:

```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 control list_controllers
```

Expected result:

```text
cobra_controller            joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster            active
```

If the command waits for
`/controller_manager/list_controllers`, Gazebo is not running on the same
ROS domain. Check both terminals:

```bash
echo $ROS_DOMAIN_ID
```

Both must print `42`. Do not activate FR3 controllers while troubleshooting
the Cobra simulation.

## Testing the Gazebo robot

Open a second terminal while Gazebo is running:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 1. Verify the controllers

```bash
ros2 control list_controllers
```

Expected result:

```text
cobra_controller            joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster            active
```

If either controller is not active, inspect the launch terminal before sending motion commands.

### 2. Read the current joint positions

```bash
ros2 topic echo /joint_states --once
```

Joint units and configured limits:

| Joint | Type | Unit | Configured range |
|---|---|---|---|
| `joint_1` | Revolute | radians | -1.8326 to +1.8326 |
| `joint_2` | Revolute | radians | -2.6180 to +2.6180 |
| `joint_3` | Prismatic | metres | 0.00 to 0.21 |
| `joint_4` | Revolute | radians | -6.2832 to +6.2832 |

Positive `joint_3` motion moves the quill downward because its URDF axis is `0 0 -1`.

### 3. Test each joint separately

Test Joint 1:

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1], points: [{positions: [0.5], time_from_start: {sec: 3}}]}}"
```

Test Joint 2:

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_2], points: [{positions: [-0.5], time_from_start: {sec: 3}}]}}"
```

Test Joint 3 with 100 mm downward travel:

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_3], points: [{positions: [0.10], time_from_start: {sec: 3}}]}}"
```

Test Joint 4:

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_4], points: [{positions: [0.7], time_from_start: {sec: 3}}]}}"
```

### 4. Test coordinated motion

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.5, -0.5, 0.10, 0.7], time_from_start: {sec: 4}}]}}"
```

### 5. Return to the home configuration

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 4}}]}}"
```

### Expected test results

- Every action reports that the goal was accepted and finishes successfully.
- `joint_1` and `joint_2` rotate the two horizontal SCARA links.
- `joint_3` moves the quill vertically.
- `joint_4` rotates the tool shaft.
- The base remains fixed and the robot does not shake, collapse, or pass through its configured joint limits.
- `ros2 topic echo /joint_states --once` reports positions close to the commanded targets.

## Visual and collision geometry

The committed STL files are used for appearance by default. The package exports its parent share directory to `GZ_SIM_RESOURCE_PATH`, allowing Gazebo to resolve the converted `model://omron_cobra_s600_description/meshes/...` URIs. Gazebo deliberately uses simple primitive collision geometry for faster and more stable contact simulation.

To troubleshoot mesh loading or run without the STL visuals:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py use_meshes:=false
```

## RViz-only display

```bash
ros2 launch omron_cobra_s600_description display.launch.py
```

The committed STL visuals are loaded automatically.
