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

[Common problems and fixes](docs/TROUBLESHOOTING.md)

## Start Gazebo

The four STL files are committed and enabled by default:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```


## Continuous side-to-side control demo

The linear and nonlinear demos use the same continuous sinusoidal reference so
their tracking errors can be compared fairly. Only `joint_1` swings; Joints
2-4 hold their startup positions.

Default reference:

- amplitude: 0.8 rad
- period: 6.0 s
- motion: centre to one side, through centre to the other side, continuously

Update and rebuild before running:

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git switch main
git pull --ff-only origin main
chmod +x scripts/control_demo.py

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
```

Run nonlinear computed-torque control:

```bash
ros2 launch omron_cobra_s600_description nonlinear_demo.launch.py
```

Run linear PD control for comparison:

```bash
ros2 launch omron_cobra_s600_description linear_demo.launch.py
```

The implementation is in
[`scripts/control_demo.py`](scripts/control_demo.py). This is an approximate
Gazebo dynamics demonstration and must not be used directly on physical
hardware.

## Linear versus nonlinear test process

Use separate terminals because the Gazebo launch, diagnostic commands, and data
recorder must run at the same time. Type each command once; do not join two
`ros2 launch` commands on the same line.

| Terminal | Purpose |
|---|---|
| Terminal 1 | Run Gazebo, the effort controller, and either the linear or nonlinear controller |
| Terminal 2 | Verify controllers, joint feedback, desired trajectory, and tracking error |
| Terminal 3 | Record identical topics for later comparison |

Every terminal used for this experiment must use the same ROS domain:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Prepare the latest version

Run this once before the experiments:

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git switch main
git pull --ff-only origin main
chmod +x scripts/control_demo.py

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

### Test 1: linear PD control

In Terminal 1:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
pkill -f gz
ros2 launch omron_cobra_s600_description linear_demo.launch.py move_time:=5.0
```

Keep Terminal 1 running. In Terminal 2:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /control_demo/error --once
```

Expected controller state:

```text
effort_controller         active
joint_state_broadcaster   active
```

In Terminal 3, record the test:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/ros2_ws/bags
cd ~/ros2_ws/bags

ros2 bag record -o linear_test \
  /clock \
  /joint_states \
  /control_demo/desired \
  /control_demo/error \
  /effort_controller/commands
```

Record several complete motion cycles, then press `Ctrl+C` in Terminal 3.
Stop Terminal 1 with `Ctrl+C` before starting the nonlinear test.

### Test 2: nonlinear computed-torque control

Restart from the same initial position. In Terminal 1:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
pkill -f gz
ros2 launch omron_cobra_s600_description nonlinear_demo.launch.py move_time:=5.0
```

Use Terminal 2 for the same controller and topic checks. In Terminal 3:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/bags

ros2 bag record -o nonlinear_test \
  /clock \
  /joint_states \
  /control_demo/desired \
  /control_demo/error \
  /effort_controller/commands
```

Again, record the same number of complete motion cycles.

### Test different Kp and Kd gains

The gain arrays in `launch/nonlinear_demo.launch.py` use this joint order:

| Array index | Joint | Type |
|---|---|---|
| 0 | `joint_1` | revolute |
| 1 | `joint_2` | revolute |
| 2 | `joint_3` | prismatic |
| 3 | `joint_4` | revolute |

Baseline nonlinear gains:

```python
'kp': [18.0, 18.0, 45.0, 25.0],
'kd': [8.0, 8.0, 14.0, 8.0],
```

The feedback part of the controller is

```text
position error:  e     = q_desired - q
velocity error:  e_dot = dq_desired - dq
feedback:        u     = Kp * e + Kd * e_dot
```

- `Kp` is the position-correction gain. A larger value pulls the joint toward
  its desired position more strongly and usually reduces slow tracking error.
- `Kd` is the velocity-error gain and provides damping. A larger value usually
  reduces overshoot and oscillation, but too much can make motion sluggish and
  can amplify noisy velocity measurements.

Use the observed response to decide which gain to change:

| Observed behaviour | Likely cause | Adjustment |
|---|---|---|
| Slow response or large position error | `Kp` is too low | Increase `Kp` |
| Fast response with overshoot | Damping is too low | Increase `Kd` |
| Continuous oscillation | `Kp` is too high or `Kd` is too low | Decrease `Kp` or increase `Kd` |
| Very sluggish motion | `Kd` is too high | Decrease `Kd` |
| Repeated effort saturation | Gains are too aggressive, usually `Kp` | Decrease `Kp` |
| Noisy effort or shaking | `Kd` may amplify velocity noise, or both gains are high | Decrease `Kd`; if needed, decrease `Kp` |
| Smooth response, small error, little overshoot, and moderate effort | Gains are suitable | Keep the gains |

To test new gains, edit the arrays in
`launch/nonlinear_demo.launch.py`. Tune only one joint at a time:

1. Keep `Kd` fixed and increase `Kp` in steps of about 10-20 percent.
2. Stop increasing `Kp` when oscillation, large overshoot, or effort
   saturation appears; then reduce it slightly.
3. Increase `Kd` gradually until overshoot and oscillation decrease.
4. Repeat the same test for the next joint.

A good gain set is not simply the largest one. It gives small tracking error,
little overshoot, no sustained oscillation, smooth commanded effort, and a
reasonable settling time.

For example, test slightly higher Joint 1 gains:

```python
'kp': [20.0, 18.0, 45.0, 25.0],
'kd': [9.0, 8.0, 14.0, 8.0],
```

Rebuild and launch a slow test first:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42

pkill -f gz
ros2 launch omron_cobra_s600_description nonlinear_demo.launch.py move_time:=8.0
```

Record each gain set with a unique bag name, for example:

```bash
mkdir -p ~/ros2_ws/bags
cd ~/ros2_ws/bags

ros2 bag record -o nonlinear_kp20_kd9 \
  /clock \
  /joint_states \
  /control_demo/desired \
  /control_demo/error \
  /effort_controller/commands
```

Keep the trajectory, initial pose, payload, physics settings, `move_time`, and
recording duration unchanged. Compare tracking-error RMSE, maximum absolute
error, overshoot, settling time, oscillation, and RMS commanded effort. Stop
and lower the gains if the robot becomes unstable or repeatedly reaches its
effort limits. Joint 3 is prismatic, so its useful gains can be very different
from the rotary-joint gains.

### Fair comparison requirements

Keep these conditions identical:

- initial joint configuration
- desired trajectory and target
- `move_time`
- Gazebo physics settings
- payload
- recording duration

Compare joint-error RMSE, maximum absolute error, phase lag, overshoot, and
commanded effort. Repeat with `move_time:=8.0`, `5.0`, and `3.0`. Faster
motion produces stronger dynamic coupling and usually makes the difference
between linear PD and computed-torque control easier to observe.

Inspect each recording with:

```bash
ros2 bag info ~/ros2_ws/bags/linear_test
ros2 bag info ~/ros2_ws/bags/nonlinear_test
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
