# OMRON / Adept Cobra s600 ROS 2 + Gazebo

ROS 2 Jazzy + Gazebo Harmonic simulation of the OMRON/Adept Cobra s600 SCARA robot.

The robot is represented as an R-R-P-R chain:

`world -> base_link -> joint_1 -> inner_link -> joint_2 -> outer_link -> joint_3 -> quill_slide -> joint_4 -> quill_link`

## What is included

- Xacro robot description with collision geometry
- SolidWorks mass, centre-of-mass, and inertia values for Links 1-3
- Gazebo Harmonic integration through `gz_ros2_control`
- Joint-state broadcaster
- Joint trajectory and effort control support
- STL visuals for the original robot appearance
- Four controller demos implemented in one ROS 2 Python node:
  - PD control
  - Computed-torque control
  - Sliding-mode control
  - Adaptive-gain robust control

## Model assumptions that still require verification

- Joint 1 to Joint 2: 325 mm
- Joint 2 to quill axis: 275 mm
- Joint 1 limit: +/-105 degrees
- Joint 2 limit: +/-150 degrees
- Joint 3 stroke: 0-210 mm along negative Z
- Joint 4 limit: +/-360 degrees
- Base inertia is a fixed-base approximation inferred from the documented 41 kg total mass
- Effort limits, damping, and the tiny inertia on virtual `quill_slide` are simulation placeholders
- The SolidWorks output coordinate system for every link must match its URDF link frame

Do not use this simulation controller directly on physical hardware until the model and limits have been validated.

---

# ROS 2 Jazzy / Ubuntu 24.04 setup

Install the required packages:

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
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

## Updating after GitHub changes

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git switch main
git pull --ff-only origin main
chmod +x scripts/control_demo.py

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

If local files have uncommitted edits, check `git status` before pulling.

---

# Launch files

The main launch files are:

| Launch file | Purpose |
|---|---|
| `display.launch.py` | RViz-only visualization |
| `gazebo.launch.py` | Base Gazebo simulation |
| `linear_demo.launch.py` | Legacy PD demo |
| `nonlinear_demo.launch.py` | Legacy computed-torque demo |
| `pd_control.launch.py` | PD controller test |
| `computed_torque_control.launch.py` | Computed-torque test |
| `sliding_mode_control.launch.py` | Sliding-mode test |
| `adaptive_control.launch.py` | Adaptive-gain robust test |

For the current controller comparison, use the four dedicated launch files:

```text
pd_control.launch.py
computed_torque_control.launch.py
sliding_mode_control.launch.py
adaptive_control.launch.py
```

---

# Controller architecture

All four launch files use the same implementation:

[`scripts/control_demo.py`](scripts/control_demo.py)

The launch file mainly selects the controller mode and parameter values. The actual control equations are implemented in `control_demo.py`.

```text
launch file
    |
    | mode + gains
    v
control_demo.py
    |
    v
desired_trajectory()
    |
    | q_ref, dq_ref, ddq_ref
    v
tracking_errors()
    |
    | e, e_dot
    v
+-------------------------------+
| PD                            |
| Computed Torque               |
| Sliding Mode                  |
| Adaptive-Gain Robust Control  |
+-------------------------------+
    |
    | tau
    v
clamp_effort()
    |
    v
/effort_controller/commands
    |
    v
Gazebo Cobra s600
    |
    v
/joint_states
    |
    +------ closed-loop feedback ------> control_demo.py
```

The mapping is:

```text
pd_control.launch.py
    -> mode = pd
    -> linear_pd()

computed_torque_control.launch.py
    -> mode = computed_torque
    -> nonlinear_computed_torque()

sliding_mode_control.launch.py
    -> mode = sliding_mode
    -> sliding_mode_control()

adaptive_control.launch.py
    -> mode = adaptive
    -> adaptive_control()
```

---

# Desired trajectory

The current reference trajectory moves Joint 1 and Joint 2 continuously.

Joint 1:

```text
q1_d = q1_0 + A1 sin(omega t)
```

Joint 2:

```text
q2_d = q2_0 + A2 sin(omega t + pi/2)
```

Default values:

```text
Joint 1 amplitude = 0.8 rad
Joint 2 amplitude = 0.5 rad
Period            = 6.0 s
Phase difference  = 90 degrees
```

Joint 3 and Joint 4 hold their startup positions.

```text
Joint 1 -> sinusoidal motion
Joint 2 -> sinusoidal motion with 90-degree phase shift
Joint 3 -> hold
Joint 4 -> hold
```

Moving Joints 1 and 2 together is more useful than moving only Joint 1 because the first two SCARA joints are dynamically coupled. This makes the difference between a simple PD controller and model-based controllers easier to observe.

---

# Tracking error

All four controllers use the same tracking error definition:

```text
e     = q_desired - q
e_dot = dq_desired - dq
```

where:

- `q_desired` is desired joint position
- `q` is measured joint position
- `dq_desired` is desired joint velocity
- `dq` is measured joint velocity

---

# 1. PD control

Implemented by:

```python
linear_pd()
```

Control law:

```text
tau = Kp * e + Kd * e_dot
```

PD control does not explicitly compensate for the robot dynamic model.

Conceptually:

```text
desired trajectory
      -> error
      -> Kp*e + Kd*e_dot
      -> torque
```

Main parameters:

```text
kp
kd
```

Typical effects:

| Observation | Possible adjustment |
|---|---|
| Slow response / large tracking error | Increase `Kp` |
| Overshoot | Increase `Kd` |
| Sustained oscillation | Reduce `Kp` or increase `Kd` |
| Very sluggish motion | Reduce `Kd` |
| Shaking / noisy effort | Reduce `Kd`; if necessary reduce `Kp` |
| Effort saturation | Reduce gains |

---

# 2. Computed-torque control

Implemented by:

```python
nonlinear_computed_torque()
```

The virtual acceleration is:

```text
v = ddq_desired + Kd*e_dot + Kp*e
```

Then the robot model converts the desired acceleration into torque:

```text
tau = M(q)v + C(q,dq)dq + g(q)
```

Conceptually:

```text
desired trajectory
      -> tracking error
      -> PD-like acceleration correction
      -> inverse dynamics model
      -> torque
```

Main parameters:

```text
kp
kd
```

The key difference from simple PD is the use of the approximate robot dynamics model.

---

# SCARA inverse dynamics

Implemented by:

```python
scara_inverse_dynamics()
```

The first two joints use an approximate coupled two-link SCARA model.

Conceptually:

```text
M(q) * acceleration
+ Coriolis / centrifugal terms
+ gravity terms
= required joint torque
```

The important coupling appears because Joint 1 torque depends on both Joint 1 and Joint 2 acceleration, and Joint 2 torque also depends on both accelerations.

```text
tau1 = m11*v1 + m12*v2 + C1

tau2 = m12*v1 + m22*v2 + C2
```

This is why simultaneously moving Joints 1 and 2 produces a better nonlinear-control demonstration.

---

# 3. Sliding-mode control

Implemented by:

```python
sliding_mode_control()
```

Sliding surface:

```text
s = e_dot + lambda*e
```

Virtual acceleration:

```text
v = ddq_desired
    + lambda*e_dot
    + smc_gain*tanh(s / boundary_layer)
```

The inverse dynamics model then converts `v` into torque.

Main parameters:

```text
smc_lambda
smc_gain
boundary_layer
```

Parameter meaning:

### `smc_lambda`

Controls how strongly the tracking error is driven toward the sliding surface.

Increasing it generally gives faster convergence but can make the response more aggressive.

### `smc_gain`

Controls the strength of the robust switching action.

Increasing it can improve disturbance rejection but can also increase control effort and chattering-like behaviour.

### `boundary_layer`

The implementation uses:

```text
tanh(s / boundary_layer)
```

instead of a hard `sign(s)` function.

Smaller boundary layer:

```text
more aggressive
more sign-like
more potential chattering
```

Larger boundary layer:

```text
smoother
less chattering
possibly larger tracking error
```

---

# 4. Adaptive-gain robust control

Implemented by:

```python
adaptive_control()
```

It uses the same sliding variable:

```text
s = e_dot + lambda*e
```

but the robust gain changes online.

Adaptation law:

```text
k_hat_dot = gamma*abs(s) - sigma*k_hat
```

Virtual acceleration:

```text
v = ddq_desired
    + lambda*e_dot
    + k_hat*tanh(s / boundary_layer)
```

Main parameters:

```text
adaptive_gamma
adaptive_sigma
adaptive_gain_max
smc_lambda
boundary_layer
```

Interpretation:

```text
tracking error increases
      -> |s| increases
      -> adaptive gain increases
      -> stronger corrective action
```

When tracking error becomes smaller, the sigma term causes the adaptive gain to decay gradually.

Important: this implementation adapts the robust gain. It is not a full rigid-body parameter estimator and should not be described as a complete Slotine-Li parameter-adaptive controller.

---

# Relationship between the four controllers

A useful conceptual progression is:

```text
PD
 |
 | add robot dynamics model
 v
Computed Torque
 |
 | add robust sliding term
 v
Sliding Mode
 |
 | make robust gain change online
 v
Adaptive-Gain Robust Control
```

| Controller | Error feedback | Robot model | Robust term | Online adaptation |
|---|---:|---:|---:|---:|
| PD | Yes | No | No | No |
| Computed Torque | Yes | Yes | No | No |
| Sliding Mode | Yes | Yes | Yes, fixed gain | No |
| Adaptive-Gain Robust | Yes | Yes | Yes | Yes, robust gain |

---

# Visual controller testing workflow

For the first stage of this project, the controllers can be compared visually in Gazebo.

Do not start all four Gazebo simulations at the same time. Run one controller, observe it, stop Gazebo, then run the next controller.

```text
Test 1: PD
    -> stop
Test 2: Computed Torque
    -> stop
Test 3: Sliding Mode
    -> stop
Test 4: Adaptive
```

Running multiple Gazebo instances is technically possible with separate namespaces/domains/ports, but it is unnecessary for this experiment and can cause topic, controller-manager, Gazebo transport, and CPU/GPU conflicts.

## Prepare once

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

---

# Stage 1 - PD visual tuning

Run:

```bash
pkill -f gz
ros2 launch omron_cobra_s600_description pd_control.launch.py
```

Tune:

```text
kp
kd
```

Suggested process:

```text
1. Start with baseline gains.
2. Observe tracking speed, overshoot and oscillation.
3. Change Kp.
4. Rebuild.
5. Relaunch.
6. Observe again.
7. Change Kd.
8. Rebuild and repeat.
```

Change one parameter group at a time so that the effect is easy to understand visually.

---

# Stage 2 - Computed-torque visual tuning

Run:

```bash
pkill -f gz
ros2 launch omron_cobra_s600_description computed_torque_control.launch.py
```

Tune:

```text
kp
kd
```

Compare against PD while keeping the same:

```text
trajectory
initial pose
amplitude
period
Gazebo physics
```

Watch for:

```text
tracking accuracy
phase lag
overshoot
oscillation
smoothness
```

---

# Stage 3 - Sliding-mode visual tuning

Run:

```bash
pkill -f gz
ros2 launch omron_cobra_s600_description sliding_mode_control.launch.py
```

Tune:

```text
smc_lambda
smc_gain
boundary_layer
```

Watch for:

```text
convergence speed
tracking robustness
shaking / chattering
smoothness
```

---

# Stage 4 - Adaptive visual tuning

Run:

```bash
pkill -f gz
ros2 launch omron_cobra_s600_description adaptive_control.launch.py
```

Tune:

```text
adaptive_gamma
adaptive_sigma
adaptive_gain_max
smc_lambda
boundary_layer
```

Watch for:

```text
how quickly the controller reacts to error
whether the response becomes too aggressive
whether oscillation decreases or increases
tracking smoothness
```

---

# Editing parameters and rebuilding

The current controller parameters are provided through the launch files.

Typical workflow:

```text
edit launch file
    -> save
    -> rebuild
    -> source workspace
    -> stop previous Gazebo
    -> launch again
    -> observe visually
```

Commands:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42

pkill -f gz
ros2 launch omron_cobra_s600_description pd_control.launch.py
```

Replace the launch filename with the controller being tested.

---

# Joint order for gain arrays

The gain arrays use this order:

| Array index | Joint | Type |
|---|---|---|
| 0 | `joint_1` | revolute |
| 1 | `joint_2` | revolute |
| 2 | `joint_3` | prismatic |
| 3 | `joint_4` | revolute |

For example:

```python
'kp': [40.0, 40.0, 220.0, 2.0]
'kd': [12.0, 12.0, 35.0, 0.5]
```

means:

```text
index 0 -> Joint 1
index 1 -> Joint 2
index 2 -> Joint 3
index 3 -> Joint 4
```

Joint 3 is prismatic, so its useful gain values can be very different from the rotary joints.

---

# What good tuning looks like

A good controller is not simply the controller with the largest gains.

Look for:

```text
small tracking error
little overshoot
no sustained oscillation
smooth movement
reasonable response speed
no repeated effort saturation
```

For visual tuning, change one parameter at a time and compare the motion under the same reference trajectory.

---

# Useful ROS 2 topics

The controller uses:

```text
/joint_states
/control_demo/desired
/control_demo/error
/effort_controller/commands
```

Check current state:

```bash
ros2 topic echo /joint_states --once
```

Check tracking error:

```bash
ros2 topic echo /control_demo/error --once
```

Check controller state:

```bash
ros2 control list_controllers
```

Expected effort-control experiment state:

```text
effort_controller         active
joint_state_broadcaster   active
```

---

# Optional quantitative comparison

Visual comparison is useful during early tuning. Later, the same four controllers can be compared quantitatively using rosbag data.

Useful metrics include:

```text
tracking RMSE
maximum absolute error
phase lag
overshoot
settling time
RMS torque
torque variation
```

For a fair comparison, keep the following identical:

```text
initial configuration
desired trajectory
trajectory amplitude
trajectory period
Gazebo physics
payload
test duration
```

---

# Isolating Cobra from an FR3 controller manager

If `ros2 control list_controllers` shows controllers such as `fr3_arm_controller` or `franka_robot_state_broadcaster`, use a separate ROS domain for the Cobra simulation.

In every Cobra terminal:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Then restart Gazebo:

```bash
pkill -f gz
ros2 daemon stop
ros2 launch omron_cobra_s600_description gazebo.launch.py
```

All Cobra terminals must use the same `ROS_DOMAIN_ID`.

---

# Basic Gazebo robot test

Start Gazebo:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```

Read joint positions:

```bash
ros2 topic echo /joint_states --once
```

Configured joint ranges:

| Joint | Type | Unit | Configured range |
|---|---|---|---|
| `joint_1` | Revolute | rad | -1.8326 to +1.8326 |
| `joint_2` | Revolute | rad | -2.6180 to +2.6180 |
| `joint_3` | Prismatic | m | 0.00 to 0.21 |
| `joint_4` | Revolute | rad | -6.2832 to +6.2832 |

Positive Joint 3 motion moves the quill downward because its URDF axis is `0 0 -1`.

---

# RViz-only display

```bash
ros2 launch omron_cobra_s600_description display.launch.py
```

The committed STL visuals are loaded automatically.

---

# Current development goal

The present goal is to use the same Cobra s600 simulation and trajectory to build intuition for progressively more advanced control methods:

```text
PD
-> Computed Torque
-> Sliding Mode
-> Adaptive-Gain Robust Control
```

The controller code is intentionally organized so that the trajectory generator, tracking-error calculation, robot dynamics, controller laws, effort limiting, diagnostics, and ROS 2 interfaces can be studied separately inside `scripts/control_demo.py`.
