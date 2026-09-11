# Nonlinear effort control in Gazebo

This optional path preserves the original position-trajectory workflow. Use
`gazebo.launch.py` for the original controller and `gazebo_effort.launch.py`
only when testing torque/force control.

## Control loop

Gazebo simulates the plant:

```text
M(q) q_ddot + c(q, q_dot) + g(q) = tau
```

The example node applies computed-torque control:

```text
v   = qd_ddot + Kd (qd_dot - q_dot) + Kp (qd - q)
tau = M(q) v + c(q, q_dot) + g(q)
```

For `joint_3`, the command is a force in newtons. The other three commands
are torques in newton-metres.

## Build and launch

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description
source install/setup.bash

pkill -f gz  # only if an earlier Gazebo process is still running
ros2 launch omron_cobra_s600_description gazebo_effort.launch.py
```

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run omron_cobra_s600_description computed_torque_controller.py --ros-args -p use_sim_time:=true
```

The controller is deliberately disabled at startup. Check the current state
and choose a target inside the URDF limits:

```bash
ros2 topic echo /joint_states --once
ros2 param set /computed_torque_controller desired_positions '[0.2, -0.2, 0.05, 0.0]'
ros2 param set /computed_torque_controller enabled true
```

Return to zero and disable:

```bash
ros2 param set /computed_torque_controller desired_positions '[0.0, 0.0, 0.0, 0.0]'
ros2 param set /computed_torque_controller enabled false
```

## Checks before enabling

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /effort_controller/commands
```

`effort_controller` and `joint_state_broadcaster` should be active. The
position controller must not be active at the same time.

## Research limitations

This is an educational baseline, not a validated OMRON actuator model. The
planar model combines the quill mass with Link 2 and uses a placeholder
effective inertia for Joint 4. Friction, gearbox inertia, motor dynamics,
payload, torque constants, and saturation must be identified before reporting
quantitative results or attempting hardware control. Verify the SolidWorks
inertial coordinate frames before using the controller for dynamics research.

Recommended order: effort direction test, low-gain PD, gravity compensation,
computed torque, model validation, and only then NMPC.
