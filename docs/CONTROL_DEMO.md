# Linear vs Nonlinear Control Demo

This demo runs the **same quintic joint-space trajectory** with two effort controllers so the comparison is fair.

Trajectory:

`initial pose -> [0.6, -0.8, 0.08, 0.6] -> hold -> initial pose`

The first, second and fourth entries are revolute-joint angles in radians. The third entry is the prismatic joint position in metres.

## 1. Build

```bash
cd ~/ros2_ws/src
# clone only if you do not already have the repository
git clone https://github.com/ytang19-glitch/OMRON-Cobra-S600-ROS2-URDF.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If the repository already exists:

```bash
cd ~/ros2_ws/src/OMRON-Cobra-S600-ROS2-URDF
git pull
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Run the linear PD demo

```bash
ros2 launch omron_cobra_s600_description linear_demo.launch.py
```

Linear controller:

```text
tau = Kp (q_d - q) + Kd (qdot_d - qdot)
```

It does not explicitly use the robot mass matrix, Coriolis/centrifugal terms or gravity model.

## 3. Run the nonlinear computed-torque demo

Close the first Gazebo process, then run:

```bash
ros2 launch omron_cobra_s600_description nonlinear_demo.launch.py
```

Nonlinear controller:

```text
v = qddot_d + Kd (qdot_d - qdot) + Kp (q_d - q)

tau = M(q) v + C(q,qdot) qdot + g(q)
```

The current implementation uses an approximate SCARA dynamics model based on the dimensions, masses and inertias committed in the URDF. It is for simulation/teaching and must be validated before use on real hardware.

## 4. Useful topics

Actual robot state:

```bash
ros2 topic echo /joint_states
```

Desired trajectory:

```bash
ros2 topic echo /control_demo/desired
```

Tracking error:

```bash
ros2 topic echo /control_demo/error
```

Effort command:

```bash
ros2 topic echo /effort_controller/commands
```

## 5. Fair comparison

Use exactly the same:

- robot model
- initial pose
- target pose
- quintic trajectory
- move time
- physics settings
- effort limits

Run each controller separately and compare:

1. **Tracking RMSE**: lower is better.
2. **Maximum absolute tracking error**: lower is better.
3. **Settling/lag**: does the actual joint motion stay close to the desired motion?
4. **Control effort**: compare peak torque/force and integrated squared effort.
5. **Gravity effect on joint 3**: linear PD must fight gravity only through feedback, while computed torque explicitly includes gravity compensation.

The strongest professor demo is a plot of `q_d(t)` and `q(t)` for the same joint under both controllers, plus a second plot of tracking error.

## 6. What result should you expect?

Do not assume the nonlinear controller must always look better. If the model parameters are inaccurate or gains are poorly tuned, computed torque can perform worse. The scientific result is the measured comparison, not a predetermined winner.
