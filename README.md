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
- Primitive fallback visuals, so Gazebo works before the STL files are added
- Optional STL visuals for the original robot appearance

## Model assumptions that still require verification

- Joint 1 to Joint 2: 325 mm
- Joint 2 to quill axis: 275 mm
- Joint 1 limit: +/-105 degrees
- Joint 2 limit: +/-150 degrees
- Joint 3 stroke: 0-210 mm along negative Z
- Joint 4 limit: +/-360 degrees
- Effort limits, damping, and the tiny inertia on virtual `quill_slide` are simulation placeholders
- The SolidWorks output coordinate system for every link must match its URDF link frame. If it does not, transform the COM and inertia tensor before using the model for dynamics research.

Do not use this draft to command the physical robot until these values have been checked against official documentation or measurements.

## ROS 2 Jazzy / Ubuntu 24.04 setup

Install Gazebo Harmonic and the ROS 2 control packages:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim \
  ros-jazzy-gz-ros2-control \
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

## Start Gazebo

The repository currently uses primitive visuals by default because the four binary STL files have not been committed:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```

After the simulator starts, verify the controllers:

```bash
ros2 control list_controllers
```

Expected result:

```text
cobra_controller            joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster            active
```

Send a safe test trajectory. Joint 3 is in metres; the other joints are in radians:

```bash
ros2 action send_goal /cobra_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4], points: [{positions: [0.3, -0.5, 0.08, 0.4], time_from_start: {sec: 4}}]}}"
```

## Use the STL visuals

Copy these exact files into `meshes/`:

- `fixed-base.STL`
- `link1.STL`
- `link2.STL`
- `link3.STL`

Then rebuild and launch with:

```bash
cd ~/ros2_ws
colcon build --packages-select omron_cobra_s600_description
source install/setup.bash
ros2 launch omron_cobra_s600_description gazebo.launch.py use_meshes:=true
```

The STL files are used for appearance only. Gazebo uses simple primitive collision geometry for faster and more stable contact simulation.

## RViz-only display

```bash
ros2 launch omron_cobra_s600_description display.launch.py
```

Add `use_meshes:=true` after installing the STL files.
