# OMRON / Adept Cobra s600 ROS 2 URDF

Draft ROS 2 description reconstructed from four supplied STL files.

## Robot structure

The robot is modeled as the SCARA chain R-R-P-R:

`base_link -> joint_1 -> inner_link -> joint_2 -> outer_link -> joint_3 -> quill_slide -> joint_4 -> quill_link`

Identified mesh assemblies:

- `fixed-base.STL` -> base assembly
- `link1.STL` -> inner link assembly
- `link2.STL` -> outer link assembly
- `link3.STL` -> quill assembly

Approximate planar link-center distances used:

- Joint 1 to Joint 2: 325 mm
- Joint 2 to quill axis: 275 mm
- Total planar reach: 600 mm

## Joint limits used

- J1: +/-105 deg
- J2: +/-150 deg
- J3: 0 to 210 mm
- J4: +/-360 deg

J3 moves along `-Z`; `q=0` is the retracted/up position.

## Important limitations

This is a kinematic/visual URDF draft for RViz and further refinement.

Mass and inertia are intentionally omitted because STL geometry alone does not provide trustworthy real robot mass/material data. The `effort` values in the URDF are placeholders and must not be treated as physical actuator limits.

The meshes committed through this chat are low-poly preview versions generated from the supplied STL geometry so the repository can be cloned and tested directly. Keep the same filenames if you later replace them with the original full-resolution STL files.

Before Gazebo, MuJoCo, torque control, collision-critical planning, or hardware control, verify joint origins/axes against the physical robot or official CAD and add correct inertial properties.

## Ubuntu / ROS 2 usage

```bash
cd ~/ros2_ws/src
git clone https://github.com/ytang19-glitch/OMRON-Cobra-S600-ROS2-URDF.git omron_cobra_s600_description

cd ~/ros2_ws
colcon build --packages-select omron_cobra_s600_description
source install/setup.bash
ros2 launch omron_cobra_s600_description display.launch.py
```

In RViz, set the Fixed Frame to `base_link` if it is not selected automatically.
