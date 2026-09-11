# Mesh files

The four original OMRON/Adept Cobra s600 STL files are committed in this folder:

- `fixed-base.STL`
- `link1.STL`
- `link2.STL`
- `link3.STL`

The Xacro applies `scale="0.001 0.001 0.001"` because the mesh coordinates use millimetres while ROS uses metres.

The STL files are visual geometry. Gazebo uses simplified primitive collision geometry for better simulation performance and stability.
