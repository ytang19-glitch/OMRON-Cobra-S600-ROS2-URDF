# Mesh files

The URDF expects these four mesh files in this folder:

- `fixed-base.STL`
- `link1.STL`
- `link2.STL`
- `link3.STL`

These are the original STL files supplied for the OMRON/Adept Cobra s600 model. They are binary STL files, so they are not embedded by the chat GitHub connector used to create this repository.

After cloning the repository, copy the four STL files into this directory without changing their filenames. The URDF already applies `scale="0.001 0.001 0.001"` because the meshes use millimetres.
