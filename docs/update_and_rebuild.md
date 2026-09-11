# Pull GitHub changes and rebuild

Use this workflow after the repository has been changed on GitHub. Run each
command separately in the terminal.

## Normal update workflow

First stop the running ROS 2 launch with `Ctrl+C`. If an old Gazebo process
did not close, you can use:

```bash
pkill -f gz
```

Then update the local repository:

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git status
git switch main
git pull --ff-only origin main
```

If you intentionally want a feature branch, replace `main` with that branch
name.

Rebuild and source the workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

Launch the updated Gazebo simulation:

```bash
ros2 launch omron_cobra_s600_description gazebo.launch.py
```

## One-time dependency installation

You do not need to install this package after every GitHub update. Install it
only if it is missing:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-bridge
```

The complete dependency list is in the main README. Running `rosdep install`
after pulling is recommended because a GitHub update may add a new dependency.

## Verify that the new version is active

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git log -1 --oneline

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix omron_cobra_s600_description
```

The package prefix should point to:

```text
~/ros2_ws/install/omron_cobra_s600_description
```

## If `git pull` reports local changes

Do not overwrite your work. Inspect it first:

```bash
cd ~/ros2_ws/src/omron_cobra_s600_description
git status
git diff
```

Either commit your changes before pulling, or temporarily store them:

```bash
git stash push -u -m "local work before GitHub update"
git pull --ff-only origin main
git stash pop
```

After `git stash pop`, inspect any reported merge conflicts before rebuilding.

## Clean rebuild only when necessary

A normal `colcon build` is usually enough. If ROS 2 still uses stale package
files, remove only this package's generated build products and rebuild:

```bash
cd ~/ros2_ws
rm -rf build/omron_cobra_s600_description
rm -rf install/omron_cobra_s600_description
source /opt/ros/jazzy/setup.bash
colcon build --packages-select omron_cobra_s600_description --symlink-install
source install/setup.bash
```

Do not delete the source directory under
`~/ros2_ws/src/omron_cobra_s600_description`.
