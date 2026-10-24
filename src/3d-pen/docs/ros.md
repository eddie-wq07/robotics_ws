# ROS setup — Ubuntu 22.04

This is the only robot stack: ROS 2 Humble + Flexiv ROS 2 + MoveIt 2 + RViz. The current launcher is mock hardware only; Gazebo and physical execution are deferred.

1. Install [ROS 2 Humble desktop](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
2. Build Flexiv using its [official instructions](https://github.com/flexivrobotics/flexiv_ros2/tree/783e1ab1379458e7dd4979d67f88f054d326f7d3). Use the pinned Rizon/RDK-v1 revision below, not the default RDK-v2 branch.

```bash
sudo apt install -y build-essential cmake git python3-venv python3-colcon-common-extensions python3-rosdep python3-vcstool wget
source /opt/ros/humble/setup.bash
mkdir -p ~/flexiv_ros2_ws/src
cd ~/flexiv_ros2_ws/src
git clone --branch humble-v1 https://github.com/flexivrobotics/flexiv_ros2.git
git -C flexiv_ros2 checkout 783e1ab1379458e7dd4979d67f88f054d326f7d3
cd ~/flexiv_ros2_ws
vcs import src < src/flexiv_ros2/flexiv.humble.repos --recursive --skip-existing
touch src/flexiv_rdk/COLCON_IGNORE
# Run `sudo rosdep init` once if rosdep has never been initialized on this PC.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
cd src/flexiv_rdk/thirdparty
bash build_and_install_dependencies.sh ~/flexiv_install
cd ..
cmake -S . -B build -DCMAKE_INSTALL_PREFIX="$HOME/flexiv_install"
cmake --build build --target install --config Release
cd ~/flexiv_ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH="$HOME/flexiv_install"
```

The pinned upstream manifest selects `flexiv_description` `humble-v3.1` and RDK `v1.9.4.1`. No custom ROS package or colcon build is needed for this project's single Python adapter.

3. From this repository, run `./setup.sh`, prepare an image, then launch:

```bash
.venv/bin/tactile prepare examples/apple.png --out runs/apple
./rviz.sh runs/apple/job.json
```

`FLEXIV_WS=/path/to/workspace ./rviz.sh ...` supports a different Flexiv workspace location. The launcher sets local ROS domain 73 and uses the dummy serial `Rizon4s-000000`. The pinned model prefixes its links/group with that serial; `ros/preview.py` uses the same names.

In RViz, the MoveIt display shows the robot, fixed fixture, attached tool, and successful planned trajectory. Add **MarkerArray → /tactile/path** to see requested pen strokes. Inspect `runs/apple/moveit_report.json` for `PLANNED` or `REFUSED`; the report includes the input job's hash. Do not confuse the requested waypoint overlay with a successful motion plan.

The approach uses MoveIt's planning service; the drawing/lift path uses collision-aware Cartesian interpolation. Partial Cartesian results are rejected. MoveIt's preview timing is not calibrated extrusion timing. No trajectory execution service is called.

**Status:** this adapter has been checked for syntax and its input geometry tested offline on macOS. Actual ROS service calls, the imported scene, and resulting motion still need integration validation on this Ubuntu setup.

## Using a Mac

**Yes, RViz can run on Apple Silicon.** [RoboStack](https://robostack.github.io/GettingStarted.html) supplies ROS packages for macOS through Pixi. This is separate from this repository's Ubuntu-only Flexiv launcher.

To try RViz itself, first install Pixi using the linked instructions, then in a new terminal:

```bash
pixi init ~/rviz-mac --channel https://prefix.dev/robostack-humble --channel conda-forge
cd ~/rviz-mac
pixi add ros-humble-desktop
pixi run rviz2
```

This opens the RViz application; it does not install the Flexiv robot or run this project's MoveIt adapter. These Mac commands are documented from RoboStack, not tested on this machine.

For our full workflow, use the Ubuntu PC and view its desktop from the Mac. Another option is an Ubuntu 22.04 ARM virtual machine, with graphics and Flexiv dependencies to validate. Native Mac support for our entire stack would need a separate integration pass.

RViz displays robot models, camera data, and trajectories. MoveIt computes robot motion and checks collisions. Mock hardware supplies joint states without moving the real arm; none of these models the pen's molten plastic.
