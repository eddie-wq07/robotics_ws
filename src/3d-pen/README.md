# 3D pen source history and compatibility

The active application now lives in the modular ROS 2 packages alongside this
checkout. Start with [the combined architecture](../../docs/architecture.md)
and the workspace scripts:

```bash
cd ~/robotics_ws
./scripts/install-pen.sh
./scripts/start-pen.sh fake
```

The browser uses the installed image/path nodes and C++ execution action server.
Preview must complete for the current inputs before Execute unlocks. Fake mode
runs the pipeline without connecting to the physical arm. Real calibration
remains in `pen_description/config/calibration.template.yaml` with TBD values.

## Active source owners

| Responsibility | Location |
| --- | --- |
| Image extraction and inspectable 2D drawing | `../pen_image_processing/` |
| Plane/sphere geometry | `../pen_surface_geometry/` |
| Timed toolpath, curved travel, and extrusion events | `../pen_toolpath/` |
| ROS messages and actions | `../pen_interfaces/` |
| C++ job execution and controller switching | `../pen_execution/` |
| Tool, holder, jig, surface, and calibration | `../pen_description/` |
| Servo firmware and behavior | `../pen_servo/` |
| Browser, launch files, and application configuration | `../pen_bringup/` |
| Images, meshes, generated jobs | `../../assets/` |

This Git repository retains the original development history, examples,
attribution, and legacy preview integration. `COLCON_IGNORE` excludes it from
the workspace build. `src/tactile/relief.py`, `surface.py`, and `slicer.py` are
compatibility imports of the active packages rather than independent algorithms.
The `setup.sh` entry point delegates to the workspace installer.

## Legacy MoveIt/RViz preview

`rviz.sh`, `ros/preview.py`, `docs/ros.md`, and the original `tactile prepare` CLI
belong to the older mock-only MoveIt preview. They do not execute the new pen jobs
and are not the active startup path. In particular, the historical installation
instructions target a different workspace/vendor revision; use the workspace
installer above on Mocha.

The legacy algorithms remain covered by `tests/`. After installing the new
packages, run them explicitly with the shared scientific environment:

```bash
cd ~/robotics_ws
PYTHONPATH="$PWD/src/3d-pen/src" .venv-pen/bin/python -m pytest -q src/3d-pen/tests
```

The previous browser/robot-planning stack is preserved in Git commit `90802bd`.
Selected browser assets are reused by `pen_bringup`; the obsolete custom IK and
robot planner are not part of the new execution path. See [attribution](THIRD_PARTY.md).
