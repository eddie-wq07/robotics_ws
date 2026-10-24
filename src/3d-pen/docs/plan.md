# Minimal architecture

**Goal:** one tactile design printed on the jig-mounted hemisphere.

```text
Picture
  ↓
Python: relief + curved pen strokes
  ↓ job.json (geometry, poses, draw/travel segments)
One ROS adapter: scene + planning requests
  ↓
MoveIt 2: approach, IK, collision checking
  ↓
RViz: assembly and planned-motion playback
```

| Keep now | Owner |
| --- | --- |
| Picture → raised lines/fill | Existing image code |
| Known hemisphere curvature + jig placement | YAML configuration / supplied CAD |
| Pen-tip offset + attached tool geometry | Configuration; physical calibration pending |
| Robot model and mock controller | Official Flexiv ROS 2 integration |
| Robot planning and collision checks | MoveIt 2 |
| Visualization | RViz |

**Two commands:** `tactile prepare IMAGE --out runs/NAME`, then `./rviz.sh runs/NAME/job.json` on Ubuntu.

**Current implementation:** geometry export works locally; ROS adapter and launcher are written, but live integration has not been tested on Ubuntu. A generated job is not a validated robot plan. The adapter rejects incomplete MoveIt paths and never executes them.

**Real CAD:** seven [Onshape STLs](../assets/source/onshape/README.md) are included. The actual sphere has 40 mm radius and tabs; the current preview's 75 mm hemisphere is nominal. Activate the real parts only with matching curvature and mounting transforms.

**Leave out for now:** browser UI/API, custom IK/collision planner, simulated probing, Gazebo physics, arbitrary scans/USDZ, camera correction loops, AI control, and hardware execution. Camera/servo mounting envelopes remain in the scene for clearance checking.

**Next:** validate RViz/MoveIt on the Ubuntu PC → replace nominal CAD and calibrate tip/jig → bench-test extrusion → add one hardware execution path with synchronized pen control. Add D405 feedback only when the first physical print works.
