# Pen cell description

The imported [Tylenol scan](assets/tylenol/README.md) includes a validated PBR
visual, full-quality source, inspection manifest and standalone material preview.
The runtime GLB is installed under `share/pen_description/assets/tylenol`.

This package composes the vendor Flexiv arm, combined holder/pen, and fixed
jig/workpiece into one URDF. It opens no robot connection. `pen_tip` is the
physical tip and `pen_rdk_tcp` records the separately calibrated active SDK TCP.
Neither a mesh nor a TF link configures the robot's actual TCP or payload.

Canonical STL assets and their provenance manifest live in
`../../assets/meshes` relative to this package. CMake verifies metre units and
hashes and installs the assets into `share/pen_description/meshes`, so URDF
references are portable `package://pen_description/meshes/...` URLs. Rebuild
after changing the canonical assets.

## Calibration

`config/tylenol_sun.yaml` and `config/tylenol_moon.yaml` store the shared
bottle/cradle calibration and independent 36 mm sun/moon selections. Run `scripts/start-tylenol-preview.sh` from the workspace
to open its dedicated preview at port 8095. The holder is registered to the jig's
CAD hole pattern; bottle seating and clocking remain inferred. The real profiles retain the operator-accepted placement and reviewed approach/Home
routes; see the Tylenol asset README for qualification limits.

`config/sphere.yaml` is the complete saved real-robot configuration
restored at the 1000 Hz / 5 mm tracking-limit checkpoint. It includes all measured
transforms, tool and payload data, fixture registration, Home and approach routes,
pen timing, drawing motion, selected artwork/fill, executor parameters, Cartesian
stiffness, and browser/camera settings. The local Mocha startup scripts read this
file; the older local `drawing-profile.yaml` path links to it. Driver stiffness
is applied on Cartesian activation and requires a driver restart after editing.

`config/calibration.template.yaml` is the real-machine template: unknown
measurements remain null/TBD. Sam can fill the measured values in a local copy
and set `calibrated: true` after validation. `config/fake.yaml` contains
explicit synthetic values for testing only. The loader rejects that profile in
real mode and rejects missing, non-finite, or non-unit transforms.

Each `T_A_B` stores the pose of frame B in A: `xyz_m` and ROS quaternion order
`quaternion_xyzw`. `T_world_surface` is the canonical planner transform: the
sphere **center** or plane **drawing face**, not the mesh bounding-box origin.
The renderer derives the jig-to-surface joint from `T_world_base`, `T_base_jig`,
and `T_world_surface`. This keeps the composed model and planning coordinates
consistent, including a rotated base. Sphere radius is 0.040 m; flat STL
thickness is 0.0025 m and its visual/collision origin is adjusted accordingly.
Real execution currently requires `T_world_base` to be identity: the controller
passes `world` target coordinates directly to the RDK's robot-base frame. A
shifted or rotated base is supported for preview and fake-model checks only.
Real plane profiles also require explicit `surface.size_m: [0.08, 0.10, 0.0025]`
to match the installed flat mesh.

`T_flange_holder`, `T_holder_pen`, `T_flange_pen_tip`, and
`T_flange_rdk_tcp` remain separate: CAD origins do not determine tip calibration.
The active `pen_holder_combined.stl` already contains the seated pen. `pen_body`
remains an empty compatibility frame, preventing a second pen from rendering.
`payload`, pen timing, qualified routes, and execution qualification accompany
the profile but are not magically applied to hardware by rendering the model.
The `qualification` booleans (`rt`, `clearance`, `pen`, `tool`) all start false;
they are evidence gates consumed by execution, not substitutes for the tests.
`routes.approach` and `routes.park` stay null until taught and checked. Each
qualified route records `start_positions` for seven joints and timed
`points: [{time_s, positions}]`, plus `qualified: true` after actual validation.

`camera.T_flange_camera` is also TBD. It locates the D405's bottom screw mount
relative to the flange; when measured, set `camera.enabled: true` to compose
the installed RealSense D405 model. Nominal optical extrinsics are disabled so
the camera driver can publish its measured internal transforms without a
second publisher. The description does not start the camera driver.

The default fake profile includes the vendor D405 mesh on the rear drilled plate,
looking toward the workpiece, and a generic blue micro-servo with a white horn
beside the pen. These placements are approximate. `visual_servo.T_holder_servo`
sets the preview servo pose; `camera.T_flange_camera` in `fake.yaml` sets the
preview camera pose. Real camera measurements remain unset in the measured
profile. Neither accessory starts a device or operates the servo.
The D405's rear face is centered between the rear plate's two holes at holder
`[92, 0, -45]` mm, facing along the pen; fastener compatibility is not established.

## Render without hardware

```bash
source ~/robotics_setup/env.sh
ros2 run pen_description render_description \
  --profile ~/robotics_ws/src/pen_description/config/fake.yaml \
  --mode fake --robot-sn Rizon4s-000000 --ros2-control > /tmp/pen-cell-fake.urdf
```

The existing combined driver launch accepts `description_file:=...` for a
pre-rendered URDF. The application's bringup should render and validate first,
then pass that file to the single existing driver. A fake URDF must only be
used with fake mode. The renderer does not launch drivers, publish duplicate
robot-state trees, or move the robot.

## Browser appearance and CAD mounting

The browser loads the vendor OBJ/MTL materials, including the gray arm bodies
and white covers. `config/visualization.yaml` records a display-only holder
registration against the vendor flange mesh. The new combined STL is converted
from millimetres to metres and retains its assembled pen orientation. The holder has four 6.5 mm
clearance holes and the flange has four 5 mm holes, both on a 50 mm bolt circle.
The mating faces are coincident; maximum center mismatch in the supplied
triangulated meshes is 0.0074 mm. The source mesh hashes guard this registration.

This registration only affects synthetic browser models. It does not change
the URDF used by controllers or any calibration profile. The fourfold bolt
pattern leaves physical clocking ambiguous. The combined CAD fixes pen seating
and points the tip outward along the flange axis; its CAD tip is approximately
224.7 mm beyond the flange face. Physical TCP calibration remains separate.

The TCP offset `[224.709, 2.292, -1.993]` mm is stored in
`config/calibration.mocha.yaml` in the user's tool-flange measurement frame;
224.709 mm points outward. It is control calibration, not a CAD placement or
browser annotation. The active RDK TCP still requires verification.

The viewport contains geometry only; measurement labels and legends are omitted.
The jig pose is fitted to four user-taught pen-tip contacts at the CAD groove
bottoms, local `(X,Y,Z) = (+/-50,+/-50,73.75)` mm. The fit is a rigid transform;
CAD dimensions are preserved. The fitted jig origin in robot base coordinates
is `[510.375, -2.115, -14.040]` mm, with roll/pitch/yaw
`[-0.786, -0.174, 89.922]` degrees. RMS contact residual is 0.684 mm and the largest
residual is 0.951 mm. The recorded contacts and fit evidence are referenced by
`config/calibration.mocha.yaml`.

The flat's CAD drawing face remains at jig Z=75 mm, with a 90-degree local turn.
Its fitted center is base `[509.346, -2.341, 60.953]` mm; the measured plane tilt
is retained. Drawing clearance is 1.3 mm along that plane's outward normal.
`fake.yaml` uses the same fitted jig/surface geometry, and `visualization.yaml`
retains the CAD-only flat/sphere mounting transforms. Previous approach and park
routes are marked unqualified because they predate this fixture registration.

The CAD registration does not fill other missing calibration values. Use **Tool mount**
below the viewport to inspect the holder. Robot materials retain the vendor's
stored diffuse colors under neutral, white-balanced lighting. STL tooling
colors remain approximations until physical color references are supplied.

`visualization.yaml` can also hold a timestamped `joint_snapshot` read from the
physical arm. The synthetic browser uses its seven radians-valued joints to
pose the vendor model; the header identifies it as a saved snapshot. It never
publishes these values to controllers. Real-mode rendering continues to use
live joint feedback. Read-only source evidence is retained under
`~/robotics_setup/validation/pen/robot-joint-snapshot.json`.
