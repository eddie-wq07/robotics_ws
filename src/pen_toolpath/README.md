# Pen toolpath

Compile an inspectable `pen_drawing` and calibrated plane/sphere into a timed
Cartesian job. Geometry, timed samples, extrusion events and preview share the
same generated path. The compiler uses rest-to-rest quintic segments and evaluates
the analytic surface at each sample; it checks intersample chords and a conservative
pen/holder capsule against the workpiece. Full-arm and fixture collision planning
is not provided: real execution additionally requires recorded physical clearance
and route qualification.

Run the installed scientific worker with the isolated Python environment:

```bash
.venv-pen/bin/python -m pen_toolpath.worker --request request.json --out assets/outputs/job-id
```

Example request:

```json
{"image_path":"/absolute/path/apple.png","size_mm":20,"surface":"sphere","execution_mode":"fake","profile_path":"/absolute/path/fake.yaml"}
```

Outputs: drawing.json, drawing.png, relief.json/png, picture.png, toolpath.json,
job.json, report.json and request.json. Set `drawing_path` instead of `image_path`
to compile a previously extracted drawing. Missing real calibration is rejected.
The scientific libraries do not connect to ROS or the robot.

The pen tip defaults to **1.3 mm above the physical surface**, measured along the
outward surface normal. The active sphere profile uses **1.25 mm** clearance;
values below `0.00125` on the sphere or `0.0013` on the plane are rejected. Lift and
air-run offsets add to that clearance. Surface geometry itself remains at the
physical surface; the compiler applies the tip clearance once, then converts
the tip pose to the RDK TCP pose using the calibrated transforms. The legacy
preview slicer also starts its first layer at 1.3 mm clearance.

Browser requests also provide `robot_preview`: the current URDF joint chain,
starting joint positions and CAD tip transform. The worker uses seeded,
bounded Jacobian IK (as in the original pen preview) to generate
`robot_preview.json`. It includes approach, drawing, retract and return joint
frames, synchronized with the job's extrusion phases. Failed IK rejects the
preview rather than displaying a disconnected trace. Joint limits, velocity
limits and pose errors are checked; collisions remain unqualified. These
preview joint frames are never submitted to the hardware executor.

ROS `path_generation_node` wraps this worker with the `GeneratePath` action.
Parameters: `worker_python`, `worker_timeout_s` (180), `jobs_root`. It uses ROS
system Python and scrubs PYTHONPATH before spawning the scientific environment.

Validation:

```bash
env -u PYTHONPATH .venv-pen/bin/python -m pytest -q src/pen_toolpath/test src/pen_image_processing/test
source ~/robotics_setup/env.sh
source install/setup.bash
ROS_DOMAIN_ID=91 /usr/bin/python3 src/pen_toolpath/test/ros_actions_smoke.py
```

The smoke test starts only the two processing nodes, sends an image action followed
by a path action, and confirms artifact handoff. It does not start a robot driver.

Physical command frame: the current Flexiv hardware sends the ROS API's `world`
coordinates directly to the RDK base frame. Real profiles must therefore have
identity `T_world_base`; translated or rotated world/base relationships are
rejected until command and feedback frame conversion is implemented together.
Preview/fake profiles can still represent other frame relationships.

Drawing speed is `speed_m_s`; non-extruding speed is `travel_speed_m_s`.
`drawing.local_speed_limits: true` uses a speed profile along each continuous
stroke, slowing near tight curvature rather than applying its worst bend's speed
to the whole stroke. Starts, stops and pen recycle boundaries still stop, and the
existing sampled motion checks remain enforced. It defaults to false and is
enabled for the Tylenol moon profile only.
Active profiles set `drawing.sample_rate_hz: 1000` to match the 1,000 Hz Cartesian target publisher.
The rounded stroke path uses a smooth cubic curve between its samples, so
1,000 Hz sampling does not introduce velocity corners at the old polyline joins.
`drawing.draw_speed_scale: 1.5` and `drawing.non_extruding_speed_scale: 3.0`
multiply moving portions of those schedules. Non-extruding scaling also applies
to the saved joint approach, connecting motion, lowering, lifting, transfers,
and return. Pen lead/release dwells and the 15-second recycle interval with its
2-second pause remain unchanged. The independently retimed Go home command
remains at its already configured 3x setting.
Both accept metres/second and are checked separately against their configured
limits. If no travel speed is supplied, travel uses the drawing speed.
Motion settings are independent in each surface's calibration profile:
`drawing.speed_m_s`, `drawing.travel_speed_m_s`, `drawing.approach_speed_scale`,
and `pen.stand_off_m`. The flat venue profile's base speeds are 0.012 m/s drawing,
0.030 m/s travel, 2x initial setup, and 0.0013 m tip stand-off. The sphere
profile's base speeds are 0.004 m/s drawing, 0.0075 m/s travel, 1x initial setup, and
0.00125 m tip stand-off. A sphere entry-only transition may start below the
raised travel height, but must already meet the drawing stand-off and pass
the same sampled surface/tool clearance checks.

With a saved joint approach, generation computes its endpoint TCP using the
calibrated robot model and includes a checked Cartesian segment from that
endpoint to the saved Cartesian entry. Thus the first executable Cartesian
sample matches where the joint approach actually ends; preview-only connecting
motion cannot hide a start-pose mismatch from the executor.

Continuous extrusion is split at 15 seconds (`pen.recycle_interval_s`), including
the press lead time. The compiler stops at a point on the same stroke, releases
the pen, and holds the exact pose for 2 seconds (`pen.recycle_pause_s`) before
pressing again. The recycle press lead then elapses before motion continues.
The current press lead is **0.8 seconds** after lowering at a stroke start;
`pen.recycle_press_delay_s: 0.5` preserves the **0.5-second** lead after a recycle.
Continuous strokes use a short acceleration phase, constant
cruise speed, and braking at the next deliberate stop instead of a quintic
speed ramp spread over the entire stroke. Acceleration and curvature limits
still apply; there is no instantaneous jump to full speed.
At the end of each stroke, `pen.end_extrusion_dwell_s: 0.5` keeps extrusion
on at the final point for 0.5 seconds total before release. Extrusion then turns
off and lifting begins at the next motion sample, without an extra off dwell.
This extra endpoint dwell does
not run at mid-stroke recycle pauses; it is included in the 15-second press
budget, with an earlier recycle if necessary to leave room.
There is no lift during a recycle pause. Each split uses a rest-to-rest segment;
short strokes and normal lifts between strokes are unchanged.
The current profiles use `pen.release_delay_s: 0.0`; the explicit off sample
still precedes lifting at the end of each stroke.

### Joint-controller trajectory output

For `execution_backend: joint`, the worker requires a serial-specific
`robot_kinematics` file pinned by SHA-256. It writes `joint_trajectory.json`
and binds its digest into `job.json`. Display-only kinematic corrections are
rejected. The generated preview uses the same retimed waypoints.

The scientific worker needs `toppra==0.6.10` (install in `.venv-pen`, not ROS
system Python). TOPPRA retimes a continuous-velocity cubic joint path under
velocity and acceleration limits. A virtual clock coordinate prevents shorter
source dwells or higher requested path speed. The final ROS cubic is checked
at its exact velocity/acceleration extrema; interpolated tip deviation is also
checked. This establishes numerical path checks, not collision qualification.

Joint jobs use continuous stroke timing with small tangent corner blends
(up to 0.25 mm trim) instead of restarting a quintic at every outline vertex.
Factory-model IK uses tighter numerical tolerances so sampling does not create
false stationary runs. The final schedule preserves two-second stationary
recycle pauses; periodic continuous sections are aligned to the requested
15-second interval by slowing motion, never by inserting an early hold.

Joint plans now bias redundant IK motion toward the checked approach posture
and reject poor conditioning at both sampled and interpolated positions. The
scaled Jacobian uses a 0.15 m rotation weight, a maximum condition number of 60,
and a minimum singular value of 0.015. A rejected path needs a different approach,
placement, or posture; reducing speed alone does not satisfy this geometric check.
# Downward pen orientation

Drawing paths keep a steady heading and point the pen vertically downward
wherever the calibrated tool envelope permits. A smooth, minimal tilt toward
the surface normal is added where needed for capsule clearance and the
maximum deviation from the local surface normal configured by
`drawing.max_surface_normal_angle_deg` (default **25 degrees**). Valid values
are 0 through 89 degrees; the Tylenol profile sets **35 degrees**. This is a
maximum, not a requested tilt; tool-envelope clearance can require a tighter
angle. There is no
fixed sphere printing-region angle cap. Surface mapping, pen/holder clearance,
joint reachability and motion limits still apply. The selected tip stand-off is preserved, and TCP positions and
motion timing are recomputed from the pen-tip transform and actual orientation.
Approach and return transitions still connect smoothly to the recorded poses.
