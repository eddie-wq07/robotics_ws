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

The pen tip defaults to **2 mm above the physical surface**, measured along the
outward surface normal. `stand_off_m` values below `0.002` are rejected. Lift and
air-run offsets add to that clearance. Surface geometry itself remains at the
physical surface; the compiler applies the tip clearance once, then converts
the tip pose to the RDK TCP pose using the calibrated transforms. The legacy
preview slicer also starts its first layer at 2 mm clearance.

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
