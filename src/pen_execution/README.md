# Pen execution

The `executor` C++ ROS node consumes a reviewed `pen_job` artifact through
`/pen/execute` (`pen_interfaces/action/ExecuteDrawing`). Goal fields are
`job_path`, the SHA-256 of the exact file bytes in `expected_sha256`, and
`air_run`. Air runs require a separately compiled/previewed air-offset artifact;
the executor does not offset an already-reviewed path.

`execution_mode` defaults to `preview`, which rejects execution. `fake` requires
the loaded hardware plugin to be `mock_components/GenericSystem`, matching the
configured robot serial. It performs a clearly reported synthetic preposition,
then runs the same sampled Cartesian job. It does not validate real kinematics.
`real` additionally requires `commissioning_verified=true`, a measured profile,
unchanged profile hash, qualification flags, and qualified approach/park routes.
Calibration remains a separate, supervised task.

Real jobs provide `profile_path` and `profile_source_sha256` for the measured
profile file, `qualification` flags `rt`, `clearance`, `pen`, and `tool`, and
`joint_routes.approach` / `joint_routes.park`. Each route contains
`qualified: true`, seven `start_positions` in radians, and `points` containing
seven `positions` plus strictly increasing `time_s`. Start positions must match
measured joints within 0.03 rad. These flags record commissioning evidence; they
do not obtain that evidence. Automatic RDK tool-name/payload readback is not
implemented by this node. Until that existing-hardware-owner capability and
physical commissioning are verified, keep `commissioning_verified` false.

The node attaches to an existing controller manager; it never starts a driver
or opens an SDK connection. Target publication is best-effort depth one at
1,000 Hz (one target every 1 ms). The active drawing profiles also compile at
1,000 Hz. Joint/Cartesian switching retains the existing 1 kHz hardware loop;
measured Cartesian feedback remains at its configured 100 Hz.
On Cartesian activation, the hardware owner sets X/Y/Z stiffness from the active
profile via `FLEXIV_CARTESIAN_LINEAR_STIFFNESS_N_M` (10,000 N/m in the restored profile),
keeps the robot's nominal rotational stiffness, and uses the SDK's nominal damping.
The setter runs before entering RT mode; it is not called in the streaming loop.
The configured TCP position tracking-error limit is 5 mm (`tracking_limit_m: 0.005`).
The pen output changes only pin 0 through the existing GPIO controller.
No output is commanded on process startup or artifact preview.

Dense filled jobs are bounded to 500,000 samples and 128 MiB, matching the
planner's sample cap. These loading limits do not change motion, tracking,
clearance, or extrusion-time limits.

Interfaces:

- `/pen/execute`: execution action and progress/result.
- `/pen/heartbeat`: `std_msgs/Empty`, required at least once per two seconds
  while a job is active; generate only while the controlling client is live.
  Initial preflight waits at most the configured heartbeat timeout for the
  first fresh heartbeat, without sending hardware commands. Afterward expiry
  stops the job; there is no recurring grace period.
- `/pen/stop`: `std_srvs/Trigger`, requests stopping; inspect final status for
  confirmation.
- `/pen/execution_status`: diagnostics with phase, mode, progress, sample index,
  requested extrusion, job hash and concrete error.
- `/pen/execution_pose`: measured TCP for the browser.

On cancellation/error, command publication stops, pin 0 is requested low, joint
goals are canceled, and arm controllers are deactivated if this executor owns
them. No automatic return movement or restart occurs. A failed hardware link
can retain DO0; the ESP32's independently tested release behavior is necessary.
Reported GPIO state is a request, not physical button/flow feedback.

Jobs are restricted to `jobs_root` and are loaded entirely before motion.
Relative times, pose finiteness, quaternion norm, sample gaps/jumps, phase rules,
pen schedule agreement, and file hashes are checked before commanding hardware.
Every extrusion transition must have an explicit event. The validator enforces
the configured stationary press/release delays and maximum continuous press
duration before accepting a job; these settings come from the compiled profile.
Physical commissioning flags are evidence gates, not a substitute for obtaining
the calibration and timing evidence described in `docs/architecture.md`.

Verification without physical hardware:

```bash
source ~/robotics_setup/env.sh
colcon test --packages-select pen_execution
python3 src/pen_execution/test/fake_integration.py
```

The integration harness starts a mock driver on local ROS domain 91 and checks
completion, pin-0-only output changes, modified-job rejection, cancellation,
and client-heartbeat loss. Set `PEN_TEST_JOB` to an existing fake job JSON to
exercise a full compiled image path. It prints its temporary evidence directory
and stops only the processes it started.

## Joint-controller simulation

The executor supports `execution_backend:=joint_simulation` only with
`execution_mode:=fake`. It verifies mock hardware, then sends the generated
`robot_preview.json` trajectory to `rizon_arm_controller` using
`FollowJointTrajectory`. The Cartesian controller stays inactive. The preview
file is bound to the job by SHA-256. Stop and browser-heartbeat cancellation
remain active, and feedback drives the existing live UI.

Use an isolated ROS domain for this test. The fake launch accepts `profile`
and `execution_backend` arguments; a recorded Home in the profile initializes
the fake joints. This does not qualify generated joint trajectories for the
physical robot or model physical compliance/vibration.

When preparing another joint simulation, the preview may include a smooth
joint move from the current fake pose to the saved approach start. This
simulation-only connector is not permitted for physical jobs.

## Factory-model joint execution

`execution_backend:=joint` accepts a separately generated, SHA-bound
`joint_trajectory.json`. Physical jobs require the matching serial-specific
factory kinematics, a measured start TCP within 0.5 mm of the model, and the
existing commissioning and controller ownership checks. The generator uses
continuous-velocity cubic interpolation and locally increases segment durations
to limit joint velocity to 0.5 rad/s and acceleration to 1 rad/s². Interpolated
path deviation is checked to 0.5 mm. Browser Stop, heartbeat, joint goal
constraints and measured TCP tracking checks remain active.

A profile's `joint_execution.air_offset_m` prepares an extrusion-off raised
check. Extrusion requires `joint_execution.drawing_enabled` and a retimed pen
schedule that still meets its press/recycle limits. These are explicit modes;
simulation previews alone never authorize physical motion.

The ROS update loop is 1,000 Hz with FIFO priority 50. The current hardware
position interface uses Flexiv `SendJointPosition` in `NRT_JOINT_POSITION`, so
this is not an end-to-end RT joint streaming implementation.

Joint execution now requires the joint controller to claim both position and
velocity interfaces so the hardware forwards the planned velocities to RDK.
Exception guards issue release/cancel before large trajectory objects are
destroyed. Physical trajectories must contain the new posture validation;
older artifacts must be regenerated. These changes require a controller/driver
restart and do not constitute physical qualification of the modified command path.

## Pen warmup

The UI's **Warm up pen · 30s** button calls `/pen/warmup` (`std_srvs/srv/Trigger`).
It can be requested with a cold or hot pen; a hot pen may start extruding on that first press.
The button stays available and explains when an active operation must be stopped first.
The executor checks hardware identity, an active GPIO controller, an inactive
Cartesian controller, and stationary joint feedback. It issues one 0.2-second
press and releases, then waits until 30 seconds from the press. No arm target,
controller activation/switch, or second servo press is issued. The 30 seconds
is operator-selected, not a temperature measurement or manufacturer guarantee.

Warmup owns the executor until it completes or stops. `/pen/stop`, cancellation
through the UI, expired browser heartbeat, or detected arm movement aborts it
and requests servo release. Status phases are `WARMUP_STARTING`, `WARMUP_PRESS`,
`WARMUP_HEATING`, and `WARMUP_COMPLETE` / `WARMUP_STOPPED` / `WARMUP_FAILED`;
`warmup_id` identifies the specific request. The UI preserves the reviewed
drawing and asks the operator to check temperature and click Execute afterward.
Warmup never starts automatically, including on startup or before a drawing.

The browser's **Go home · 3×** uses the current measured pose and saved
calibration directly, without requiring a prepared drawing. It cancels pending
preview preparation, checks a fresh joint route to the taught Home, and retains
the same geometric clearance samples while retiming that route to one-third
of the original duration (up to 0.45 rad/s). It checks robot joint velocity
limits and refuses an unsafe return. Active robot operations must be stopped
first. Completion regenerates the drawing preview from Home.

## One-use raised clearance check

`clearance_check_sha256` is empty by default. An operator-reviewed job with
`purpose: air_clearance_check` can be enabled for one execution by pinning its
exact SHA-256 at executor startup. This is a commissioning motion, not a completed
surface qualification. It requires the existing commissioned real Cartesian
executor and existing RT/pen/tool qualifications; only the unproven surface
clearance and regular drawing approach/park are replaced by the complete approved
air path. No normal profile qualification flags are changed.

Before motion, the executor independently enforces all extrusion off, a planar
surface, a 19 mm minimum tool-capsule clearance, a 250 mm local workspace, a
180-second duration, 6 mm/s linear speed, 0.1 rad/s angular speed, 0.15 m/s²
acceleration, return to the initial pose, and a 0.5 mm match to the measured start.
Normal heartbeat, Stop, tracking and controller checks remain active. The permit
is consumed before the controller switches; retrying requires explicit rearming.
