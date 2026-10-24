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
100 Hz. Joint/Cartesian switching retains the existing 1 kHz hardware loop.
The pen output changes only pin 0 through the existing GPIO controller.
No output is commanded on process startup or artifact preview.

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
