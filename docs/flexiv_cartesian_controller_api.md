# Flexiv Cartesian controller: API and configuration

For local startup, controller switching and shutdown, follow
[the operator guide](robot_startup.md). Keep both documents local and uncommitted.

The package is implemented and built. Fake-hardware API and switching checks
pass. Real joint-control bringup and FIFO priority 50 were verified. Physical
Cartesian operation, core isolation and sustained RT deadlines remain unqualified.

## Architecture

- Package: `~/robotics_ws/src/flexiv_cartesian_controller`
- Plugin: `flexiv_cartesian_controller/FlexivCartesianController`
- Instance: `flexiv_cartesian_controller`

The plugin shares the existing Flexiv hardware interface and its single SDK
connection with `rizon_arm_controller`. The controller manager runs
`read → update → write` at 1000 Hz. In Cartesian mode, hardware `write()` calls
`StreamCartesianMotionForce()` in `RT_CARTESIAN_MOTION_FORCE` mode. All axes
initially use motion control; wrench and velocity/acceleration feedforward are zero.

Cartesian activation sets X/Y/Z stiffness from `FLEXIV_CARTESIAN_LINEAR_STIFFNESS_N_M`
(10,000 N/m in the restored pen profile; nominal if unset),
retains nominal rotational stiffness, and uses the SDK default damping ratio.
The existing hardware owner calls `SetCartesianImpedance()` in NRT Cartesian
mode before starting RT streaming.

The robot's onboard controller performs Cartesian control and IK. There is no
CRISP dependency, host-side control law, interpolation or trajectory planner.
ROS callbacks run outside the update path. A fixed-size latest-target mailbox
connects the subscriber to the controller update. Pose publication uses a
realtime publisher with a separate publishing thread.

Real hardware activation automatically runs `ZeroFTSensor` through the existing
SDK owner before any controller starts. Launch with the mounted tool stationary
and free of external contact. Zeroing returns to stopped `IDLE`; controller
switches do not repeat it. Failure or expiry of its 10-second completion deadline
aborts startup. SDK calls can block beyond that deadline. Live startup zeroing,
a three-second current-pose hold, and target-withdrawal timeout passed on
2026-10-24; evidence is in
`~/robotics_setup/validation/pen-hardware/hold-timeout-result.json` and
`commissioning-driver.log`. This does not establish sustained timing or drawing
motion qualification.

## ROS interfaces

Names below assume the default namespace and robot serial on Mocha.

| Interface | Type | Behavior |
| --- | --- | --- |
| `/flexiv_cartesian_controller/target_pose` | `geometry_msgs/msg/PoseStamped` | Continuous target input; only the latest accepted pose is streamed |
| `/flexiv_cartesian_controller/tcp_pose` | `geometry_msgs/msg/PoseStamped` | Measured pose, nominally 100 Hz, only while active and not stop-latched |
| `/flexiv_cartesian_controller/status` | `diagnostic_msgs/msg/DiagnosticArray` | Status at 10 Hz while configured, including inactive/active/stop-latched state and counters |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | Standard controller switching |
| `/Rizon4s_063112/tcp_pose` | `geometry_msgs/msg/PoseStamped` | Existing vendor broadcaster; measured pose available with either controller selected |
| `/joint_states` | `sensor_msgs/msg/JointState` | Existing combined joint-state output |
| `/flexiv_rizon_arm/joint_states` | `sensor_msgs/msg/JointState` | Existing arm broadcaster output before aggregation |

The vendor pose prefix is the serial with `-` replaced by `_`. The Cartesian
plugin's topic names are relative to its controller node; the loader can target
an existing namespaced manager. Preserve deployment-specific remappings.

Target QoS is **best effort, volatile, KeepLast(1)**, with DDS lifespan set to the
command timeout. Cartesian measured-pose output uses sensor-data QoS. Status
reports cumulative `rejected_targets`, `late_cycles_over_1_5ms`, and `max_period_ns`.
It also reports `last_rejection_reason` and cumulative `rejected_<reason>` counters
for invalid frame, invalid timestamp, stale timestamp, invalid pose,
pre-activation target, expired queued target, translation jump, and rotation jump.
The drawing executor includes the last reason in its target-rejection error.
The status counters describe controller update periods, not end-to-end robot latency.

## Target contract

- Position is in metres, in the configured robot-world frame (`world` by default).
  The controller does not transform frames.
- ROS orientation order is `(x, y, z, w)`. The plugin maps it to SDK
  `[x, y, z, qw, qx, qy, qz]` and normalizes valid quaternions.
- Publish nonzero, current timestamps from the same synchronized system-time
  domain. `use_sim_time=true` is rejected. Future and expired stamps are rejected.
  Timestamp age is checked against the current ROS clock on receipt. The update
  loop then adds elapsed steady-clock time since receipt to that recorded age.
  It does not compare message stamps with the earlier control-cycle timestamp:
  a valid target may arrive while that cycle is already running. Transport and
  queue age together must fit the unchanged `command_timeout` budget.
- Non-finite values, near-zero quaternions, wrong frames and excessive translation
  or orientation jumps from measured TCP are rejected without renewing freshness.
- The publisher supplies smooth targets and enforces trajectory speed/acceleration
  limits. Jump checks alone are neither trajectory limits nor collision avoidance.
- Inactive messages are ignored. Activation seeds measured pose and rejects
  commands from before that activation. Keep sending fresh targets after switching.
- The update loop streams at its configured rate independently of message arrival.
  A missing or stale target latches a stop; the hardware stops the robot and a
  non-RT supervisor requests controller deactivation. Reactivation must be explicit.
  SDK stop is blocking and is outside healthy streaming.

## Parameters and defaults

Parameters are read on configure; reconfigure to apply changes.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `resource` | Generated as `${robot_sn}_tcp` | Cartesian hardware resource |
| `joints` | Generated as `${robot_sn}_joint1` … `joint7` | Seven joint-position exclusion resources |
| `controller_manager` | `/controller_manager` | Manager used for stop/deactivation requests |
| `world_frame` | `world` | Required target frame and measured output frame |
| `update_rate` | `1000` | Requested controller update rate; manager must also run at 1000 Hz |
| `require_rt_scheduling` | `true` | Reject activation unless the update thread uses SCHED_FIFO |
| `command_timeout` | `0.1` s | Initial target grace and accepted-command receipt timeout |
| `max_translation_jump` | `0.05` m | Maximum target distance from measured TCP |
| `max_rotation_jump` | `0.35` rad | Maximum target orientation difference |
| `max_cycle_gap` | `0.02` s | Stop if an observed update period exceeds this gap |
| `state_rate` | `100.0` Hz | Nominal Cartesian measured-pose publication rate |

The shared manager explicitly uses `thread_priority: 50` and `update_rate: 1000`
in `flexiv_bringup/config/rizon_controllers.yaml`. Both controllers use that
same thread. Priority and permissions do not guarantee 1 ms deadlines on the
generic kernel. Do not disable the scheduling requirement for physical testing.

## Build and launch configuration

```bash
source ~/robotics_setup/env.sh
cd ~/robotics_ws
colcon build --packages-select flexiv_description flexiv_hardware flexiv_cartesian_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

Restart the driver after changing its library or hardware description.

Combined `bringup.launch.py` accepts `robot_sn`, `rizon_type`,
`use_fake_hardware`, `start_rviz`, and `config_file`. Identity defaults come from
`FLEXIV_ROBOT_SN` / `FLEXIV_ROBOT_TYPE` in `~/.config/flexiv/robot.env` through
`~/robotics_setup/env.sh`. It always starts joint control and loads Cartesian
control inactive.

To load only the inactive plugin into an already-running rebuilt hardware driver:

```bash
ros2 launch flexiv_cartesian_controller load_controller.launch.py
```

This loader additionally accepts `prefix` and `controller_manager` for other
existing deployments. These overrides are not arguments of the combined bringup.

For fake-hardware startup use `robot_sn:=Rizon4s-000000 use_fake_hardware:=true`.
A fake-hardware switching run without FIFO can use a custom YAML setting
`require_rt_scheduling: false`; the physical setup retains the default `true`.

## Persistent RT permissions

Configuration files on Mocha:

- `/etc/security/limits.d/99-robotics-realtime.conf`: RT allowance 50 and unlimited memlock.
- `/etc/systemd/system/user@1002.service.d/50-robotics-realtime.conf`: matching user-manager limits.

Desktop logins inherit the permissions; already-running shells and tmux servers
retain their previous limits. Real-hardware combined bringup checks its shell's
RT allowance before starting. See the operator guide for checking permission
versus the actual running thread policy. No kernel or core-isolation change was applied.

## Validation evidence

Evidence lives under `~/robotics_setup/validation/cartesian/`:

| File | Verified behavior |
| --- | --- |
| `offset-test-result.json` | Offset publisher preview, +/-3 cm bounds, return to start, Ctrl+C and distance cap on fake hardware |
| `result.json` | Fake switching, mutual exclusion, measured pose, quaternion mapping, input rejection, timeout and scheduling guard |
| `combined-bringup-result.json` | Joint active / Cartesian inactive startup with fake hardware |
| `fifo-bringup-result.json` | Fake Cartesian/joint switching under FIFO priority 50 |
| `real-bringup-result.json` | Historical real joint-control startup with Cartesian inactive and FIFO priority 50 |

Real shutdown subsequently logged `IDLE` and clean process exit. These are
historical checks; inspect current state before operation. The hardware uses
one SDK state snapshot per cycle, but SDK copies and other active components
still need timing qualification. GPIO changes retain blocking SDK behavior.

The 2026-10-23 audit added GPIO lifecycle and partial-command regression coverage,
SDK I/O exception handling, and validation of every offset-test feedback pose
(finite values, unit quaternion, frame, and timestamp). GPIO now uses a realtime
mailbox and publisher, preserves unspecified outputs, and requires fresh commands
after reactivation. Its SDK write remains blocking. The hardware returns an error
and disables further arm streaming after an SDK read/write failure; physical
failure-path behavior still requires supervised validation.

The current read-only probe also reports the exact readiness reason, E-stop,
reduced-state, and recovery flags. See the operator guide for GPIO commands and
the remaining physical-network/RT qualification requirements.

Plan: `~/robotics_setup/plans/2026_1023_minimal_cartesian_rt_plan.md`.


## Offset test publisher

`ros2 run flexiv_cartesian_controller cartesian_offset_test` previews a route
from measured TCP pose. Add `--execute` to authorize physical motion and
controller switching. See the [local test procedure](robot_startup.md#optional-small-cartesian-offset-test).
This external Python publisher generates a cosine ramp; the controller and SDK
streaming loop are unchanged. It is a supervised test utility, not a planner
inside the controller. It retains initial orientation, caps displacement to
3 cm on one axis at a time, and restores joint control after successful completion.
