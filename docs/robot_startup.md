# Mocha robot: startup, switching and shutdown

Use this guide from Mocha's local desktop. Startup is manual after every PC
reboot. The installed software, robot identity and RT permissions persist;
the driver and tmux sessions do not. No robot boot service is configured.

## Network and read-only connection check

The robot connects directly to Mocha's **enp3s0** Ethernet interface.
The manual IPv4 addresses, checked 2026-10-24, are **192.168.2.51** for the
robot and **192.168.2.101/24** for Mocha (netmask **255.255.255.0**). Keep the
wired gateway and DNS empty and MTU at **1500**. The persistent NetworkManager
profile is **Wired connection 1**; Wi-Fi supplies the internet/default route.
Do not switch the wired profile back to DHCP. The robot-side physical port was
not independently identified after recabling.

`~/robotics_setup/env.sh` loads the workspace and then the validated deployment
under `~/robotics_setup/validation/joint-safety-fix/install`. This matches the
running hardware, executor, and calibration packages; the base workspace install
alone contains older profile validation. Keep that deployment directory available.
The standard pen and Tylenol preview shortcuts apply this environment last.

The RDK and ROS driver discover **Rizon4s-063112** by serial number. They do not
need a robot IP parameter, and the local DDS configuration contains no old IP
addresses. The tablet may show **192.168.100.1**, the separate General Port
address; use **192.168.2.51** for diagnostics of this configured direct link.

Check the network:

```bash
ip -brief address show enp3s0
ip route get 192.168.2.51
ping -I enp3s0 -c 5 192.168.2.51
```

The route should show `dev enp3s0 src 192.168.2.101`. Existing RDK/ROS
processes can retain sockets bound to the old local address after a network
change. A stale real driver must be stopped before a standalone SDK probe;
restarting real bringup enables the arm and zeroes F/T, so do that only with
explicit operating authorization and the preparation below. Re-sourcing
`env.sh` alone does not repair sockets in a running process.

With no other SDK client or robot driver running, verify SDK communication:

```bash
source ~/robotics_setup/env.sh
timeout -k 3 20 ~/robotics-venv/bin/python ~/robotics_setup/validation/robot-connectivity.py
```

This check only connects and reads state; it does not enable the arm or set
outputs. Results are saved in `~/robotics_setup/validation/robot-connectivity.json`.
The direct-link SDK check on 2026-10-24 succeeded with all seven joints and
advancing timestamps; at that check the robot was stopped, idle, not operational,
and had no faults. The existing driver and affected ROS application nodes were
then restarted with user authorization. Driver enabling and F/T zeroing completed;
joint control was active, Cartesian inactive, and all three real-control UIs
reported an idle, ready executor with fresh joint and pose feedback. No drawing
or motion test was requested. Treat these as historical checks, not live status.
Evidence: `~/robotics_setup/validation/direct-network-20261024/result.json`.
Communication can work before the robot is ready for motion.

`operational_status: OperationalStatus.NOT_ENABLED` means communication is
working but the servos have not been enabled. The probe does not enable them.

For a one-shot control-box DO0-high command with no ROS driver running:

```bash
~/robotics_ws/scripts/set_do0_high.sh
```

With the ROS driver already running, use its GPIO controller instead of opening
another SDK client:

```bash
source ~/robotics_setup/env.sh
ros2 topic pub --once /Rizon4s_063112/gpio_outputs flexiv_msgs/msg/GPIOStates \
  '{states: [{pin: 0, state: true}]}'
```

Use `state: false` to command low. Only explicitly specified pins change. The
GPIO controller leaves outputs unchanged on activation/deactivation and ignores
commands while inactive; reactivation requires fresh commands. Successful SDK
delivery is not a measurement of physical voltage. Confirm attached equipment
and the required inactive/output state before use.

## Physical real-time qualification still required

The direct-link check on 2026-10-24 measured five pings at
**0.183/0.220/0.268 ms** minimum/average/maximum, with no packet loss. These
samples are below the vendor's sub-1 ms round-trip requirement. This verifies
short-term reachability, not sustained latency, bandwidth, or the complete
1 kHz control cycle. The earlier router-path measurement is superseded.
The generic kernel has no reserved cores; CPU 14 and sibling 15 remain
unisolated. FIFO priority 50 alone does not qualify loop deadlines.

Qualification must include hardware reads/writes, all broadcasters, and network
traffic. The vendor state broadcaster still performs ordinary ROS publishes from
its update callback, and GPIO changes call a blocking SDK method. These remain
sources of timing variation. Do not change digital outputs during a timed RT run.
The reported fake-hardware passes do not validate physical motion or failure
recovery. Tool/TCP calibration, payload, collision geometry, and live hold/stop
behavior must match and be verified on the actual installation.

Vendor references: [system requirements](https://www.flexiv.com/software/rdk/manual/v1.x/system_requirements.html)
and [network configuration](https://www.flexiv.com/software/rdk/manual/v1.x/connect_user_computer.html).

## 1. Prepare

Power on the robot and confirm **AUTO (REMOTE)** in Flexiv Elements. The
Ethernet connection and RDK license must be available. The existing hardware
driver enables the arm and may clear faults during startup.

Every real hardware startup also zeroes the F/T sensor automatically. Keep the
robot and mounted tool stationary and free of external contact, including the
plate, before launching. After enabling, the existing hardware owner enters
`NRT_PRIMITIVE_EXECUTION`, runs `ZeroFTSensor`, waits for `terminated=1`, then
returns to stopped `IDLE` and refreshes measured state before controllers start.
No homing is performed and ordinary controller switches do not repeat zeroing.
Zeroing failure, lost readiness, shutdown, or expiry of its 10-second completion
deadline aborts startup, with a best-effort stop and no zeroing retry. Individual
SDK calls are blocking; the deadline does not impose a hard overall time limit.
Live startup zeroing and a three-second Cartesian hold followed by target timeout
passed on 2026-10-24. Timeout left both motion controllers inactive and the robot
in `IDLE`. Evidence: `~/robotics_setup/validation/pen-hardware/hold-timeout-result.json`
and `commissioning-driver.log` in that directory. Explicit controller deactivation
and return to joint control also passed, followed by a +/-5 mm Z test at five
seconds per leg. That run recorded 0.373 mm maximum tracking error, 0.110 mm final
position error, and 17 rejected targets requiring investigation. Evidence:
`stop-execute-result.json` and `z-execute-result.json` in the same directory.
The application's Stop service, drawing motion, and sustained timing qualification
remain separate checks.

Press **Ctrl+Alt+T** and run:

```bash
source ~/robotics_setup/env.sh
ulimit -r
pgrep -af '[/]opt/ros/humble/lib/controller_manager/ros2_control_node'
```

`ulimit -r` should print **50**. It is permission to request real-time priority;
ordinary programs still use normal scheduling. If it prints `0`, log out of
the Ubuntu desktop and back in, then open a new terminal. Old tmux servers
retain their old limits. Real-hardware bringup rejects an allowance below 50.

The `pgrep` command should print nothing. If a driver exists, use that instance
or [stop it](#5-stop) before starting another.

## 2. Launch and verify

```bash
ros2 launch flexiv_cartesian_controller bringup.launch.py
```

Leave this terminal running. To include RViz, append `start_rviz:=true`.

| Component | Startup state |
| --- | --- |
| `rizon_arm_controller` | Active — default joint control |
| `flexiv_cartesian_controller` | Loaded, inactive |
| Shared update loop | 1000 Hz, SCHED_FIFO priority 50 |
| RViz | Off by default |

Open a second terminal with **Ctrl+Alt+T**:

```bash
source ~/robotics_setup/env.sh
ros2 control list_controllers
ps -eLo pid,tid,cls,rtprio,comm | grep ros2_control
```

Expect the controller states above and one update thread showing **FF / 50**.
Other threads may show `TS`. Switching controllers retains this same loop and
priority. Core isolation and sustained deadline qualification remain pending.

## Optional: small Cartesian offset test

Start the normal bringup above and leave joint control active. Clear the motion
area and keep the physical stop accessible. This test moves in **robot-world
X/Y/Z**, keeps the initial orientation, and returns to its starting pose after
each positive or negative offset.

Preview a 3 cm X-axis test in a second local terminal:

```bash
source ~/robotics_setup/env.sh
ros2 run flexiv_cartesian_controller cartesian_offset_test --axis x --distance 0.03
```

The preview reads current measured pose and prints the route. To execute it:

```bash
ros2 run flexiv_cartesian_controller cartesian_offset_test --axis x --distance 0.03 --execute
```

The helper automatically switches to Cartesian control, sends smooth targets
at 100 Hz, moves `start → +3 cm → start → −3 cm → start`, and restores joint
control on successful completion. Each leg takes 4 seconds by default. The
native controller continues its 1 kHz SDK streaming.

Test `--axis y` and `--axis z` separately next. After validating each axis,
`--axis all` runs X, then Y, then Z; it does not command a diagonal move.
For a smaller first movement, use `--distance 0.005` (5 mm). Distances above
3 cm and leg times below 3 seconds are rejected.

Do not manually activate the Cartesian controller before this helper: it expects
joint active / Cartesian inactive. The helper stops on stale pose feedback,
controller stop status, or tracking error over 15 mm. On Ctrl+C or an error it
stops target publishing and requests Cartesian deactivation; it does not command
an automatic return movement. Check controller state before continuing.

## 3. Select Cartesian control

Prepare a publisher that sends fresh, smooth targets continuously using the
[API contract](flexiv_cartesian_controller_api.md#target-contract). The Cartesian
controller's own `tcp_pose` output is silent while inactive. The existing robot
state broadcaster provides the current measured pose before switching:

```bash
ros2 topic echo /Rizon4s_063112/tcp_pose --once
```

Use measured state to initialize the publisher. Do not use a zero/default pose
as a starting target. Physical Cartesian operation remains unqualified; begin
with supervised current-pose holding.

With the publisher ready and continuing to send fresh targets:

```bash
ros2 control switch_controllers --strict \
  --deactivate rizon_arm_controller \
  --activate flexiv_cartesian_controller
ros2 control list_controllers
```

Expected: Cartesian active, joint inactive. The hardware enters
`RT_CARTESIAN_MOTION_FORCE`. Messages received before activation are ignored;
a fresh accepted target must arrive within **100 ms**, and targets must keep
arriving within that timeout. Otherwise the controller requests a stop and
deactivates. See the API document for limits, QoS and timing details.

## 4. Return to joint control

While Cartesian control is active:

```bash
ros2 control switch_controllers --strict \
  --deactivate flexiv_cartesian_controller \
  --activate rizon_arm_controller
ros2 control list_controllers
```

If a timeout already left both controllers inactive, activate only joint control:

```bash
ros2 control switch_controllers --strict --activate rizon_arm_controller
```

Inspect controller states and hardware logs after a failed switch before retrying.
A failed request does not prove that the robot restored its previous mode.

## 5. Stop

Press **Ctrl+C** in the launch terminal and wait for shutdown. The log should
show the robot entering `IDLE`, successful hardware deactivation, and the
controller-manager process exiting. Verify:

```bash
pgrep -af '[/]opt/ros/humble/lib/controller_manager/ros2_control_node'
```

Expected: no output. This stops ROS control; the robot cabinet remains powered.
Use the robot's physical controls for cabinet shutdown or emergency stopping.

## Optional: run the same manual launch in tmux

From a local terminal with RT allowance 50 and no existing driver:

```bash
~/.local/bin/tmux new-session -d -s robot \
  'bash ~/robotics_setup/launch-robot-headless.sh'
tail -f ~/robotics_setup/robot-headless.log
```

Ctrl+C exits `tail`, while the driver continues in tmux. Stop the driver with:

```bash
~/.local/bin/tmux send-keys -t robot:0.0 C-c
```

Then verify shutdown as above. Tmux keeps the process running when the terminal
window closes, but not across a PC reboot. An older tmux server may require
restarting after the desktop login to inherit the RT permissions.


## If startup waits for controller-manager services

Look at the earlier driver output, not only the repeated spawner warnings.
If the SDK connects but startup stops after `Enabling robot ...`, the robot
has not become operational. Check Flexiv Elements: select AUTO (REMOTE),
check the E-stop and resolve the indicated readiness condition.

New driver launches reject MANUAL mode or a pressed E-stop with a clear error.
The readiness wait after Enable is limited to 30 seconds and reports status
flags on failure. Correct the robot state, then restart bringup. The SDK calls
themselves are blocking, so this is a readiness-wait limit, not an overall
connection timeout.

Run the offset-test preview only after `ros2 control list_controllers` works
and shows joint control active. A missing measured pose can be a consequence
of failed bringup, not a wrong topic name.
