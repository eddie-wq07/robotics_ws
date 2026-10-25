# Start the real Tylenol demo

Use these steps after a reboot or when the stack is stopped. Leave both terminals
running. If the driver and app are already running, just refresh the existing
browser tab—do not launch another copy.

Before starting: release the E-stop, select **AUTO (REMOTE)**, and keep the arm
and pen stationary and free of contact. Driver startup enables the arm and
zeroes the force sensor; it does not home the robot.

## 1. Terminal 1 — robot driver

```bash
cd ~/robotics_ws
source ~/robotics_setup/env.sh
export ROS_DOMAIN_ID=0
export FLEXIV_CARTESIAN_LINEAR_STIFFNESS_N_M=10000

if pgrep -f '[/]opt/ros/humble/lib/controller_manager/ros2_control_node' >/dev/null; then
  echo "Driver already running; do not start a second copy."
else
  ros2 launch flexiv_cartesian_controller bringup.launch.py \
    robot_sn:=Rizon4s-063112 use_fake_hardware:=false start_rviz:=false
fi
```

Wait for the robot to become operational and the controllers to activate.

## 2. Terminal 2 — pen app and web server

This starts the camera, image/path services, executor and web server together.
Do not run it if that app stack is already running.

```bash
cd ~/robotics_ws
source ~/robotics_setup/env.sh
export ROS_DOMAIN_ID=0
ros2 launch pen_bringup app.launch.py \
  execution_mode:=real execution_backend:=cartesian \
  robot_sn:=Rizon4s-063112 \
  profile:="$PWD/src/pen_description/config/tylenol_sun.yaml" \
  port:=8081 commissioning_verified:=true
```

`commissioning_verified:=true` uses the checks already completed for this setup;
it does not perform a new physical calibration.

## 3. Browser

Refresh the existing tab at **http://127.0.0.1:8081/**.

- `sun.png` selects `tylenol_sun.yaml`.
- `moon.png` selects `tylenol_moon.yaml`.
- Wait for preparation to finish. Use **Go home**, **Warm up pen** and **Execute**
  yourself when ready; startup does not run them automatically.

If only the web server is stopped and the camera/services/executor are still
running, use this **instead of step 2**:

```bash
cd ~/robotics_ws
PEN_HTTP_PORT=8081 ./scripts/start-tylenol-real.sh
```

After an E-stop, an `unconfigured` hardware state needs a driver restart: stop
the old driver with **Ctrl+C** in its terminal, then repeat step 1 after releasing
the E-stop and restoring AUTO (REMOTE). Refresh the browser afterward.
