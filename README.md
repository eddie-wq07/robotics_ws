# TODOS

- Eddie: Develop simple, effective pitch.
    i)
    ii) Positioning with AI generated questions a judge might ask on first ask. //
    iii) Write Devpost submission
    iv) Begin collecting feedback -> iteration  //
    v) Video submission
    vi) Slideshow


- Sam: Finalize the pipeline. 4 objects running highly consistently
    - Review entire repo
    - otpimize sun demo
    - add moon demo


- Things to get from office:
    a) Ethernet cables
    b) Mics
    c) Yellow PLA

# Flexiv 3D pen workspace

The modular application turns an image into a surface-following drawing job, previews it in the browser, and executes through the existing ROS 2 controllers. The agreed design and commissioning boundary are in [docs/architecture.md](docs/architecture.md).

## Install and start on Mocha

```bash
cd ~/robotics_ws
./scripts/install-pen.sh
./scripts/start-pen.sh fake
```

Open <http://127.0.0.1:8080>. The fake mode runs in ROS domain 73 and does not construct a Flexiv SDK robot. Select/upload a JPG or PNG, choose a surface and size, then press **Preview**. **Execute** unlocks after that preview succeeds. Changing inputs requires a new preview. **Stop** cancels the job; closing the browser causes the execution heartbeat to expire. The playback rate affects only visualization.

For extraction and preview without a controller manager, use `./scripts/start-pen.sh preview`. Execute stays disabled in this mode. To use the browser through SSH, forward the loopback port, for example `ssh -L 8080:127.0.0.1:8080 ubuntu@<mocha-address>`, then open the same URL on your computer. Set `PEN_HTTP_PORT` or `PEN_ROS_DOMAIN_ID` explicitly if another application already owns the defaults.

## Source and data

- `src/pen_image_processing`: image to inspectable 2D drawing.
- `src/pen_surface_geometry`: analytical surface mapping.
- `src/pen_toolpath`: sampled poses, timing, clearance checks, and extrusion events.
- `src/pen_interfaces`: ROS action contracts.
- `src/pen_execution`: C++ job executor, controller switching and DO0 requests.
- `src/pen_servo`: C++ behavior tests and independent ESP32 firmware.
- `src/pen_description`: tool/fixture model and calibration profiles.
- `src/pen_bringup`: browser and launch integration.
- `assets/meshes`: canonical STLs and provenance; `assets/inputs`: source images.
- `assets/outputs/<job>`: drawing, toolpath, exact execution job, report, and result.

Saved run snapshots are published through Git LFS in
[`assets/archives`](assets/archives/README.md). That directory includes restore
instructions. Source files, configuration profiles, launch scripts, dependency
requirements, documentation, and reusable assets are tracked directly; local
builds, installed packages, virtual environments, and caches are excluded.

An input image can have a sibling `<image filename>.strokes.json` file to retain
explicit marks that the feature simplifier would otherwise remove. Its version-1
object contains `image_sha256` and `strokes_px` (lists of pixel-coordinate
polylines). These strokes use the image's crop and scale, appear in both relief
and toolpath previews, and are included in input-change checks before execution.
Curated designs may also set `replace_extracted`, supply `crop_xyxy`, and add
`fills_px` polygons (`shell`, optional `holes`, `name`, `hatch_angle_deg`). These
regions get a perimeter and 0.9 mm hatch spacing for the 1 mm strand, even when
general image filling is off. The venue design outlines the water and island,
fills only the central dome, and retains the explicit X. Rebuild its vector
sidecar with `scripts/build-venue-map.py`; preview at 79.89 mm on the flat surface (about 55% of its face by rectangular footprint).

The browser retains the original 3d-pen four-panel styling and viewer helpers. Preview now solves a bounded joint trajectory using the vendor URDF and the combined CAD's pen tip. Play animates approach, drawing, lifts, retract and return; the saved arm snapshot supplies only the starting pose. `robot_preview.json` stores this simulation separately from the executable `job.json`. Joint limits and kinematic tracking are checked; full-arm collisions are not. The old `src/3d-pen` checkout preserves its Git history and remains excluded from colcon.

Panel 1 shows the physical D405 camera in every mode, including simulation.
`app.launch.py` starts the shared `camera_feed` service on localhost port 8082;
standalone browser runs can start it with `python3 -m pen_bringup.camera_feed`
after sourcing the robotics environment. It uses the installed RealSense Python
SDK and OpenCV, captures 640 x 480 RGB at 15 FPS, and never connects to the arm.
Browsers share this camera owner and display an offline state if frames stop.
During execution, panels 2 and 3 follow the executor sample index and fresh ROS
joint states; a saved pose is not substituted for missing live joint feedback.

**Go home** uses `rizon_arm_controller` to return to the user-recorded seven-joint
Home target saved in `~/.config/flexiv/home_pose.yaml` and the measured profile.
It does not switch to Cartesian control or invoke the Flexiv Home primitive.
The return is checked against joint limits and the tool capsule's clearance to
the fitted plane; if the direct joint path is too close to the plate, the UI asks
for the tool to be lifted clear. The same executor owns Stop and heartbeat
handling. **Reset** affects playback only. No home move occurs automatically.
The drawing's approach/park route is a separate, previously checked route.

## Measurements and physical operation

Step-by-step commissioning and real-run commands: [Pen real-robot runbook](docs/real_robot_runbook.md).

Sam's measurements belong in a copy of `src/pen_description/config/calibration.template.yaml`. Unknown values are explicit null/TBD, never assumed zero. `fake.yaml` contains synthetic test geometry only. Real execution is gated by complete calibration, qualified routes, and hardware commissioning; see [the operator guide](docs/robot_startup.md). Build and browser startup do not flash the ESP32. Firmware instructions are in `src/pen_servo/README.md`.

Keep `commissioning_verified=false` for now. Automatic readback of the active RDK tool/TCP/payload and comparison with the loaded description remain pending. The compiler checks the selected surface and a tool envelope; fixture and full-arm collision clearance still require qualification. The browser demo's synthetic values do not fill these gaps.

## Validation

```bash
source ~/robotics_setup/env.sh
source ~/robotics_ws/install/setup.bash
colcon test --packages-select pen_execution pen_description pen_servo pen_bringup
colcon test-result --verbose
env -u PYTHONPATH .venv-pen/bin/python -m pytest src/pen_image_processing/test src/pen_surface_geometry/test src/pen_toolpath/test
```

Fake hardware verifies software integration, not physical reachability, extrusion, calibration, collision avoidance, or real-time deadlines.

The browser regression runs against a started fake application, using the locally installed Chrome:

```bash
env -u PYTHONPATH .venv-pen/bin/python -m pip install playwright==1.64.1
env -u PYTHONPATH .venv-pen/bin/python scripts/test-pen-browser.py
```

Validation records and browser screenshots are under `~/robotics_setup/validation/pen`; firmware/description evidence is in `~/robotics_setup/validation/pen-hardware`. After Stop, the executor deliberately leaves both arm controllers inactive. Restart the fake application for another complete demo. Preview and software builds never connect to the physical robot.
