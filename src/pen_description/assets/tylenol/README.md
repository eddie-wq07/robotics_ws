# Tylenol scan

Imported from `USB DISK/tylenol.glb` on 2026-10-24. The replacement source
passes PNG integrity and full image-decode checks for all three embedded 8K
textures. `manifest.json` records source/runtime hashes, sizes, geometry hashes,
material texture sizes, bounds and transforms.

- `source.glb`: unchanged full-quality scan with 8192 × 8192 textures.
- `tylenol.glb`: simulation visual with 2048 × 2048 PNG textures. Geometry,
  UVs, normals, tangents, indices, node transforms and material settings are
  preserved. Normal-map vectors are renormalized after texture resizing.
- `preview.html`: standalone material viewer using the workspace's Three.js.

The mesh has 904,415 vertices and 1,785,900 triangles. Mesh-local dimensions
are approximately 87.63 × 56.40 × 58.20 mm under glTF's metre convention.
The operator accepted the CAD dimensions and placement for real drawing after a raised air pass. No contact measurement of the bottle was performed. Preserve the
embedded node rotation when importing; glTF scene coordinates are Y-up.
For a ROS Z-up scene, apply a +90-degree X rotation to the imported scene root.
Do not apply that conversion twice.

## Inspect without starting ROS

From the workspace root:

```bash
python3 -m http.server 8094 --bind 127.0.0.1
```

Open <http://127.0.0.1:8094/src/pen_description/assets/tylenol/preview.html>.
The viewer uses neutral environment lighting and the embedded base-color,
normal, occlusion, roughness and metalness maps. It does not connect to hardware.
For a remote workstation, forward port 8094 through SSH.

## Use in simulation

The `pen_description` package installs the runtime GLB and manifest under
`share/pen_description/assets/tylenol`. The ROS resource URI is
`package://pen_description/assets/tylenol/tylenol.glb`; use a glTF-capable
renderer to retain its embedded PBR materials. This import does not add the
object to the current robot scene or change calibration. Mesh simplification,
collision geometry, mass and inertia remain separate preparation steps if
needed. This dense scan is a visual asset, not a qualified collision mesh.

Regenerate the smaller visual from the workspace root, using the existing
Python environment with Pillow and numpy:

```bash
env -u PYTHONPATH .venv-pen/bin/python \
  src/pen_description/scripts/prepare_scan_asset.py \
  src/pen_description/assets/tylenol/source.glb \
  src/pen_description/assets/tylenol/tylenol.glb
```

The original is about 273 MB. Hosting it on GitHub requires Git LFS or external
source-asset storage; no Git staging or LFS policy was changed. The damaged
first import is retained outside the repository under
`~/robotics_setup/validation/tylenol-import/damaged-source.glb` for diagnosis.

## Jig assembly and sun preview

`holder-source.stl` is the original `tylenol_holder2.stl` from USB;
`holder.stl` is its millimetre-to-metre conversion. `assembly.json` records
hashes and the transforms also stored in `../../config/tylenol_sun.yaml`.
The holder's four 3.5 mm holes are at CAD X=±46 mm, Z=7.9/21.9 mm.
After a 90-degree X rotation and translation `[0,14.9,72.5]` mm,
they align with the jig pattern at X=±46 mm, Y=±7 mm. The inner cradle
radius is 28 mm. Mechanical fastener fit is not established by this alignment.

The bottle is horizontal as seen in the shared D405 camera. Its scanned body
was initially fitted over source X=-10..20 mm. For the 36 mm sun, the
checked region is X=-11..35 mm, with a maximum scan radius of 29.495 mm
about that fitted axis. The planning cylinder uses 29.6 mm radius and a
46 mm axial region; its center shifts 7 mm toward the bottle base. The
physical bottle and holder placements are unchanged. The operator confirmed the bottle was secured in the simulated placement and that the raised real path was centered and visibly clear. Placement remains based on operator-accepted CAD geometry; the camera is contextual evidence. The scan and holder are separate rigid visuals
in one `pen_surface` link, preserving textures.

From the workspace root, run `scripts/start-tylenol-preview.sh` and open
<http://127.0.0.1:8095/>. It starts no ROS bridge or robot connection.
The profile selects `sun.png`, 36 mm, and the **Tylenol + holder** dropdown.
`sun.strokes.json` preserves both discs and all eight rays; its active copy is
`assets/inputs/sun.png.strokes.json`. Flat and 80 mm sphere use their own saved
profiles when selected in this preview server.

`moon.png` is an alternative 36 mm crescent outline on the same bottle surface.
Selecting `sun.png` loads `tylenol_sun.yaml`; selecting `moon.png` loads
`tylenol_moon.yaml`. Both retain the same calibration, approach and Home route.
The sun keeps 15 mm/s drawing, 45 mm/s travel, 1.9 mm clearance and a 35-degree
angle limit. The moon requests 4 mm/s drawing, 45 mm/s travel and 2 mm clearance,
with the same angle limit. Its `drawing.local_speed_limits: true` permits faster
motion along smooth arcs while slowing for tight bends, within the existing
velocity, acceleration and orientation checks. The image pipeline extracts one
closed centreline with no fill; its shape is unchanged. The sun remains the
launcher default.
The moon's checked surface path takes about 53 seconds; the approach and Home
return add time. Local speed limits still slow tight bends more than open curves.
The PNG source and required attribution are recorded in
[`moon-source.md`](moon-source.md).

The planner wraps the image around a horizontal cylinder and checks sampled
chords, the conservative tool capsule, velocity/acceleration and the configured
normal-angle limit (`drawing.max_surface_normal_angle_deg`, currently 35 degrees).
The robot preview checks IK and joint limits. It has no
full-arm or holder collision qualification. The real profile uses the executed joint approach and a checked return to the recorded Home. It retains the shared hardware qualification and documents the operator-accepted CAD placement.
Camera snapshot, screenshots and validation output are under
`~/robotics_setup/validation/tylenol-assembly`.

## Preparation progress and saved checks

While the robot trajectory is checked, the Pen strokes panel shows the actual
fraction of drawing samples checked and an approximate time remaining based on
the current check rate. Approach, return and artifact-saving stages are labelled
separately; their bars are indeterminate when no reliable work count is available.

Successful preview and real results are saved under `assets/outputs/.preview_cache`.
An unchanged request can reuse them after refreshing the UI or restarting the
preview service. Reuse requires matching image/annotation hashes, profile,
settings, robot kinematics, planner source and numerical dependency versions.
Preview-only starting positions must match exactly; real starting positions
must match within 0.005 rad per joint, as for explicitly saved jobs.
Cached artifacts are hash-verified before use;
each browser request still gets its own result and review token. The most recent
eight results are retained, without removing original job folders. No cached
result initiates robot motion or replaces the executor's live checks.

## Real UI

Run `scripts/start-tylenol-real.sh` and open <http://127.0.0.1:8095/>. This connects
to the existing ROS executor; it does not start the driver or command motion.
The selected 36 mm sun uses 15 mm/s drawing, 45 mm/s travel and 1.9 mm nominal
tip clearance. Execute starts the extrusion-enabled drawing after final checks.

An optional `prepared_trajectory` directory in a profile can supply an explicitly
saved job, preview and hash manifest. Otherwise completed preparations use the
automatic cache described above. Matching inputs reload with a fresh review
token; changed settings require preparation again. The real executor still
verifies the reviewed file, current state, exclusive controller ownership and
tracking limits before motion.

A real 10 mm raised, extrusion-off pass completed successfully on 2026-10-24.
The operator confirmed centering and visible clearance. Evidence is in
`assets/outputs/tylenol-air-20261024T192943Z/`. This supports the accepted CAD
placement; it does not constitute contact probing or complete arm collision analysis.
