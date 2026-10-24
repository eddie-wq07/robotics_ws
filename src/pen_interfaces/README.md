# Pen interfaces

`/pen/prepare_drawing` (`PrepareDrawing`) and `/pen/generate_path` (`GeneratePath`)
accept a JSON request and an output directory underneath the node's `jobs_root`.
Both report stage/progress, support cancellation, and return an artifact path.

Image request: `image_path`, `size_mm`, `fill`. Path request: `drawing_path`,
`surface` (`sphere` or `plane`), `profile_path`, `execution_mode` (`preview`,
`fake`, `real`) and optional `settings`. The path worker also accepts `image_path`
for standalone end-to-end use. Path generation preserves existing image artifacts.

`/pen/execute` (`ExecuteDrawing`) accepts `job_path`, the SHA-256 of that
exact file (`expected_sha256`), and `air_run`. Result and cancellation status
must be checked; goal submission alone does not imply execution succeeded.

A version-1 `pen_job` is JSON with `frame: world`, metre/second units, ROS XYZW
quaternions, calibration identity, `samples` and `pen_events`. Samples contain
`time_s`, `pose: [x,y,z,qx,qy,qz,qw]`, `phase` and `extrude`. Time starts at zero
and increases strictly. Extrusion is allowed only during `DRAW`. Every lift is
preceded by a `RELEASE` interval and an explicit false pen event. Events coincide
with sample times. First/last samples have extrusion off.

`profile_sha256` identifies canonical parsed profile contents;
`profile_source_sha256` identifies the raw `profile_path` file used at compile
time. `expected_sha256` always hashes the complete job file's raw bytes.

Synthetic jobs set `fake_only: true`, `calibrated: false`. None of these messages
or JSON schemas authorizes hardware execution by itself.
