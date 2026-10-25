# Drawing profile selection

The UI discovers drawing profiles in `src/pen_description/config/*.yaml` and
`*.yml`. Each drawing profile has a `surface` and a `selection` block:

```yaml
selection:
  image: eddiemom.png
  images: [eddiemom.png, another_portrait.png]  # optional additional matches
  surface: photo_board
  size_mm: 120
  fill: false
```

Existing profiles map `eddiemom.png` to `photo_board.yaml`, `jackolantern.png` to
`sphere.yaml`, `venue_map_marked.png` to `flat.yaml`, and `sun.png` to `tylenol.yaml`.
Use the YAML dropdown to choose another profile explicitly. Selecting a profile
loads its default image, surface, size and fill; selecting a photo loads its
matching profile. An unmatched upload keeps the chosen profile. If a photo is
listed in multiple profiles, select a profile or surface to disambiguate it.

`ProfileCatalog` is the shared resolver for the UI selection API, model rendering,
trajectory preparation and Home. Preparation reads the complete selected YAML,
including transforms, image processing, speeds, pen clearance and timing. Each
job records the profile filename and SHA-256; changes to the photo, annotations,
profile or selection invalidate the previous execution review. New/renamed YAMLs
and edits are discovered without restarting the server; edits to the selected
profile reload its defaults and prepare a new preview when the UI is idle.

Successful preview and real preparations are cached with verified artifact
hashes. Real preparations may reuse a cached path when every starting joint is
within 0.005 rad of its saved value, matching the explicit saved-job check.
Returning to the same Home pose therefore tolerates small encoder differences.
Image, annotation, profile, model, planner and dependency changes still force
new preparation. Every cache load requires a fresh browser review token and the
normal live checks at Execute; the cache never starts motion.

Switching profiles never starts motion, warms the pen, changes the server's
execution mode, or restarts the driver. Real execution still requires a real-mode
server and an eligible measured profile. Driver-level settings such as stiffness
remain subject to the existing explicit restart workflow. A preview-only profile
cannot enable real execution through selection.

API: `GET /api/state` includes profile metadata; `GET /api/selection` resolves
`image`, `profile`, `surface`, or `fallback_profile`; `/api/model`, `/api/preview`
and `/api/home` accept the selected `profile` filename. Clients started before this
feature continue using the surface-based interface until their server is restarted.

## Profile flags and Execute availability

Open **Profile flags** beside the YAML selector to edit `calibrated` and
`fake_only`. These switches update only the two explicit top-level booleans in
that YAML, preserve its other contents, and invalidate previous trajectory
reviews. Edits require the current profile revision and are blocked while this
server is preparing or executing a job. They do not change qualification flags,
physical measurements, driver state, or server mode.

Execute is grey while a known prerequisite is missing; the reason appears next
to it. A preview server stays preview-only even with `calibrated: true` and
`fake_only: false`. Physical qualification and checked approach/return routes
remain required, and the executor still performs final hardware
checks when execution is requested.

## Connected browser

`ros2 launch pen_bringup connected_ui.launch.py` starts only the browser on port
8096 and attaches it to the existing real executor. It does not start a driver,
start another executor, warm the pen, or move the arm. The local shortcut is
`scripts/start-photo-real.sh`; `scripts/start-photo-preview.sh` remains an offline
alternative.

A connected server can display and prepare unfinished profiles without authorizing
their execution. Such profiles are compiled in preview mode and Execute displays
the missing physical requirements. Qualified profiles compile in real mode. The
backend rejects preview-prepared jobs at Execute even if flags are changed later;
a new real preparation is required.
