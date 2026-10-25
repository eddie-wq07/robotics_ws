# Eddie and mom photo board

Photo/cardboard workpiece for `assets/inputs/eddiemom.png`. The original image is
preserved here as `eddiemom.png` and embedded in `photo_board.glb`.

- Photo/cardboard size: 120 x 120 mm.
- Combined modeled thickness: 5.35 mm. The photo drawing face is 5.35 mm above
  the calibrated jig top at jig Z=75 mm.
- Surface origin: the top of the photo. The visual board extends downward.
- Six simplified strokes outline the two foreground people and large face,
  hand and hat shapes. The water bottle, background people, beach texture, jacket interior
  details, clothing creases and small facial details are omitted.
- `eddiemom.png.strokes.json` replaces the automatic edges with these outlines.
  It is tied to the source image by SHA-256 and retains the full image frame,
  without recentering or rescaling the people. Strokes stop 1 mm from the paper edge.
- Preview motion: 7 mm/s trace, 15 mm/s travel, 2 mm pen clearance, 250 Hz samples.

Run `scripts/start-photo-real.sh` to attach the UI to the existing real executor
at http://127.0.0.1:8096/. `scripts/start-photo-preview.sh` provides offline mode.
The profile is `src/pen_description/config/photo_board.yaml`. Calibration and
clearance were confirmed by the operator; the provenance is saved in
`registration.operator_clearance_evidence`. The agent did not observe the
operator's physical clearance check. Hardware qualification is carried from
the unchanged sphere setup, with its previously qualified approach rechecked
against this plane and reversed for return. Normal executor checks remain on.

The GLB is authored in the ROS surface frame (Z up); no glTF axis conversion
should be applied by the viewer. Its photo UVs use the same right/up mapping
as the planar motion model.

The photo-board camera and pen-stroke panels are viewed rotated 180 degrees;
this display change does not rotate the underlying photo or motion coordinates.
