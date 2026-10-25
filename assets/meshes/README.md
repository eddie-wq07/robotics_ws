# Onshape assets

Source: [team's hackathon document](https://cad.onshape.com/documents/618b5d2bd89bee4efe7c7a72/w/61ea072912c4caea42eb5a8b/e/67d428c6e2b198ed7d60f6ff). Exported 2026-10-23; binary STL, **meters**, fine resolution, Z-up.

| File | CAD bounding size, mm | Role |
| --- | --- | --- |
| `sphere.stl` | 80 × 100 × 80 | 80 mm diameter sphere, including mounting tabs |
| `jig.stl` | 210 × 120 × 75 | Object support and frame attachment |
| `pen_holder.stl` | 188.8 × 67.5 × 102.5 | Pen mounting bracket |
| `pen_holder_combined.stl` | 224.709 × 67.5 × 102.5 | Active holder and seated pen assembly; imported from Downloads and converted from mm to m |
| `pen.stl` | 193.4 × 20.0 × 24.0 | Imported `3D_Pen_decimated` model |
| `tab.stl` | 36 × 22 × 4 | Separate tab |
| `flat.stl` | 80 × 100 × 2.5 | Alternative flat test piece |
| `cylinder.stl` | 80 × 80 × 92 | Alternative cylindrical test piece |

All eight meshes are watertight. `manifest.json` records SHA-256 hashes, exact bounds, and original export names. The combined holder entry also records the original download hash and unit conversion. The assembly's four STL members duplicate the flat, sphere, cylinder, and jig geometries; their mapping is recorded rather than storing duplicate files.

**Placement pending:** these STL exports retain part-local coordinates, not assembly occurrence transforms. The pen's long axis is CAD X. Do not assume these origins equal the robot base or flange. The current nominal 75 mm-radius hemisphere is a different object; update curvature and placement together before planning against this 40 mm-radius sphere. The holder's measured flange-to-tip transform is still needed.
