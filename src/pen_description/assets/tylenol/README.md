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
These dimensions have not been checked against the real object. Preserve the
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
