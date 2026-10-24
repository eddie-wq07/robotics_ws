"""Fixed fixture and attached tool meshes; ROS/MoveIt owns the robot model."""
from pathlib import Path
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import yaml

from .jig import heights, stand_parts, surface_parts
from .surface import load_surface


def asset_mesh(asset):
    path = Path(asset["file"])
    if path.suffix.lower() not in {".stl", ".glb"}:
        raise ValueError("CAD must be STL or GLB")
    scale = {"m": 1, "cm": 0.01, "mm": 0.001}.get(asset.get("units"))
    if scale is None:
        raise ValueError("CAD requires explicit units: m, cm, or mm")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if asset.get("sha256") and asset["sha256"] != digest:
        raise ValueError(f"CAD changed since import: {path}")
    mesh = trimesh.load(path, force="scene", process=False).to_geometry()
    if not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
        raise ValueError("CAD must contain finite triangle geometry")
    mesh.apply_scale(scale)
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", asset.get("rotation_deg", [0, 0, 0]), degrees=True).as_matrix()
    transform[:3, 3] = asset.get("position_m", [0, 0, 0])
    if not np.isfinite(transform).all():
        raise ValueError("CAD placement must be finite")
    mesh.apply_transform(transform)
    return mesh


def scene_meshes(cell, spec):
    """Meshes use either robot-base coordinates or the moving flange coordinates."""
    origin = np.array([*cell["plate"]["center_m"], cell["plate"]["top_z_m"]])
    jig = yaml.safe_load(Path(cell["jig_config"]).read_text())
    h = heights(jig, origin[2])
    parts = stand_parts(jig, origin[:2], h) + surface_parts(jig, "hemisphere", origin[:2], h)
    rows, counts = [], {}

    def add(name, category, mesh, frame="base", status="nominal"):
        rows.append({"id": name, "category": category, "mesh": mesh, "frame": frame, "status": status})

    for mesh in parts:
        category = mesh.metadata["component"]
        if category == "surface":
            # Use the active surface's radius and position, including future calibration.
            surface = load_surface(spec)
            arc = np.linspace(0, np.pi / 2, 32)
            profile = np.vstack([[0, 0], np.column_stack([surface.R*np.cos(arc), surface.R*np.sin(arc)])])
            mesh = trimesh.creation.revolve(profile, sections=64)
            mesh.apply_translation(surface.centre + origin)
        counts[category] = counts.get(category, 0) + 1
        add(f"{category}_{counts[category]}", category, mesh)
    table = trimesh.creation.box([2, 2, 0.05])
    table.apply_translation([0, 0, -0.025])
    add("table", "table", table)

    pen = cell["pen"]
    length, nose, radius = pen["length_m"], pen["nose_length_m"], pen["body_radius_m"]
    adapter, tip_gap = 0.015, 0.003
    body = length - adapter - nose - tip_gap
    if body <= 0 or min(radius, nose, pen["tip_radius_m"]) <= 0:
        raise ValueError("Pen dimensions must be positive and leave room for the body")
    for name, r, size, z in [("adapter", 0.025, adapter, adapter/2),
                             ("pen_body", radius, body, adapter + body/2),
                             ("nozzle", pen["tip_radius_m"], tip_gap, length-tip_gap/2)]:
        mesh = trimesh.creation.cylinder(radius=r, height=size, sections=24)
        mesh.apply_translation([0, 0, z])
        add(name, name, mesh, "flange")
    mesh = trimesh.creation.revolve([[0, 0], [radius, 0], [pen["tip_radius_m"], nose], [0, nose]], sections=32)
    mesh.apply_translation([0, 0, adapter + body])
    add("tapered_nose", "tapered_nose", mesh, "flange")
    for accessory in pen.get("accessories", []):
        mesh = trimesh.creation.box(accessory["size_m"])
        mesh.apply_translation(accessory["position_m"])
        add(accessory["name"], accessory["name"], mesh, "flange", "placeholder")
    for asset in yaml.safe_load(Path(cell["assets_config"]).read_text())["assets"]:
        mesh = asset_mesh(asset)
        frame = asset["frame"]
        if frame == "build":
            mesh.apply_translation(origin)
        if frame not in {"build", "world", "flange"}:
            raise ValueError("CAD frame must be build, world, or flange")
        frame = "flange" if frame == "flange" else "base"
        if asset.get("replaces"):
            rows = [r for r in rows if not (r["category"] == asset["replaces"] and r["frame"] == frame)]
        add(asset["name"], asset.get("replaces") or "imported", mesh.convex_hull, frame,
            "imported CAD — conservative convex hull; placement uncalibrated")
    names = [r["id"] for r in rows]
    if len(names) != len(set(names)):
        raise ValueError("Scene component names must be unique")
    return [{**{k: v for k, v in row.items() if k != "mesh"},
             "vertices": row["mesh"].vertices.tolist(), "triangles": row["mesh"].faces.tolist()} for row in rows]
