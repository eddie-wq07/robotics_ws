"""Export one self-contained, unplanned job for the thin MoveIt/RViz adapter."""
import numpy as np
from scipy.spatial.transform import Rotation

from .scene import scene_meshes


def make_job(toolpath, cell):
    if cell["robot"]["model"] != "Rizon4s":
        raise ValueError("The ROS preview is configured for Rizon4s only")
    if toolpath["surface"].get("fixture") != "hemisphere" or toolpath["surface"]["type"] != "sphere":
        raise ValueError("The initial workflow supports the jig-mounted hemisphere only")
    strokes = toolpath["strokes"]
    if not strokes:
        raise ValueError("The picture produced no printable strokes")
    offset = np.array([*cell["plate"]["center_m"], cell["plate"]["top_z_m"]], float)
    if not np.isfinite(offset).all():
        raise ValueError("Fixture placement must be finite")
    length = float(cell["pen"]["length_m"])
    if not np.isfinite(length) or length <= 0:
        raise ValueError("Flange-to-tip distance must be finite and positive")
    segments = []
    previous_x, previous_q = np.array([1., 0, 0]), None

    def segment(kind, points, normals, stroke_id=None):
        nonlocal previous_x, previous_q
        tips = np.asarray(points, float) + offset
        normals = np.asarray(normals, float)
        if tips.ndim != 2 or tips.shape[1] != 3 or normals.shape != tips.shape or not np.isfinite([tips, normals]).all():
            raise ValueError("Stroke points and normals must be matching finite XYZ arrays")
        poses = []
        for tip, normal in zip(tips, normals, strict=True):
            norm = np.linalg.norm(normal)
            if norm < 1e-8:
                raise ValueError("Surface normals must be nonzero")
            z = -normal / norm
            x = previous_x - z * np.dot(previous_x, z)
            if np.linalg.norm(x) < 1e-6:
                seed = np.eye(3)[np.argmin(np.abs(z))]
                x = seed - z * np.dot(seed, z)
            x /= np.linalg.norm(x)
            q = Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z])).as_quat()
            if previous_q is not None and np.dot(q, previous_q) < 0:
                q = -q
            poses.append([*(tip - length*z), *q])
            previous_x, previous_q = x, q
        segments.append({"kind": kind, "stroke": stroke_id,
                         "tcp_points_m": tips.tolist(), "flange_poses": poses})

    first, last = strokes[0], strokes[-1]
    # Approach and retract are path geometry, not proof of reachable robot motion.
    p, n = np.array(first["points_m"][0]), np.array(first["normals"][0])
    segment("TRAVEL", [p + n*0.01, p], [n, n])
    for i, stroke in enumerate(strokes):
        if i:
            travel = toolpath["travels"][i-1]
            segment("TRAVEL", travel["points_m"], travel["normals"])
        segment("DRAW", stroke["points_m"], stroke["normals"], stroke["id"])
    p, n = np.array(last["points_m"][-1]), np.array(last["normals"][-1])
    segment("TRAVEL", [p, p + n*0.01], [n, n])
    return {"schema_version": 1, "kind": "tactile_ros_job", "planning_status": "UNPLANNED",
            "preview_only": True, "calibrated": False, "robot": "Rizon4s",
            "units": "m", "pose_format": "x,y,z,qx,qy,qz,qw",
            "frame": "robot_base", "flange_to_tip_m": length,
            "surface": toolpath["surface"], "segments": segments,
            "scene": scene_meshes(cell, toolpath["surface"]),
            "notes": ["Nominal geometry and mounting poses", "No extrusion or hardware execution",
                      "MoveIt must check the entire path; partial paths are rejected"]}
