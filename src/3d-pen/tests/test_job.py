"""Verify the geometry contract passed to MoveIt, without requiring ROS."""
from copy import deepcopy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
import trimesh
import yaml

from tactile.job import make_job
from tactile.scene import asset_mesh
from tactile.relief import make_relief, relief_to_json
from tactile.slicer import slice_relief
from tactile.surface import load_surface
from tests.conftest import ROOT


@pytest.fixture(scope="module")
def inputs():
    process = yaml.safe_load((ROOT / "configs/process.yaml").read_text())
    cell = yaml.safe_load((ROOT / "configs/cell.yaml").read_text())
    surface = load_surface(ROOT / "configs/surface.yaml")
    relief = relief_to_json(make_relief(ROOT / "examples/apple.png", 45, process, fill=False))
    return slice_relief(relief, surface, process), cell


def test_tool_offset_reconstructs_every_tip_and_normal(inputs, monkeypatch):
    monkeypatch.chdir(ROOT)
    path, cell = inputs
    job = make_job(path, cell)
    draw = [s for s in job["segments"] if s["kind"] == "DRAW"]
    assert len(draw) == len(path["strokes"])
    offset = np.array([*cell["plate"]["center_m"], cell["plate"]["top_z_m"]])
    previous_q = None
    for segment in job["segments"]:
        poses = np.array(segment["flange_poses"])
        axes = Rotation.from_quat(poses[:, 3:]).as_matrix()[:, :, 2]
        reconstructed = poses[:, :3] + axes * cell["pen"]["length_m"]
        np.testing.assert_allclose(reconstructed, segment["tcp_points_m"], atol=1e-12)
        for q in poses[:, 3:]:
            if previous_q is not None:
                assert np.dot(q, previous_q) > 0.99  # No sudden orientation flips.
            previous_q = q
    for segment, stroke in zip(draw, path["strokes"], strict=True):
        np.testing.assert_allclose(segment["tcp_points_m"], np.array(stroke["points_m"]) + offset)
        axes = Rotation.from_quat(np.array(segment["flange_poses"])[:, 3:]).as_matrix()[:, :, 2]
        np.testing.assert_allclose(axes, -np.array(stroke["normals"]), atol=1e-6)
    assert job["planning_status"] == "UNPLANNED" and job["preview_only"]
    assert job["calibrated"] is False
    json.dumps(job, allow_nan=False)


def test_scene_keeps_fixture_fixed_and_tool_attached(inputs, monkeypatch):
    monkeypatch.chdir(ROOT)
    job = make_job(*inputs)
    parts = {part["id"]: part for part in job["scene"]}
    assert parts["d405_camera"]["frame"] == parts["extrusion_servo"]["frame"] == "flange"
    assert parts["d405_camera"]["status"] == "placeholder"
    assert parts["surface_1"]["frame"] == parts["stand_1"]["frame"] == "base"
    cap = np.array(parts["surface_1"]["vertices"])
    np.testing.assert_allclose([cap.min(0), cap.max(0)], [[0.375, -0.075, 0.22], [0.525, 0.075, 0.295]], atol=1e-8)
    assert len(parts) == len(job["scene"])


@pytest.mark.parametrize("normal", [[0, 0, 0], [float("nan"), 0, 1]])
def test_invalid_normals_are_refused(inputs, normal):
    path, cell = deepcopy(inputs)
    path["strokes"][0]["normals"][0] = normal
    with pytest.raises(ValueError, match="normal|finite"):
        make_job(path, cell)


def test_import_scale_and_glb_node_transform(tmp_path):
    mesh = trimesh.creation.box([20, 30, 40])
    source = tmp_path / "part.stl"
    mesh.export(source)
    with pytest.raises(ValueError, match="units"):
        asset_mesh({"file": str(source)})
    scaled = asset_mesh({"file": str(source), "units": "mm", "position_m": [0.1, 0, 0]})
    np.testing.assert_allclose(scaled.extents, [0.02, 0.03, 0.04])
    np.testing.assert_allclose(scaled.centroid, [0.1, 0, 0], atol=1e-10)
    scene = trimesh.Scene()
    transform = np.eye(4); transform[0, 3] = 0.25
    scene.add_geometry(trimesh.creation.box([0.02]*3), transform=transform)
    source = tmp_path / "part.glb"; scene.export(source)
    loaded = asset_mesh({"file": str(source), "units": "m"})
    np.testing.assert_allclose(loaded.centroid, [0.25, 0, 0], atol=1e-8)


def test_prepare_clears_stale_moveit_report(tmp_path, monkeypatch):
    from tactile.cli import main
    monkeypatch.chdir(ROOT)
    (tmp_path / "moveit_report.json").write_text('{"status":"PLANNED"}')
    main(["prepare", "examples/apple.png", "--out", str(tmp_path)])
    assert not (tmp_path / "moveit_report.json").exists()
    job = json.loads((tmp_path / "job.json").read_text())
    assert job["planning_status"] == "UNPLANNED"
    assert (tmp_path / "relief.png").is_file()
