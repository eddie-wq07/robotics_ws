"""The surface maps: normals, tangents, stretch and size limits."""

import numpy as np
import pytest

from tactile.surface import load_surface

ROOT_SURFACES = ["plane", "can", "can_upright", "sphere"]


@pytest.mark.parametrize("name", ROOT_SURFACES)
def test_normals_and_frames_match_the_map(name):
    from tests.conftest import ROOT

    surface = load_surface(ROOT / f"tests/fixtures/surfaces/{name}.yaml")
    surface.fit(0.03)
    rng = np.random.default_rng(0)
    s, t = rng.uniform(-0.025, 0.025, 40), rng.uniform(-0.025, 0.025, 40)
    e = 1e-6
    along_s = (surface.point(s + e, t) - surface.point(s - e, t)) / (2 * e)
    along_t = (surface.point(s, t + e) - surface.point(s, t - e)) / (2 * e)
    normal = surface.normal(s, t)
    e_s, e_t, n = surface.frame(s, t)
    assert np.abs(np.sum(along_s * normal, 1)).max() < 1e-6
    assert np.abs(np.sum(along_t * normal, 1)).max() < 1e-6
    assert np.allclose(np.linalg.norm(normal, axis=1), 1)
    assert np.allclose(np.sum(np.cross(e_s, e_t) * n, 1), 1)  # Right-handed, normal outward.
    # Conformal: s and t stretch equally and stay perpendicular.
    assert np.allclose(np.linalg.norm(along_s, axis=1), np.linalg.norm(along_t, axis=1), atol=1e-6)
    assert np.allclose(np.linalg.norm(along_s, axis=1), surface.scale(s, t), atol=1e-6)


@pytest.mark.parametrize("name", ROOT_SURFACES)
def test_height_and_normal_inverts_the_map(name):
    from tests.conftest import ROOT

    surface = load_surface(ROOT / f"tests/fixtures/surfaces/{name}.yaml")
    surface.fit(0.03)
    s, t = np.array([0.0, 0.01, -0.02]), np.array([0.0, -0.015, 0.02])
    lift, normal = surface.height_and_normal(surface.point(s, t, 0.0016))
    assert np.allclose(lift, 0.0016)
    assert np.allclose(normal, surface.normal(s, t))


def test_markers_too_big_for_the_surface_are_refused():
    from tests.conftest import ROOT

    with pytest.raises(ValueError, match="ball"):
        load_surface(ROOT / "tests/fixtures/surfaces/sphere.yaml").fit(0.06)
    with pytest.raises(ValueError, match="can"):
        load_surface(ROOT / "tests/fixtures/surfaces/can.yaml").fit(0.04, (0.04, -0.02, 0.02))
    with pytest.raises(ValueError, match="end of the can"):
        load_surface(ROOT / "tests/fixtures/surfaces/can.yaml").fit(0.03, (0.02, -0.06, 0.06))


def test_globe_projection_keeps_distances_from_the_pole():
    from tests.conftest import ROOT

    globe = load_surface(ROOT / "tests/fixtures/surfaces/globe.yaml")
    notes = globe.fit(np.pi * globe.R / 2)  # A whole hemisphere.
    assert notes["picture_is_a_map"] and abs(notes["angle_deg"] - 90) < 1e-6
    pole = globe.point(0.0, 0.0)
    assert np.allclose(pole, globe.centre + [0, 0, globe.R])
    # Every point a map distance r from the centre lies r along the ball from the pole.
    rng = np.random.default_rng(1)
    rho, beta = rng.uniform(0, np.pi * globe.R / 2, 30), rng.uniform(0, 2 * np.pi, 30)
    points = globe.point(rho * np.cos(beta), rho * np.sin(beta))
    up = (points - globe.centre) @ [0, 0, 1] / globe.R
    assert np.allclose(np.arccos(np.clip(up, -1, 1)) * globe.R, rho)
    # The map's edge is the equator.
    edge = globe.point(np.pi * globe.R / 2, 0.0)
    assert abs(edge[2] - globe.centre[2]) < 1e-9
    with pytest.raises(ValueError, match="hemisphere"):
        globe.fit(np.pi * globe.R * 0.6)
