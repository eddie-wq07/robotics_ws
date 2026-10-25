"""Strokes lie on their layers; travels stay off the surface."""

import numpy as np
import pytest

from tactile.slicer import slice_relief
from tactile.surface import load_surface
from pen_toolpath.clearance import MIN_PEN_CLEARANCE_M
from tests.conftest import ROOT


@pytest.mark.parametrize("name", ["plane", "can", "can_upright", "sphere"])
def test_strokes_sit_on_their_layer_and_travels_lift_clear(house, process, name):
    surface = load_surface(ROOT / f"tests/fixtures/surfaces/{name}.yaml")
    toolpath = slice_relief(house, surface, process)
    height = process["pen"]["layer_height_mm"] / 1000
    retract = process["slice"]["retract_mm"] / 1000
    assert toolpath["layers"] == len(process["levels"]["outline"])
    assert not toolpath["legibility"]["warnings"]
    for stroke in toolpath["strokes"]:
        lift, normal = surface.height_and_normal(np.asarray(stroke["points_m"]))
        assert np.allclose(lift, MIN_PEN_CLEARANCE_M + (stroke["layer"] - 1) * height, atol=1e-6)
        assert np.allclose(normal, stroke["normals"], atol=1e-5)
        steps = np.linalg.norm(np.diff(stroke["points_m"], axis=0), axis=1)
        # Spacing is sampled in surface coordinates; curved offset layers expand it.
        bound = (process["slice"]["sample_mm"] / 1000
                 * float(surface.scale(0.0, 0.0))
                 * surface.offset_scale(stroke["lift_mm"] / 1000))
        assert steps.max() <= bound + 2e-7  # Exported coordinates round to 0.1 micrometre.
    for travel in toolpath["travels"]:
        lift, _ = surface.height_and_normal(np.asarray(travel["points_m"]))
        across = np.asarray(travel["lifts_mm"]) / 1000
        assert np.allclose(lift, across, atol=1e-6)
        assert lift.min() >= 0.0
        # Between lifting off and lowering, the pen stays at the retract height.
        assert lift[5:-5].min() >= retract if len(lift) > 10 else True


def test_every_layer_is_covered(house, process):
    toolpath = slice_relief(house, load_surface(ROOT / "tests/fixtures/surfaces/plane.yaml"), process)
    by_layer = {}
    for stroke in toolpath["strokes"]:
        by_layer[stroke["layer"]] = by_layer.get(stroke["layer"], 0) + stroke["length_mm"]
    # The fill layer is by far the longest; ridges narrow as they rise.
    heights = [by_layer[k] for k in sorted(by_layer)]
    assert heights[0] > 2 * heights[1] and all(a >= b for a, b in zip(heights[1:], heights[2:]))


def test_ridges_are_several_strands_wide(house, process):
    toolpath = slice_relief(house, load_surface(ROOT / "tests/fixtures/surfaces/plane.yaml"), process)
    outline_passes = [s for s in toolpath["strokes"] if s["level"] == "outline" and s["layer"] == 2]
    assert len(outline_passes) == process["levels"]["outline"][1]
