"""A picture into a relief: which kind of picture, which levels."""

from tactile.relief import make_relief, relief_to_json
from tests.conftest import ROOT


def levels(data):
    return {level["name"]: level for level in data["levels"]}


def test_flat_colour_clipart_is_a_silhouette_with_three_levels(house):
    assert house["mode"] == "silhouette"
    found = levels(house)
    assert len(found["fill"]["polygons"]) == 1
    assert len(found["outline"]["centerlines"]) == 1
    # The roof line, two windows and the door.
    assert len(found["detail"]["centerlines"]) >= 4
    assert found["outline"]["layers"] > found["detail"]["layers"] > found["fill"]["layers"]
    # Ridges are more than one strand wide at their base.
    assert found["outline"]["profile"][0] >= 2 and found["detail"]["profile"][0] >= 2
    # The picture is scaled to 64 mm; sharp tips thinner than a fingertip can
    # feel (the eaves) are then trimmed, so the relief comes out a little smaller.
    left, bottom, right, top = house["bounds_mm"]
    assert 58 < max(right - left, top - bottom) <= 64.5


def test_line_art_is_detected_and_filled(process):
    data = relief_to_json(make_relief(ROOT / "examples/house_lines.png", 64, process))
    assert data["mode"] == "lineart"
    found = levels(data)
    assert len(found["fill"]["polygons"]) == 1
    assert len(found["detail"]["centerlines"]) >= 4


def test_no_fill_leaves_only_ridges(process):
    data = relief_to_json(make_relief(ROOT / "examples/house.png", 64, process, fill=False))
    assert "fill" not in levels(data)


def test_apple_and_sun(process):
    apple = levels(relief_to_json(make_relief(ROOT / "examples/apple.png", 60, process)))
    assert len(apple["fill"]["polygons"]) == 2  # The apple with its stem, and the leaf.
    sun = levels(relief_to_json(make_relief(ROOT / "examples/sun.png", 70, process)))
    assert len(sun["fill"]["polygons"]) == 9  # The disc and eight rays.
    assert len(sun["detail"]["centerlines"]) == 1  # The ring around the core.


def test_dome_style_is_highest_in_the_middle(process):
    from tactile.relief import heightmap, relief_from_json

    data = relief_to_json(make_relief(ROOT / "examples/apple.png", 60, process, style="dome"))
    assert data["style"] == "dome"
    assert all(level["kind"] == "area" for level in data["levels"])
    layers, _, (left, bottom, right, top) = heightmap(relief_from_json(data), 0.5)
    rows, cols = layers.shape
    middle = layers[rows // 2 - 4: rows // 2 + 4, cols // 2 - 4: cols // 2 + 4].max()
    edge = layers[layers > 0].min()
    assert middle >= 6 and edge == process["dome"]["base_layers"]
    # Each layer sits inside the one below it.
    from shapely import unary_union

    regions = [unary_union(level["polygons"]) for level in relief_from_json(data)["levels"]]
    assert all(lower.buffer(0.05).contains(upper) for lower, upper in zip(regions, regions[1:]))


def test_whole_picture_keeps_the_map_centred(process):
    import numpy as np

    size = np.pi * 75
    data = relief_to_json(make_relief(ROOT / "examples/globe.png", size, process, crop=False))
    left, bottom, right, top = data["bounds_mm"]
    # The land stays inside the picture's frame (the equator) and is not
    # re-centred: the pole, not the land's middle, is the marker centre.
    assert max(abs(left), abs(right), abs(bottom), abs(top)) <= size / 2 + 1
    assert right - left > 0.8 * size
    assert abs((top + bottom) / 2) > 5
