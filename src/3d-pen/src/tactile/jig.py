"""Nominal jig meshes in robot-base coordinates; not printable CAD.

The top of the mounting flange comes from configs/cell.yaml. The stand/base
stack downward, and the hemisphere/riser come from configs/jig.yaml.
"""

import numpy as np
import trimesh
import yaml


COLOURS = {"stand": (128, 136, 146), "base": (222, 196, 140), "pin": (70, 74, 82),
           "screw": (60, 60, 66), "flange": (120, 168, 210), "riser": (205, 185, 150),
           "surface": (240, 236, 226)}


def _part(mesh, kind):
    mesh.metadata["component"] = kind
    mesh.visual.face_colors = [*COLOURS[kind], 255]
    return mesh


def _cylinder(radius, length, centre, axis):
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=32)
    mesh.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], axis))
    mesh.apply_translation(centre)
    return mesh


def _box(size, centre):
    mesh = trimesh.creation.box(size)
    mesh.apply_translation(centre)
    return mesh


def _half_cylinder(radius, length, centre):
    """A half-cylinder lying along x, flat side down, its axis at ``centre``."""
    arc = np.linspace(0, np.pi, 49)
    ring = np.column_stack([np.zeros_like(arc), radius * np.cos(arc), radius * np.sin(arc)])
    near, far = ring + [-length / 2, 0, 0], ring + [length / 2, 0, 0]
    count = len(arc)
    vertices = np.vstack([near, far, [[-length / 2, 0, 0], [length / 2, 0, 0]]])
    faces = []
    for i in range(count - 1):  # the curved side
        faces += [[i, i + 1, count + i + 1], [i, count + i + 1, count + i]]
    faces += [[0, count, 2 * count - 1], [0, 2 * count - 1, count - 1]]  # the flat bottom
    for i in range(count - 1):  # the two ends, fanned from the middle of the flat side
        faces += [[2 * count, i + 1, i], [2 * count + 1, count + i, count + i + 1]]
    mesh = trimesh.Trimesh(vertices, faces)
    mesh.fix_normals()
    mesh.apply_translation(centre)
    return mesh


def heights(jig, top):
    """The stack's heights (metres above the table), from the jig's top face ``top`` down."""
    base_top = top - jig["flange"]["size_m"][2]
    return {"flange_top": top, "base_top": base_top,
            "stand_top": base_top - jig["base"]["size_m"][2]}


def stand_parts(jig, centre, h):
    """The base with its pins and screw holes, and the stand under it, in the world frame."""
    base, stand = jig["base"], jig["stand"]
    x, y = centre
    parts = [_part(_box(base["size_m"], [x, y, h["base_top"] - base["size_m"][2] / 2]), "base")]
    for px, py in base["pins_m"]:
        length = base["pin_proud_m"] + base["size_m"][2]
        parts.append(_part(_cylinder(base["pin_diameter_m"] / 2, length,
                                     [x + px, y + py, h["base_top"] + base["pin_proud_m"] - length / 2],
                                     [0, 0, 1]), "pin"))
    for sx, sy in base["screws_m"]:  # the inserts' openings
        parts.append(_part(_cylinder(0.0028, 0.0004, [x + sx, y + sy, h["base_top"] + 0.0002],
                                     [0, 0, 1]), "screw"))
    tall = h["stand_top"]
    if tall > 0.001:
        side, (wide, deep) = stand["profile_m"], stand["footprint_m"]
        if str(stand["kind"]) == "8020" and tall > 2 * side:
            # Four legs and a ring of rails under the base.
            for sx in (-1, 1):
                for sy in (-1, 1):
                    parts.append(_part(_box([side, side, tall - side],
                                            [x + sx * (wide - side) / 2, y + sy * (deep - side) / 2,
                                             (tall - side) / 2]), "stand"))
            for sy in (-1, 1):
                parts.append(_part(_box([wide, side, side], [x, y + sy * (deep - side) / 2,
                                                             tall - side / 2]), "stand"))
            for sx in (-1, 1):
                parts.append(_part(_box([side, deep - 2 * side, side],
                                        [x + sx * (wide - side) / 2, y, tall - side / 2]), "stand"))
        else:
            parts.append(_part(_box([wide, deep, tall], [x, y, tall / 2]), "stand"))
    return parts


def surface_parts(jig, name, centre, h):
    """One flange with its screws, riser and printed surface, in the world frame."""
    base, flange, item = jig["base"], jig["flange"], jig["surfaces"][name]
    x, y = centre
    parts = [_part(_box(flange["size_m"], [x, y, h["flange_top"] - flange["size_m"][2] / 2]), "flange")]
    for sx, sy in base["screws_m"]:  # M4 cap screws through the corners
        parts.append(_part(_cylinder(0.0035, 0.004, [x + sx, y + sy, h["flange_top"] + 0.002],
                                     [0, 0, 1]), "screw"))
    spec = yaml.safe_load(open(item["file"]))
    top, riser = h["flange_top"], item["riser_m"]
    if spec["type"] == "plane":
        size = spec["size_m"]
        parts.append(_part(_box(size, [x, y, top + size[2] / 2]), "surface"))
    elif spec["type"] == "cylinder":
        radius, length = spec["radius_m"], spec["height_m"]
        parts.append(_part(_box([length, item["riser_width_m"], riser],
                                [x, y, top + riser / 2]), "riser"))
        parts.append(_part(_half_cylinder(radius, length, [x, y, top + riser]), "surface"))
    else:
        radius = spec["radius_m"]
        parts.append(_part(_cylinder(item["riser_diameter_m"] / 2, riser, [x, y, top + riser / 2],
                                     [0, 0, 1]), "riser"))
        arc = np.linspace(0, np.pi / 2, 32)
        profile = np.vstack([[0, 0], np.column_stack([radius * np.cos(arc), radius * np.sin(arc)])])
        dome = trimesh.creation.revolve(profile, sections=64)
        dome.apply_translation([x, y, top + riser])
        parts.append(_part(dome, "surface"))
    return parts
