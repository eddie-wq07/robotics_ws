"""Compatibility adapter for legacy tactile preview export.

New execution uses compiler.py; this adapter reuses shared planar stroke and
surface algorithms so legacy preview callers do not own a second implementation.
"""
import numpy as np
from shapely import LineString, unary_union
from pen_image_processing.relief import relief_from_json
from pen_image_processing.strokes import _rings, shells, hatch, ridge_passes, order, densify
from .clearance import MIN_PEN_CLEARANCE_M

def _alongside(first, second, gap, step=0.5):
    """Longest stretch (mm) of ``first`` that runs within ``gap`` of ``second``."""
    count = max(2, int(np.ceil(first.length / step)) + 1)
    near = np.array([first.interpolate(d).distance(second) < gap
                     for d in np.linspace(0, first.length, count)])
    longest = run = 0
    for flag in near:
        run = run + 1 if flag else 0
        longest = max(longest, run)
    return longest * first.length / (count - 1), float(first.distance(second))


def legibility(relief, rules, fit_notes):
    """Measured against the tactile rules: ridges running too close, and stretch.

    Two ridges whose edges run side by side closer than ``min_gap_mm`` for
    more than that distance read as one. Ridges that only meet at a corner or a
    T do not count.
    """
    gap, strand = rules["min_gap_mm"], relief["strand_width_mm"]
    lines = []
    for level in relief["levels"]:
        if level.get("kind") != "ridge":
            continue
        half = level["profile"][0] * strand / 2
        for index, line in enumerate(level["centerlines"]):
            lines.append((f"{level['name']}-{index}", LineString(line.coords), half))
    worst = None
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            (name_a, a, half_a), (name_b, b, half_b) = lines[i], lines[j]
            reach = gap + half_a + half_b  # Centre-line distance at the gap.
            # Side by side means each runs along the other; a T or a corner,
            # where one line only ends near the other, does not count.
            run, distance = _alongside(a, b, reach)
            back, _ = _alongside(b, a, reach)
            run = min(run, back)
            edge_gap = distance - half_a - half_b
            if run > gap and (worst is None or edge_gap < worst[0]):
                worst = (edge_gap, name_a, name_b, run)
    warnings = []
    if worst:
        scale = (gap + 2 * strand) / max(worst[0] + 2 * strand, 0.5)
        advice = (f"try --size-mm {np.ceil(relief['size_mm'] * scale):.0f}"
                  if not fit_notes.get("picture_is_a_map") and scale < 4
                  else "they will feel joined there")
        warnings.append(
            f"{worst[1]} runs {max(worst[0], 0):.1f} mm from {worst[2]} for {worst[3]:.0f} mm "
            f"(ridges closer than {gap:g} mm read as one); {advice}")
    stretch = fit_notes.get("max_length_error", 0.0)
    # A map picture is meant to be unbent this way, so its stretch is no flaw.
    if stretch > 0.05 and not fit_notes.get("picture_is_a_map"):
        warnings.append(f"the surface stretches the marker by up to {stretch * 100:.0f}%")
    return {
        "ridges_too_close": None if worst is None else {
            "lines": list(worst[1:3]), "gap_mm": round(worst[0], 2), "run_mm": round(worst[3], 1)},
        "detail_lines_shortened": relief.get("notes", {}).get("detail_lines_shortened", 0),
        "max_length_error": round(stretch, 4),
        "warnings": warnings,
    }


def slice_relief(relief_data, surface, process):
    """The strokes and travels that draw ``relief_data`` on ``surface``."""
    relief = relief_from_json(relief_data)
    pen, rules, settings = process["pen"], process["tactile"], process["slice"]
    width, height = pen["strand_width_mm"], pen["layer_height_mm"]
    levels = [level for level in relief["levels"] if level["polygons"]]
    if not levels:
        raise ValueError("the relief has nothing to draw")
    everything = unary_union([p for level in levels for p in level["polygons"]])
    coords = np.vstack([np.asarray(ring.coords) for ring in _rings(everything)])
    reach = float(np.linalg.norm(coords, axis=1).max())
    extent = (float(np.abs(coords[:, 0]).max()) / 1000, float(coords[:, 1].min()) / 1000,
              float(coords[:, 1].max()) / 1000)
    fit_notes = surface.fit(reach / 1000, extent)
    layer_count = max(level["layers"] for level in levels)

    planned, position = [], np.zeros(2)
    for layer in range(1, layer_count + 1):
        lift = MIN_PEN_CLEARANCE_M * 1000 + (layer - 1) * height
        stretch = float(surface.offset_scale(lift / 1000))
        strand = width / stretch  # One strand's width in (s, t) on this layer.
        region = unary_union([p for level in levels if level["layers"] >= layer
                              and level.get("kind") != "ridge" for p in level["polygons"]])
        thick = region.buffer(-0.6 * width).buffer(0.6 * width).intersection(region)
        groups = [
            shells(thick, strand, settings["shells"], 2 * width),
            hatch(thick.buffer(-(settings["shells"] + 0.4) * strand), settings["hatch_overlap"]
                  * strand, 45.0 if layer % 2 else 135.0, strand) if not thick.is_empty else [],
            ridge_passes(levels, layer, thick, strand, 2 * width),
        ]
        for group in groups:
            ordered, position = order(group, position)
            for stroke in ordered:
                planned.append({**stroke, "layer": layer, "lift_mm": lift})

    # On a curve, split strokes where the pen would turn more than max_turn_deg
    # from the stroke's start, so no single stroke sweeps the wrist too far.
    pieces = []
    limit = np.cos(np.radians(settings["max_turn_deg"]))
    for stroke in planned:
        st = densify(stroke["points"], settings["sample_mm"])
        if surface.curvature_radius():
            normals = surface.normal(st[:, 0] / 1000, st[:, 1] / 1000)
            start = 0
            for index in range(1, len(st)):
                if normals[index] @ normals[start] < limit:
                    pieces.append(({**stroke, "closed": False}, st[start:index + 1]))
                    start = index
            if start:
                if len(st) - start >= 2:  # A one-point remainder draws nothing.
                    pieces.append(({**stroke, "closed": False}, st[start:]))
                continue
        pieces.append((stroke, st))
    strokes = []
    for index, (stroke, st) in enumerate(pieces):
        s, t = st[:, 0] / 1000, st[:, 1] / 1000
        strokes.append({
            "id": f"L{stroke['layer']}-{index:03d}",
            "layer": stroke["layer"],
            "level": stroke.get("level", "fill"),
            "kind": stroke["kind"],
            "closed": stroke["closed"],
            "lift_mm": stroke["lift_mm"],
            "length_mm": float(np.sum(np.linalg.norm(np.diff(st, axis=0), axis=1))),
            "st_mm": np.round(st, 4).tolist(),
            "points_m": np.round(surface.point(s, t, stroke["lift_mm"] / 1000), 7).tolist(),
            "normals": np.round(surface.normal(s, t), 6).tolist(),
        })
    travels = [
        travel(surface, a, b, settings) for a, b in zip(strokes[:-1], strokes[1:])
    ]
    length = sum(stroke["length_mm"] for stroke in strokes) / 1000
    e_s, e_t, normal = (v[0] for v in surface.frame(np.zeros(1), np.zeros(1)))
    return {
        "kind": "tactile_toolpath",
        "schema_version": 1,
        "surface": surface.describe(),
        "fit": fit_notes,
        # Where the marker is and which way is "up" on it, for the viewer's cameras.
        "marker_frame": {"point": np.round(surface.point(0.0, 0.0), 6).tolist(),
                         "normal": np.round(normal, 6).tolist(),
                         "right": np.round(e_s, 6).tolist(), "up": np.round(e_t, 6).tolist()},
        "layers": layer_count,
        "strand_width_mm": width,
        "layer_height_mm": height,
        "strokes": strokes,
        "travels": travels,
        "draw_length_m": round(length, 4),
        "lift_count": len(travels),
        "legibility": legibility(relief, rules, fit_notes),
    }


def travel(surface, before, after, settings):
    """Lift off ``before``'s end, cross in (s, t) above the surface, lower onto ``after``."""
    a, b = np.asarray(before["st_mm"][-1]), np.asarray(after["st_mm"][0])
    up = max(before["lift_mm"], after["lift_mm"]) + settings["retract_mm"]
    lift = np.linspace(before["lift_mm"], up, 6)
    lower = np.linspace(up, after["lift_mm"], 6)
    spacing = settings["travel_sample_mm"]
    radius = surface.curvature_radius()
    if radius:  # On a curve the pen turns as it crosses: at most 3 degrees a step.
        spacing = min(spacing, (radius * 1000 + up) * np.radians(3))
    across = densify([a, b], spacing)
    st = np.vstack([np.repeat(a[None], len(lift), 0), across[1:-1], np.repeat(b[None], len(lower), 0)])
    lifts = np.concatenate([lift, np.full(len(across) - 2, up), lower])
    s, t = st[:, 0] / 1000, st[:, 1] / 1000
    return {
        "after": before["id"],
        "before": after["id"],
        "lift_mm": up,
        "st_mm": np.round(st, 4).tolist(),
        "lifts_mm": np.round(lifts, 4).tolist(),
        "points_m": np.round(surface.point(s, t, lifts / 1000), 7).tolist(),
        "normals": np.round(surface.normal(s, t), 6).tolist(),
    }


def summary(toolpath):
    by_kind = {}
    for stroke in toolpath["strokes"]:
        by_kind[stroke["kind"]] = by_kind.get(stroke["kind"], 0) + 1
    kinds = ", ".join(f"{count} {kind}" for kind, count in sorted(by_kind.items()))
    warnings = toolpath["legibility"]["warnings"]
    return (f"{len(toolpath['strokes'])} strokes ({kinds}) in {toolpath['layers']} layers, "
            f"{toolpath['draw_length_m']:.2f} m of strand, {toolpath['lift_count']} lifts"
            + (f"; {len(warnings)} legibility warning(s)" if warnings else "; legible"))
