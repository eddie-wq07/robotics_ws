"""A relief on a surface into pen strokes, layer by layer.

All the slicing happens in the marker's flat coordinates (s, t), in mm. Layer
``k`` (1 = first) covers every level at least ``k`` layers high:

* filled regions get shells (strands following the edge) and a zigzag hatch
  inside, turned 90 degrees between layers;
* ridges (the outline and detail) get parallel passes along their centre
  line, as many as the ridge is strands wide on that layer, wherever the
  layer has no filled region under them.

Each stroke is then laid on the surface at the layer's height above it, with
the surface normal at every point (the pen is held along the inward normal).
Between strokes the pen lifts along the normal, moves in a straight line in
(s, t) at that lifted height, which on a can or ball follows the surface
instead of cutting through it, and lowers again.
"""

import numpy as np
from shapely import LinearRing, LineString, Polygon, unary_union
from shapely.affinity import rotate



def _parts(geometry):
    if geometry is None or geometry.is_empty:
        return []
    return [g for g in getattr(geometry, "geoms", [geometry]) if not g.is_empty]


def _rings(region):
    for polygon in _parts(region):
        yield polygon.exterior
        yield from polygon.interiors


def shells(thick, width, count, min_length):
    """Closed strands following the edges of ``thick``, outermost first."""
    strokes = []
    for index in range(count):
        inset = thick.buffer(-(index + 0.5) * width, join_style="mitre", mitre_limit=2.0)
        for ring in _rings(inset):
            if ring.length >= min_length:
                strokes.append({"kind": "shell", "points": np.asarray(ring.coords)[:-1],
                                "closed": True})
    return strokes


def hatch(zone, pitch, angle_deg, width):
    """Zigzag strands filling ``zone`` with lines ``pitch`` apart at ``angle_deg``."""
    if zone.is_empty:
        return []
    turned = rotate(zone, -angle_deg, origin=(0, 0))
    left, bottom, right, top = turned.bounds
    rows = []
    for y in np.arange(bottom + pitch / 2, top, pitch):
        cut = turned.intersection(LineString([(left - 1, y), (right + 1, y)]))
        pieces = sorted((np.asarray(p.coords) for p in _parts(cut) if p.length > 0.2 * width),
                        key=lambda xy: xy[:, 0].min())
        rows.append([xy[np.argsort(xy[:, 0])][[0, -1]] for xy in pieces])
    inside = turned.buffer(0.05 * width)
    used = [[False] * len(row) for row in rows]
    strokes = []
    for start_row, row in enumerate(rows):
        for start_index in range(len(row)):
            if used[start_row][start_index]:
                continue
            used[start_row][start_index] = True
            points = [row[start_index][0], row[start_index][1]]
            forward, r = False, start_row
            while r + 1 < len(rows):
                end = points[-1]
                best = None
                for index, segment in enumerate(rows[r + 1]):
                    if used[r + 1][index]:
                        continue
                    start = segment[1] if not forward else segment[0]
                    link = LineString([end, start])
                    if link.length < 3 * pitch and inside.covers(link):
                        if best is None or link.length < best[0]:
                            best = (link.length, index, segment)
                if best is None:
                    break
                _, index, segment = best
                used[r + 1][index] = True
                points += [segment[1], segment[0]] if not forward else [segment[0], segment[1]]
                forward, r = not forward, r + 1
            line = rotate(LineString(points), angle_deg, origin=(0, 0))
            strokes.append({"kind": "hatch", "points": np.asarray(line.coords), "closed": False})
    return strokes


def offset_lines(line, distance):
    """``line`` moved sideways by ``distance`` (either side for a closed ring)."""
    if abs(distance) < 1e-9:
        return [line]
    if isinstance(line, LinearRing):
        area = Polygon(line).buffer(0)
        moved = area.buffer(distance, join_style="mitre", mitre_limit=2.0)
        return [LinearRing(ring.coords) for ring in _rings(moved) if ring.length > 0]
    return _parts(line.offset_curve(distance, join_style="mitre", mitre_limit=2.0))


def ridge_passes(levels, layer, thick, width, min_length):
    """Side-by-side passes for each ridge on this layer, outside the filled regions."""
    covered = thick.buffer(-0.2 * width) if not thick.is_empty else None
    strokes = []
    for level in levels:
        if level.get("kind") != "ridge" or level["layers"] < layer:
            continue
        count = level["profile"][layer - 1]
        offsets = [(index - (count - 1) / 2) * width for index in range(count)]
        for line in level["centerlines"]:
            for offset in offsets:
                for moved in offset_lines(line, offset):
                    closed_ring = isinstance(moved, LinearRing)
                    rest = LineString(moved.coords)
                    if covered is not None and not covered.is_empty:
                        rest = rest.difference(covered)
                    for piece in _parts(rest):
                        if piece.length < min_length:
                            continue
                        points = np.asarray(piece.coords)
                        closed = closed_ring and np.allclose(points[0], points[-1])
                        strokes.append({"kind": "ridge", "level": level["name"],
                                        "points": points[:-1] if closed else points,
                                        "closed": closed})
    return strokes


def order(strokes, position):
    """Greedy nearest-start order; open strokes may run backwards, loops start anywhere."""
    remaining, result = list(strokes), []
    while remaining:
        best = None
        for index, stroke in enumerate(remaining):
            points = stroke["points"]
            if stroke["closed"]:
                distances = np.linalg.norm(points - position, axis=1)
                choice = (float(distances.min()), index, int(distances.argmin()))
            else:
                start, end = np.linalg.norm(points[0] - position), np.linalg.norm(points[-1] - position)
                choice = (float(min(start, end)), index, 0 if start <= end else -1)
            if best is None or choice[0] < best[0]:
                best = choice
        _, index, at = best
        stroke = remaining.pop(index)
        points = stroke["points"]
        if stroke["closed"]:
            points = np.vstack([np.roll(points, -at, axis=0), np.roll(points, -at, axis=0)[:1]])
        elif at == -1:
            points = points[::-1]
        result.append({**stroke, "points": points})
        position = points[-1]
    return result, position


def densify(points, spacing):
    """Points no farther apart than ``spacing``, keeping every corner."""
    points = np.asarray(points, float)
    out = [points[0]]
    for a, b in zip(points[:-1], points[1:]):
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / spacing)))
        out.extend(a + (b - a) * (np.arange(1, count + 1)[:, None] / count))
    return np.asarray(out)


