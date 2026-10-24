"""A picture into a tactile relief: a few raised levels that a fingertip can read.

Tactile graphics read by their edges, not by shading, and fingers tell apart
only two or three heights. So the relief has three levels, each a set of
regions in the marker's flat coordinates (s, t), in millimetres:

* ``fill``    the whole figure, one layer high: you feel where the figure is.
* ``outline`` a ridge just inside the figure's edge, the tallest and widest.
* ``detail``  a lower, narrower ridge along the edges between parts of the
              figure (the door, the windows, the roof line).

A ridge is several strands side by side, fewer on each layer as it rises
(``levels`` in ``configs/process.yaml``), so it has a firm base and a rounded
top, in the style of 2.5D tactile graphics rather than a single raised line.

That is style 1, ``ridges``. Style 2, ``dome``, shapes the parts themselves
instead of their edges: each part (a colour region, or an area enclosed by
lines) swells like a cushion, highest in its middle and rounded at its edges,
on a thin base. A wide part (an apple's body) rises most; a thin one (its
stem) stays low, because no part rises more than ``aspect`` times its
half-width. Parts are parted by a groove down to the base, so a finger finds
where one ends and the next begins. Its heights become stacked layers, each
smaller than the one below.

Two kinds of picture are handled. A *silhouette* (flat-colour clipart) gives
its outline from the figure's edge and its detail from the edges between its
colours. *Line art* (dark lines on a light background) gives its fill from the
area the lines enclose and its detail from the lines themselves.

Nothing here knows the surface; ``slice`` maps the relief onto one.
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from shapely import LinearRing, LineString, Polygon, unary_union
from skimage import color, filters, measure, morphology

PX_PER_MM = 10  # Working resolution: 0.1 mm per pixel.


# ---------------------------------------------------------------------------
# Picture -> figure mask


def load_picture(path):
    image = Image.open(path)
    image = image.convert("RGBA")
    return np.asarray(image)


def figure_mask(rgba):
    """The figure's pixels, the inked pixels within it, and how they were found.

    The figure includes regions enclosed by the drawing (a white window in a
    house); the ink is only what differs from the background. For line art the
    two differ a lot: the ink is the lines, the figure the area they enclose.
    """
    alpha = rgba[..., 3]
    if (alpha < 128).mean() > 0.02:
        mask = alpha >= 128
        return ndimage.binary_fill_holes(mask), mask, "transparency"
    rgb = rgba[..., :3]
    lab = color.rgb2lab(rgb)
    border = np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]])
    background = np.median(border, axis=0)
    near = np.linalg.norm(lab - background, axis=-1) < 12.0
    # Background is what looks like the border *and* is connected to it, so a
    # white window inside a house stays part of the house.
    labels = measure.label(near, connectivity=1)
    edge = np.unique(np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]]))
    mask = ~np.isin(labels, edge[edge > 0])
    if 0.005 < mask.mean() < 0.98:
        return mask, ~near, "background colour"
    gray = color.rgb2gray(rgb)
    ink = gray < filters.threshold_otsu(gray)
    return ndimage.binary_fill_holes(ink), ink, "brightness threshold"


def crop_and_scale(rgba, masks, size_mm, pad_mm=4.0, crop=True):
    """Crop to the figure (``masks[0]``), scale its longest side to ``size_mm``, pad.

    Every mask in ``masks`` is cropped and scaled the same way. With ``crop``
    off the whole picture is kept and scaled, so its centre stays the marker
    centre (a map must keep its own frame).
    """
    rows, cols = np.nonzero(masks[0])
    if not len(rows):
        raise ValueError("no figure found in the picture")
    top, bottom, left, right = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
    if not crop:
        top, bottom, left, right = 0, masks[0].shape[0], 0, masks[0].shape[1]
    scale = size_mm * PX_PER_MM / max(bottom - top, right - left)
    width = max(1, round((right - left) * scale))
    height = max(1, round((bottom - top) * scale))
    pad = round(pad_mm * PX_PER_MM)
    rgb = cv2.resize(np.ascontiguousarray(rgba[top:bottom, left:right, :3]), (width, height),
                     interpolation=cv2.INTER_AREA)
    rgb = cv2.copyMakeBorder(rgb, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(255, 255, 255))
    scaled = [
        np.pad(cv2.resize(mask[top:bottom, left:right].astype(np.float32), (width, height),
                          interpolation=cv2.INTER_AREA) > 0.5, pad)
        for mask in masks
    ]
    return rgb, scaled


# ---------------------------------------------------------------------------
# Pixels <-> marker millimetres


class Grid:
    """Pixel (row, col) to marker (s, t) mm: s right, t up, origin at the centre."""

    def __init__(self, shape):
        self.height, self.width = shape

    def to_mm(self, rows, cols):
        rows, cols = np.asarray(rows, float), np.asarray(cols, float)
        return np.column_stack([(cols + 0.5 - self.width / 2) / PX_PER_MM,
                                (self.height / 2 - rows - 0.5) / PX_PER_MM])

    def to_px(self, points):
        """(s, t) mm to (x, y) pixel coordinates for OpenCV drawing."""
        points = np.asarray(points, float)
        return np.column_stack([points[:, 0] * PX_PER_MM + self.width / 2 - 0.5,
                                self.height / 2 - points[:, 1] * PX_PER_MM - 0.5])


def disk(radius_mm):
    return morphology.disk(max(1, round(radius_mm * PX_PER_MM)))


def clean_mask(mask, min_region_mm, min_area_mm2):
    """Remove parts too thin or too small to feel, and fill pinholes."""
    area = round(min_area_mm2 * PX_PER_MM**2)
    mask = ndimage.binary_opening(mask, disk(min_region_mm / 2))
    mask = ndimage.binary_closing(mask, disk(min_region_mm / 4))
    mask = morphology.remove_small_objects(mask, min_size=area)
    return morphology.remove_small_holes(mask, area_threshold=area)


def mask_polygons(mask, grid, tolerance_mm=0.15):
    """The mask's regions as shapely polygons with holes, in mm."""
    contours, hierarchy = cv2.findContours(mask.astype(np.uint8), cv2.RETR_CCOMP,
                                           cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]

    def ring(contour):
        xy = contour[:, 0, :]
        return grid.to_mm(xy[:, 1], xy[:, 0]) if len(xy) >= 3 else None

    polygons = []
    for index, contour in enumerate(contours):
        if hierarchy[index][3] != -1:
            continue  # A hole; added to its parent below.
        shell = ring(contour)
        if shell is None:
            continue
        holes, child = [], hierarchy[index][2]
        while child != -1:
            hole = ring(contours[child])
            if hole is not None:
                holes.append(hole)
            child = hierarchy[child][0]
        polygon = Polygon(shell, holes).buffer(0).simplify(tolerance_mm)
        if not polygon.is_empty:
            polygons.append(polygon)
    union = unary_union(polygons)
    return [union] if isinstance(union, Polygon) else list(getattr(union, "geoms", []))


# ---------------------------------------------------------------------------
# Thin lines: skeleton -> polylines


_NEIGHBOURS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def trace_skeleton(skeleton, spur_px=0):
    """A one-pixel skeleton as polylines of (row, col), joined through junctions.

    Junction pixels (three or more neighbours) are grouped into nodes; paths
    run between nodes and line ends. Branches shorter than ``spur_px`` that end
    in a free end are removed, then paths through two-way nodes are joined.
    Returns a list of (points, closed).
    """
    pixels = set(zip(*np.nonzero(skeleton)))

    def neighbours(p):
        return [(p[0] + dr, p[1] + dc) for dr, dc in _NEIGHBOURS if (p[0] + dr, p[1] + dc) in pixels]

    degree = {p: len(neighbours(p)) for p in pixels}
    junctions = {p for p in pixels if degree[p] >= 3}
    node_of, nodes = {}, []
    for p in sorted(junctions):  # Group touching junction pixels into one node.
        if p in node_of:
            continue
        stack, members = [p], []
        node_of[p] = len(nodes)
        while stack:
            q = stack.pop()
            members.append(q)
            for r in neighbours(q):
                if r in junctions and r not in node_of:
                    node_of[r] = len(nodes)
                    stack.append(r)
        nodes.append(np.mean(members, axis=0))
    for p in sorted(pixels):
        if degree[p] <= 1:
            node_of[p] = len(nodes)
            nodes.append(np.array(p, float))

    edges, used = [], set()
    for start in sorted(node_of):
        for first in neighbours(start):
            if first in node_of and node_of[first] == node_of[start]:
                continue
            if (start, first) in used:
                continue
            path, previous, current = [start], start, first
            while current not in node_of:
                path.append(current)
                following = [q for q in neighbours(current) if q != previous and q not in path[-3:]]
                if not following:
                    break
                previous, current = current, following[0]
            path.append(current)
            used.add((start, path[1]))
            used.add((current, path[-2]))
            if current in node_of:
                edges.append([node_of[start], node_of[current], path])
    on_edges = {p for _, _, path in edges for p in path}
    loops = []
    for p in sorted(pixels - on_edges - set(node_of)):  # Closed rings with no node.
        if p in on_edges:
            continue
        path, previous, current = [p], None, p
        while True:
            following = [q for q in neighbours(current) if q != previous and q not in path[1:]]
            if not following or following[0] == p:
                break
            previous, current = current, following[0]
            if current == p:
                break
            path.append(current)
        on_edges.update(path)
        if len(path) > 2:
            loops.append((np.array(path, float), True))

    def length(path):
        return float(np.sum(np.linalg.norm(np.diff(np.asarray(path, float), axis=0), axis=1)))

    def counts():
        result = {}
        for a, b, _ in edges:
            result[a] = result.get(a, 0) + 1
            result[b] = result.get(b, 0) + 1
        return result

    # Remove short spurs: a branch from a junction to a free end.
    changed = spur_px > 0
    while changed:
        changed, count = False, counts()
        for edge in list(edges):
            a, b, path = edge
            if a == b:
                continue
            free = count[a] == 1 or count[b] == 1
            junction = count[a] >= 3 or count[b] >= 3
            if free and junction and length(path) < spur_px:
                edges.remove(edge)
                changed = True
                break

    # Join paths through nodes where exactly two meet.
    def oriented(edge, node):
        a, b, path = edge
        return path if a == node else path[::-1]

    changed = True
    while changed:
        changed, count = False, counts()
        for node, n in count.items():
            if n != 2:
                continue
            pair = [e for e in edges if node in (e[0], e[1])]
            if len(pair) != 2 or pair[0] is pair[1]:
                continue
            first, second = pair
            joined = oriented(first, node)[::-1] + oriented(second, node)[1:]
            far_a = first[1] if first[0] == node else first[0]
            far_b = second[1] if second[0] == node else second[0]
            edges.remove(first)
            edges.remove(second)
            edges.append([far_a, far_b, joined])
            changed = True
            break

    lines = []
    for a, b, path in edges:
        points = np.array(path, float)
        # Ends at a junction meet at the junction's centre, so lines join.
        if a < len(nodes) and np.linalg.norm(points[0] - nodes[a]) > 0:
            points = np.vstack([nodes[a], points])
        if b < len(nodes) and np.linalg.norm(points[-1] - nodes[b]) > 0:
            points = np.vstack([points, nodes[b]])
        lines.append((points, a == b and len(points) > 3))
    return lines + loops


def skeleton_lines(raster, grid, merge_gap_mm, min_line_mm, tolerance_mm=0.3):
    """Thin lines in a raster as simplified polylines in mm.

    Lines closer than ``merge_gap_mm`` are first merged into one: the raster is
    thickened by half the gap, so neighbours fuse, then thinned to its centre.
    """
    # Thin first, so thick ink merges by its centre lines, not by its edges.
    centre = morphology.skeletonize(raster)
    fused = ndimage.binary_dilation(centre, disk(merge_gap_mm / 2))
    skeleton = morphology.skeletonize(fused)
    lines = []
    for points, closed in trace_skeleton(skeleton, spur_px=merge_gap_mm * PX_PER_MM):
        mm = grid.to_mm(points[:, 0], points[:, 1])
        if closed:
            geometry = LinearRing(mm).simplify(tolerance_mm)
        else:
            geometry = LineString(mm).simplify(tolerance_mm)
        if geometry.length >= min_line_mm and len(geometry.coords) >= 2:
            lines.append(geometry)
    return lines


# ---------------------------------------------------------------------------
# Detail: edges between colours inside the figure


def colour_regions(rgb, mask, min_region_mm, min_area_mm2, merge_delta_e=18.0, k=6):
    """A label per figure pixel: its colour region, cleaned of slivers."""
    lab = color.rgb2lab(rgb).astype(np.float32)
    samples = lab[mask]
    k = min(k, len(np.unique(samples.round(), axis=0)))
    if k <= 1:
        return np.where(mask, 1, 0)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)
    cv2.setRNGSeed(0)
    _, labels, centres = cv2.kmeans(samples, k, None, criteria, 4, cv2.KMEANS_PP_CENTERS)
    labels = labels.ravel()
    # Merge colours a fingertip-drawing would not tell apart.
    parent = list(range(k))

    def root(i):
        while parent[i] != i:
            i = parent[i]
        return i

    for i in range(k):
        for j in range(i + 1, k):
            if np.linalg.norm(centres[i] - centres[j]) < merge_delta_e:
                parent[root(j)] = root(i)
    labels = np.array([root(i) for i in labels]) + 1
    image = np.zeros(mask.shape, int)
    image[mask] = labels
    # Keep only regions wide and large enough to feel; give the rest to the
    # nearest region that stays (antialiased edges, thin window frames).
    kept = np.zeros(mask.shape, int)
    area = round(min_area_mm2 * PX_PER_MM**2)
    for value in np.unique(labels):
        region = ndimage.binary_opening(image == value, disk(min_region_mm / 2))
        region = morphology.remove_small_objects(region, min_size=area)
        kept[region] = value
    if not kept.any():
        return np.where(mask, 1, 0)
    _, (rows, cols) = ndimage.distance_transform_edt(kept == 0, return_indices=True)
    filled = kept[rows, cols]
    return np.where(mask, filled, 0)


def region_edges(labels, mask):
    """Pixels inside the figure where the colour region changes."""
    edge = np.zeros(labels.shape, bool)
    edge[:, 1:] |= labels[:, 1:] != labels[:, :-1]
    edge[1:, :] |= labels[1:, :] != labels[:-1, :]
    return edge & mask & ndimage.binary_erosion(mask)


# ---------------------------------------------------------------------------
# The relief


def outline_rings(figures, width_mm, min_line_mm):
    """Centre lines of a ridge ``width_mm`` wide just inside each figure's edge."""
    rings = []
    for figure in figures:
        inset = figure.buffer(-width_mm / 2, join_style="mitre", mitre_limit=2.0)
        for polygon in getattr(inset, "geoms", [inset]):
            if polygon.is_empty:
                continue
            for ring in [polygon.exterior, *polygon.interiors]:
                if ring.length >= min_line_mm:
                    rings.append(LinearRing(ring.coords))
    return rings


def keep_away(lines, avoid, gap_mm, min_line_mm):
    """The parts of ``lines`` at least ``gap_mm`` from ``avoid``, long enough to keep."""
    if not avoid:
        return lines, 0
    zone = unary_union([line.buffer(gap_mm) for line in avoid])
    kept, cut = [], 0
    for line in lines:
        rest = LineString(line.coords).difference(zone)
        pieces = [g for g in getattr(rest, "geoms", [rest]) if not g.is_empty]
        if len(pieces) != 1 or pieces[0].length < line.length - 1e-6:
            cut += 1
        kept += [p for p in pieces if p.length >= min_line_mm]
    return kept, cut


def line_art_width_mm(mask):
    """Typical stroke width of the figure: small for line art."""
    skeleton = morphology.skeletonize(mask)
    if not skeleton.any():
        return 0.0
    distance = ndimage.distance_transform_edt(mask)
    return 2 * float(np.median(distance[skeleton])) / PX_PER_MM


def dome_layers(parts, figure, process):
    """Layers of relief per pixel for the dome style: a cushion on every part.

    ``parts`` labels each part (0 outside); ``figure`` is the whole figure,
    which gets the base layers. A part's height at a point grows with its
    distance from the part's edge along a quarter sine wave, so it rises
    steadily to a rounded peak in its middle, up to ``aspect`` times its
    half-width and at most ``max_height_mm``.
    """
    settings, layer = process["dome"], process["pen"]["layer_height_mm"]
    height = np.where(figure, settings["base_layers"] * layer, 0.0)
    groove = disk(settings["groove_mm"] / 2)
    for value in np.unique(parts[parts > 0]):
        part = ndimage.binary_erosion(parts == value, groove)
        if not part.any():
            continue
        depth = ndimage.distance_transform_edt(part) / PX_PER_MM  # mm from the part's edge
        reach = depth.max()
        top = min(settings["max_height_mm"], settings["aspect"] * reach)
        rise = top * np.sin(np.pi / 2 * np.clip(depth / reach, 0, 1))
        height = np.where(part, height + rise, height)
    return np.where(figure, np.maximum(np.round(height / layer), settings["base_layers"]), 0)


def make_relief(picture, size_mm, process, mode="auto", fill=True, style="ridges", crop=True):
    """The relief for ``picture`` at ``size_mm`` (longest side), as a dict.

    ``style`` is ``ridges`` (raised outline and detail ridges on a flat fill)
    or ``dome`` (every part swells like a cushion).
    """
    if style not in ("ridges", "dome"):
        raise ValueError(f"style must be ridges or dome, not {style!r}")
    strand = process["pen"]["strand_width_mm"]
    rules = process["tactile"]
    levels = process["levels"]
    detail_profile, outline_profile = list(levels["detail"]), list(levels["outline"])
    detail_width, outline_width = detail_profile[0] * strand, outline_profile[0] * strand
    rgba = load_picture(picture)
    raw_mask, raw_ink, how = figure_mask(rgba)
    rgb, (mask, ink) = crop_and_scale(rgba, [raw_mask, raw_ink], size_mm, crop=crop)
    grid = Grid(mask.shape)
    width = line_art_width_mm(ink)
    inked = ink.sum() / max(1, mask.sum())
    if mode == "auto":
        mode = "lineart" if width < 2.5 and inked < 0.5 else "silhouette"
    notes = {"mask_from": how, "ink_stroke_width_mm": round(width, 2),
             "inked_share": round(float(inked), 2)}

    if mode == "lineart":
        figure_raster = clean_mask(ndimage.binary_fill_holes(mask | ink), rules["min_region_mm"],
                                   rules["min_area_mm2"])
        # The outline comes from the figure's edge; the other lines are detail.
        detail_raster = ink & ndimage.binary_erosion(figure_raster, disk(outline_width + width))
    else:
        figure_raster = clean_mask(mask, rules["min_region_mm"], rules["min_area_mm2"])
        labels = colour_regions(rgb, figure_raster, rules["min_region_mm"], rules["min_area_mm2"])
        notes["colour_regions"] = int(len(np.unique(labels[labels > 0])))
        detail_raster = region_edges(labels, figure_raster)

    figures = mask_polygons(figure_raster, grid)
    if not figures:
        raise ValueError("nothing large enough to feel is left of the picture")
    if style == "dome":
        if mode == "lineart":  # Parts are the areas the lines enclose; lines become grooves.
            parts = measure.label(figure_raster & ~ndimage.binary_dilation(ink, disk(0.3)),
                                  connectivity=1)
            parts = morphology.remove_small_objects(
                parts, min_size=round(rules["min_area_mm2"] * PX_PER_MM**2))
        else:  # Each connected piece of each colour is its own part.
            parts = measure.label(labels, background=0, connectivity=1)
        stack = dome_layers(parts, figure_raster, process)
        area = round(rules["min_area_mm2"] * PX_PER_MM**2)
        result_levels = []
        for layer in range(1, int(stack.max()) + 1):
            region = morphology.remove_small_objects(stack >= layer, min_size=area)
            polygons = mask_polygons(region, grid)
            if polygons:
                result_levels.append({"name": f"dome-{layer}", "kind": "area", "layers": layer,
                                      "polygons": polygons, "centerlines": []})
        notes["parts"] = int(len(np.unique(parts[parts > 0])))
        extent = unary_union(figures).bounds
        return {
            "mode": mode, "style": "dome", "size_mm": size_mm,
            "strand_width_mm": strand,
            "layer_height_mm": process["pen"]["layer_height_mm"],
            "bounds_mm": [round(v, 3) for v in extent],
            "notes": notes, "levels": result_levels,
        }
    outline = outline_rings(figures, outline_width, rules["min_line_mm"])
    # Detail lines whose ridges would come closer than the gap are merged.
    detail = skeleton_lines(detail_raster, grid, rules["min_gap_mm"] + detail_width,
                            rules["min_line_mm"])
    # A detail ridge keeps a strand's gap from the outline ridge, so the two
    # stay separate; where it comes nearer it stops short.
    detail, cut = keep_away(detail, outline, (outline_width + detail_width) / 2 + strand,
                            rules["min_line_mm"])
    notes["detail_lines_shortened"] = cut

    def ridge(name, lines, profile):
        return {"name": name, "kind": "ridge", "layers": len(profile), "profile": profile,
                "polygons": [line.buffer(profile[0] * strand / 2) for line in lines],
                "centerlines": lines}

    result_levels = []
    if fill:
        result_levels.append({"name": "fill", "kind": "area", "layers": levels["fill"],
                              "polygons": figures, "centerlines": []})
    result_levels.append(ridge("detail", detail, detail_profile))
    result_levels.append(ridge("outline", outline, outline_profile))
    extent = unary_union([g for level in result_levels for g in level["polygons"]]).bounds
    return {
        "mode": mode,
        "style": "ridges",
        "size_mm": size_mm,
        "strand_width_mm": strand,
        "layer_height_mm": process["pen"]["layer_height_mm"],
        "bounds_mm": [round(v, 3) for v in extent],
        "notes": notes,
        "levels": result_levels,
        "_raster": {"rgb": rgb, "shape": mask.shape},
    }


# ---------------------------------------------------------------------------
# Saving and loading


def _rings(polygon):
    return [np.round(np.asarray(polygon.exterior.coords), 4).tolist(),
            *[np.round(np.asarray(r.coords), 4).tolist() for r in polygon.interiors]]


def _flat(geometries):
    for geometry in geometries:
        yield from getattr(geometry, "geoms", [geometry])


def relief_to_json(relief):
    levels = []
    for level in relief["levels"]:
        levels.append({
            "name": level["name"],
            "kind": level["kind"],
            "layers": level["layers"],
            **({"profile": level["profile"]} if "profile" in level else {}),
            "polygons": [_rings(p) for p in _flat(level["polygons"]) if not p.is_empty],
            "centerlines": [
                {"points": np.round(np.asarray(line.coords), 4).tolist(),
                 "closed": isinstance(line, LinearRing)}
                for line in level["centerlines"]
            ],
        })
    return {key: value for key, value in relief.items() if not key.startswith("_")} | {
        "kind": "tactile_relief", "schema_version": 1, "levels": levels}


def relief_from_json(data):
    levels = []
    for level in data["levels"]:
        polygons = [Polygon(rings[0], rings[1:]) for rings in level["polygons"]]
        lines = [LinearRing(c["points"]) if c["closed"] else LineString(c["points"])
                 for c in level["centerlines"]]
        levels.append({**level, "polygons": polygons, "centerlines": lines})
    return {**data, "levels": levels}


def footprints(level, strand_mm):
    """(height in layers, region) pairs that build up ``level``."""
    if level.get("kind") == "ridge":
        return [(layer, unary_union([line.buffer(count * strand_mm / 2)
                                     for line in level["centerlines"]]))
                for layer, count in enumerate(level["profile"], 1) if level["centerlines"]]
    return [(level["layers"], unary_union(level["polygons"]))] if level["polygons"] else []


def heightmap(relief, resolution_mm=0.1, margin_mm=2.0):
    """Layers of relief over a grid on the marker, which level is on top, and the bounds."""
    left, bottom, right, top = relief["bounds_mm"]
    left, bottom, right, top = left - margin_mm, bottom - margin_mm, right + margin_mm, top + margin_mm
    width = int(np.ceil((right - left) / resolution_mm))
    height = int(np.ceil((top - bottom) / resolution_mm))
    layers = np.zeros((height, width), np.uint8)
    which = np.zeros((height, width), np.uint8)

    def px(coords):
        coords = np.asarray(coords, float)
        return np.round(np.column_stack([(coords[:, 0] - left) / resolution_mm,
                                         (top - coords[:, 1]) / resolution_mm])).astype(np.int32)

    for number, level in enumerate(relief["levels"], 1):
        for raised, geometry in footprints(level, relief["strand_width_mm"]):
            region = np.zeros_like(layers)
            for polygon in _flat([geometry]):
                if polygon.is_empty or polygon.geom_type != "Polygon":
                    continue
                part = np.zeros_like(layers)
                cv2.fillPoly(part, [px(polygon.exterior.coords)], 1)
                for ring in polygon.interiors:
                    cv2.fillPoly(part, [px(ring.coords)], 0)
                region |= part
            on_top = (region > 0) & (raised >= layers)
            which[on_top] = number
            layers = np.maximum(layers, region * raised)
    return layers, which, (left, bottom, right, top)


def save_relief(relief, folder, picture):
    from .io import write_json

    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    data = relief_to_json(relief)
    write_json(folder / "relief.json", data)
    layers, _, _ = heightmap(data | {"levels": relief["levels"]})
    tallest = max(1, int(layers.max()))
    shades = np.concatenate([[255], np.linspace(215, 20, tallest)]).astype(np.uint8)
    Image.fromarray(shades[layers]).save(folder / "relief.png")
    Image.open(picture).convert("RGBA").save(folder / "picture.png")
    return data


def summary(data):
    if data.get("style") == "dome":
        left, bottom, right, top = data["bounds_mm"]
        tallest = max(level["layers"] for level in data["levels"]) * data["layer_height_mm"]
        return (f"{data['mode']}, dome style, {right - left:.0f} x {top - bottom:.0f} mm, "
                f"{data['notes'].get('parts', 0)} part(s) rising to {tallest:.1f} mm "
                f"in {len(data['levels'])} layers")
    counts = {level["name"]: level for level in data["levels"]}
    lines = len(counts.get("detail", {}).get("centerlines", []))
    rings = len(counts.get("outline", {}).get("centerlines", []))
    figures = len(counts.get("fill", {}).get("polygons", []))
    left, bottom, right, top = data["bounds_mm"]
    heights = ", ".join(
        f"{level['name']} {level['layers'] * data['layer_height_mm']:.1f} mm"
        for level in data["levels"] if level["polygons"])
    return (f"{data['mode']}, {right - left:.0f} x {top - bottom:.0f} mm, "
            f"{figures} filled region(s), {rings} outline ring(s), {lines} detail line(s); "
            f"heights: {heights}")
