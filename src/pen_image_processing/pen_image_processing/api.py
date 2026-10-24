"""Image-to-drawing boundary; this module knows no robot or surface pose."""
from copy import deepcopy
from hashlib import sha256
from importlib.resources import files
from pathlib import Path
import math
import json

import numpy as np
from PIL import Image, ImageDraw
from shapely import LineString, LinearRing, Polygon, unary_union
from shapely.affinity import translate
import yaml

from .relief import make_relief, save_relief, relief_to_json, PX_PER_MM
from .strokes import shells, hatch, order
from .io import write_json


def annotation_path(image_path):
    path = Path(image_path)
    return path.with_suffix(path.suffix + '.strokes.json')


def _add_annotations(image_path, image_size, image_hash, made, width_mm):
    """Preserve explicit pixel-space marks through the same crop/scale as the image."""
    path = annotation_path(image_path)
    if not path.exists():
        return None
    raw = path.read_bytes()
    if len(raw) > 1_000_000:
        raise ValueError('Annotation file exceeds 1 MB')
    data = json.loads(raw)
    if data.get('schema_version') != 1 or data.get('image_sha256') != image_hash:
        raise ValueError('Annotation file does not match the source image')
    strokes = data.get('strokes_px', [])
    if not isinstance(strokes, list) or not 1 <= len(strokes) <= 1000:
        raise ValueError('Annotation file requires 1..1000 strokes')
    left, top, right, bottom = made['_source_frame']['crop_xyxy']
    scaled_w, scaled_h = made['_source_frame']['scaled_size_px']
    if 'crop_xyxy' in data:
        crop = np.asarray(data['crop_xyxy'], dtype=float)
        if (crop.shape != (4,) or not np.isfinite(crop).all()
                or not 0 <= crop[0] < crop[2] <= image_size[0]
                or not 0 <= crop[1] < crop[3] <= image_size[1]):
            raise ValueError('Annotation crop must be inside the source image')
        left, top, right, bottom = crop
        scale = made['size_mm'] * PX_PER_MM / max(right-left, bottom-top)
        scaled_w, scaled_h = (right-left)*scale, (bottom-top)*scale

    def to_mm(stroke):
        points = np.asarray(stroke, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 2 or not 2 <= len(points) <= 10000
                or not np.isfinite(points).all() or (points < 0).any()
                or (points[:, 0] >= image_size[0]).any()
                or (points[:, 1] >= image_size[1]).any()):
            raise ValueError('Annotation points must be finite pixels inside the source image')
        return np.column_stack([
            ((points[:, 0] + .5 - left) * scaled_w / (right-left) - scaled_w/2) / PX_PER_MM,
            (scaled_h/2 - (points[:, 1] + .5 - top) * scaled_h / (bottom-top)) / PX_PER_MM,
        ])

    if data.get('replace_extracted') is True:
        made['levels'] = []
    lines = []
    for stroke in strokes:
        line = LineString(to_mm(stroke))
        if line.length <= 0:
            raise ValueError('Annotation stroke must have positive length')
        lines.append(line)
    made['levels'].append({'name': 'annotation', 'kind': 'ridge', 'layers': 1,
                           'profile': [1], 'centerlines': lines,
                           'polygons': [line.buffer(width_mm/2) for line in lines]})
    fills = data.get('fills_px', [])
    if not isinstance(fills, list) or len(fills) > 100:
        raise ValueError('Annotation file allows at most 100 filled regions')
    for region in fills:
        polygon = Polygon(to_mm(region['shell']), [to_mm(hole) for hole in region.get('holes', [])])
        if not polygon.is_valid or polygon.area <= 0:
            raise ValueError('Annotation fill must be a valid polygon with positive area')
        angle = float(region.get('hatch_angle_deg', 45))
        if not math.isfinite(angle):
            raise ValueError('Annotation hatch angle must be finite')
        made['levels'].append({'name': str(region.get('name', 'annotation-fill')),
                               'kind': 'area', 'layers': 1, 'polygons': [polygon],
                               'centerlines': [], 'explicit_fill': True, 'hatch_angle_deg': angle})
    made['bounds_mm'] = list(unary_union([
        poly for level in made['levels'] for poly in level['polygons']]).bounds)
    if data.get('center_on_bounds') is True:
        left, bottom, right, top = made['bounds_mm']
        dx, dy = -(left+right)/2, -(bottom+top)/2
        for level in made['levels']:
            level['centerlines'] = [translate(line, dx, dy) for line in level['centerlines']]
            level['polygons'] = [translate(poly, dx, dy) for poly in level['polygons']]
        made['bounds_mm'] = [left+dx,bottom+dy,right+dx,top+dy]
    return sha256(raw).hexdigest()


def extract_drawing(image_path, output_directory=None, *, size_mm=20.0, fill=False, process=None):
    size_mm = float(size_mm)
    if not math.isfinite(size_mm) or not 1 <= size_mm <= 200:
        raise ValueError('size_mm must be finite and between 1 and 200')
    image_path = Path(image_path)
    if image_path.stat().st_size > 20_000_000:
        raise ValueError('input image exceeds 20 MB')
    with Image.open(image_path) as image:
        image_size = image.size
        if image.width * image.height > 20_000_000:
            raise ValueError('input image exceeds 20 megapixels')
        image.verify()
    process = deepcopy(process or yaml.safe_load(files('pen_image_processing').joinpath('process.yaml').read_text()))
    # The first physical workflow is single-layer. Retain the original feature
    # extraction algorithms, with one strand per outline/detail centreline.
    process['levels'] = {'fill': 1, 'outline': [1], 'detail': [1]}
    made = make_relief(image_path, size_mm, process, fill=fill)
    image_hash = sha256(image_path.read_bytes()).hexdigest()
    annotation_hash = _add_annotations(image_path, image_size, image_hash, made,
                                       process['pen']['strand_width_mm'])
    strokes = []
    for level in made['levels']:
        if level.get('kind') == 'ridge':
            for line in level['centerlines']:
                points = np.asarray(line.coords, float)
                if len(points) >= 2:
                    strokes.append({'kind': level['name'], 'points': points, 'closed': bool(line.is_ring)})
        elif fill or level.get('explicit_fill'):
            region = unary_union(level['polygons'])
            width = process['pen']['strand_width_mm']
            boundary = shells(region, width, 1, width)
            interior = hatch(region.buffer(-width*.9), width * 0.9, level.get('hatch_angle_deg', 45), width)
            if level.get('explicit_fill'):
                for stroke in boundary + interior:
                    stroke['kind'] = level['name'] + '-' + stroke['kind']
            strokes.extend(boundary + interior)
    if fill:
        # Filled regions already have a perimeter shell. Do not trace the same
        # boundary again as an outline, which would over-extrude every edge.
        boundaries=[LinearRing(s['points']) for s in strokes if s['kind']=='shell' and s['closed']]
        strokes=[s for s in strokes if not (s['kind']=='outline' and s['closed'] and
                  any(LinearRing(s['points']).equals(boundary) for boundary in boundaries))]
    ordered, _ = order(strokes, np.zeros(2))
    if not ordered:
        raise ValueError('image produced no drawable strokes')
    output = []
    for i, stroke in enumerate(ordered):
        points = np.asarray(stroke['points'], float)
        if stroke['closed'] and not np.allclose(points[0], points[-1]):
            points = np.vstack([points, points[0]])
        output.append({'id': f'stroke-{i:04d}', 'kind': stroke['kind'], 'closed': bool(stroke['closed']),
                       'points_m': (points / 1000).tolist()})
    drawing = {'schema_version': 1, 'kind': 'pen_drawing', 'units': 'm', 'frame': 'drawing',
               'axis_convention': 's right, t up, origin image centre', 'size_mm': size_mm,
               'image_sha256': image_hash,
               'settings': {'size_mm': size_mm, 'fill': bool(fill)}, 'strokes': output,
               'bounds_mm': made['bounds_mm']}
    if annotation_hash is not None:
        drawing['annotation_sha256'] = annotation_hash
    if output_directory is not None:
        out = Path(output_directory); out.mkdir(parents=True, exist_ok=True)
        save_relief(made, out, image_path)
        write_json(out / 'drawing.json', drawing)
        _save_drawing_image(drawing, out / 'drawing.png')
    return drawing


def _save_drawing_image(drawing, path):
    canvas = Image.new('RGB', (640, 640), 'white')
    painter = ImageDraw.Draw(canvas)
    scale = 560 / (drawing['size_mm'] / 1000)
    for stroke in drawing['strokes']:
        xy = [(320 + p[0] * scale, 320 - p[1] * scale) for p in stroke['points_m']]
        painter.line(xy, fill='#172b42', width=2)
    canvas.save(path)
