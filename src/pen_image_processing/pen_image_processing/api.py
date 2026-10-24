"""Image-to-drawing boundary; this module knows no robot or surface pose."""
from copy import deepcopy
from hashlib import sha256
from importlib.resources import files
from pathlib import Path
import math

import numpy as np
from PIL import Image, ImageDraw
from shapely import unary_union
import yaml

from .relief import make_relief, save_relief, relief_to_json
from .strokes import shells, hatch, order
from .io import write_json


def extract_drawing(image_path, output_directory=None, *, size_mm=20.0, fill=False, process=None):
    size_mm = float(size_mm)
    if not math.isfinite(size_mm) or not 1 <= size_mm <= 200:
        raise ValueError('size_mm must be finite and between 1 and 200')
    image_path = Path(image_path)
    if image_path.stat().st_size > 20_000_000:
        raise ValueError('input image exceeds 20 MB')
    with Image.open(image_path) as image:
        if image.width * image.height > 20_000_000:
            raise ValueError('input image exceeds 20 megapixels')
        image.verify()
    process = deepcopy(process or yaml.safe_load(files('pen_image_processing').joinpath('process.yaml').read_text()))
    # The first physical workflow is single-layer. Retain the original feature
    # extraction algorithms, with one strand per outline/detail centreline.
    process['levels'] = {'fill': 1, 'outline': [1], 'detail': [1]}
    made = make_relief(image_path, size_mm, process, fill=fill)
    strokes = []
    for level in made['levels']:
        if level.get('kind') == 'ridge':
            for line in level['centerlines']:
                points = np.asarray(line.coords, float)
                if len(points) >= 2:
                    strokes.append({'kind': level['name'], 'points': points, 'closed': bool(line.is_ring)})
        elif fill:
            region = unary_union(level['polygons'])
            width = process['pen']['strand_width_mm']
            strokes.extend(shells(region, width, 1, width))
            strokes.extend(hatch(region.buffer(-width), width * 0.9, 45, width))
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
               'image_sha256': sha256(image_path.read_bytes()).hexdigest(),
               'settings': {'size_mm': size_mm, 'fill': bool(fill)}, 'strokes': output,
               'bounds_mm': made['bounds_mm']}
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
