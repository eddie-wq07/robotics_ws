#!/usr/bin/env python3
"""Rebuild the venue's curated vector regions; leaves the reference bitmap unchanged."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage
from shapely import Polygon

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / 'assets/inputs/venue_map_marked.png'
rgb = np.asarray(Image.open(source).convert('RGB')).astype(float)
r, g, b = rgb.transpose(2, 0, 1)
mask = (b > r + 18) & (g > r + 12) & (b > 150)
labels, _ = ndimage.label(mask)
counts = np.bincount(labels.ravel()); counts[0] = 0
mask = labels == counts.argmax()
mask = ndimage.binary_closing(mask, iterations=3)
# Fill lettering and compression pinholes while retaining the large island.
holes, _ = ndimage.label(ndimage.binary_fill_holes(mask) & ~mask)
counts = np.bincount(holes.ravel())
mask |= (holes > 0) & (counts[holes] < 1800)
contours, hierarchy = cv2.findContours(mask.astype('uint8'), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
outer = max((i for i, h in enumerate(hierarchy[0]) if h[3] < 0), key=lambda i: cv2.contourArea(contours[i]))
def ring(contour):
    pts = cv2.approxPolyDP(contour, 1.4, True)[:, 0, :]
    # Gentle rounding removes pixel stair steps without losing coves.
    return Polygon(pts).buffer(2).buffer(-2).simplify(1.2).exterior.coords[:]
water = {'name': 'water', 'shell': ring(contours[outer]), 'holes': [], 'hatch_angle_deg': 0}
child = hierarchy[0][outer][2]
while child >= 0:
    if cv2.contourArea(contours[child]) > 1800:
        water['holes'].append(ring(contours[child]))
    child = hierarchy[0][child][0]

def bezier(points):
    p = np.asarray(points)
    t = np.linspace(0, 1, 81)[:, None]
    return ((1-t)**3*p[0] + 3*(1-t)**2*t*p[1] + 3*(1-t)*t*t*p[2] + t**3*p[3]).tolist()

def circle(cx, cy, radius):
    a = np.linspace(0, 2*np.pi, 97)
    return np.column_stack((cx+radius*np.cos(a), cy+radius*np.sin(a))).tolist()
outer_arc = bezier([[212,333],[398,-19],[943,-19],[1148,334]])
inner_arc = bezier([[328,392],[499,136],[877,138],[1026,395]])
x_center = [(outer_arc[len(outer_arc)//2][i] + inner_arc[len(inner_arc)//2][i]) / 2
            for i in range(2)]
x_half_size = 27.5
x, y = x_center
# Keep the amphitheatre and shoreline; omit the plaza walks and dome spokes.
strokes = [outer_arc, inner_arc, [[212,333],[328,392]], [[1026,395],[1148,334]],
           water['shell'], *water['holes'],
           [[x-x_half_size,y-x_half_size],[x+x_half_size,y+x_half_size]],
           [[x+x_half_size,y-x_half_size],[x-x_half_size,y+x_half_size]]]
data = {
    'schema_version': 1, 'image_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
    'description': 'Curated venue: smooth amphitheatre arcs, X centered in the brown amphitheatre band between C and D, shoreline and island outlines, filled central dome. Plaza walks and dome spokes omitted.',
    'replace_extracted': True, 'center_on_bounds': True, 'crop_xyxy': [200,55,1160,770],
    'strokes_px': strokes,
    'fills_px': [{'name':'dome', 'shell':circle(674,385,77), 'holes':[], 'hatch_angle_deg':90}],
}
source.with_suffix('.png.strokes.json').write_text(json.dumps(data, indent=2)+'\n')
(ROOT/'assets/designs/venue-map-design.json').write_text(json.dumps({
    'reference': source.name, 'recommended_size_mm':79.89, 'plane_size_mm':[80,100], 'target_footprint_area_fraction':.55,
    'features':['water outline','island outline','filled dome','smooth amphitheatre arcs','two-stroke X'],
    'fill_pitch_mm':.9,'strand_width_mm':1,'stand_off_mm':1.3,'speed_mm_s':12,'travel_speed_mm_s':30,
},indent=2)+'\n')
print(f'Vector water: {len(water["shell"])} outline points; {len(water["holes"])} island opening(s)')
