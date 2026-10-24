from pathlib import Path
import pytest
from pen_image_processing.api import extract_drawing

EXAMPLE=Path(__file__).resolve().parents[2]/'3d-pen/examples/apple.png'


def test_image_to_independent_drawing(tmp_path):
    result=extract_drawing(EXAMPLE,tmp_path,size_mm=20)
    assert result['kind']=='pen_drawing' and result['units']=='m'
    assert result['strokes']
    assert all(len(s['points_m'])>=2 for s in result['strokes'])
    assert (tmp_path/'drawing.json').exists()
    assert (tmp_path/'drawing.png').exists()
    assert 'robot_sn' not in result


def test_fill_does_not_retrace_its_shell_as_an_outline(monkeypatch):
    from shapely.geometry import box
    from pen_image_processing import api
    region=box(-5,-5,5,5)
    outline=region.buffer(-.5,join_style='mitre').exterior
    relief={'bounds_mm':[-5,-5,5,5], 'levels':[
        {'name':'fill','kind':'area','polygons':[region]},
        {'name':'outline','kind':'ridge','centerlines':[outline]}]}
    monkeypatch.setattr(api,'make_relief',lambda *args,**kwargs:relief)
    filled=extract_drawing(EXAMPLE,size_mm=20,fill=True)
    assert sum(s['kind']=='shell' for s in filled['strokes'])==1
    assert any(s['kind']=='hatch' for s in filled['strokes'])
    assert not any(s['kind']=='outline' for s in filled['strokes'])
    outline_only=extract_drawing(EXAMPLE,size_mm=20,fill=False)
    assert len(outline_only['strokes'])==1 and outline_only['strokes'][0]['kind']=='outline'


@pytest.mark.parametrize('size',[0,-2,float('nan'),201])
def test_invalid_scale_refused(size):
    with pytest.raises(ValueError):extract_drawing(EXAMPLE,size_mm=size)


def test_explicit_small_x_survives_extraction_and_scales(tmp_path):
    import hashlib
    import json
    import shutil
    import numpy as np
    image=tmp_path/'apple.png';shutil.copy2(EXAMPLE,image)
    from PIL import Image
    with Image.open(image) as source:w,h=source.size
    points=[[[w*.45,h*.45],[w*.55,h*.55]],[[w*.55,h*.45],[w*.45,h*.55]]]
    sidecar=image.with_suffix('.png.strokes.json')
    sidecar.write_text(json.dumps({'schema_version':1,
        'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),'strokes_px':points}))
    small=extract_drawing(image,tmp_path/'small',size_mm=20)
    large=extract_drawing(image,tmp_path/'large',size_mm=40)
    a=[s for s in small['strokes'] if s['kind']=='annotation']
    b=[s for s in large['strokes'] if s['kind']=='annotation']
    assert len(a)==len(b)==2
    assert small['annotation_sha256']==hashlib.sha256(sidecar.read_bytes()).hexdigest()
    for first,second in zip(a,b):
        assert np.allclose(sorted(np.linalg.norm(np.asarray(first['points_m']),axis=1)*2),
                           sorted(np.linalg.norm(np.asarray(second['points_m']),axis=1)),atol=.0001)
    sidecar.write_text(json.dumps({'schema_version':1,'image_sha256':'wrong','strokes_px':points}))
    with pytest.raises(ValueError,match='does not match'):
        extract_drawing(image,size_mm=20)


def test_curated_fill_keeps_island_open_without_global_fill(tmp_path):
    import hashlib
    import json
    import shutil
    from PIL import Image
    from shapely import LineString, Polygon
    image=tmp_path/'map.png';shutil.copy2(EXAMPLE,image)
    with Image.open(image) as source:w,h=source.size
    def ring(lo,hi):
        return [[w*lo,h*lo],[w*hi,h*lo],[w*hi,h*hi],[w*lo,h*hi]]
    image.with_suffix('.png.strokes.json').write_text(json.dumps({
        'schema_version':1,'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
        'replace_extracted':True,'crop_xyxy':[0,0,w,h],
        'strokes_px':[[[w*.1,h*.1],[w*.2,h*.1]]],
        'fills_px':[{'name':'water','shell':ring(.1,.9),'holes':[ring(.4,.6)]}]}))
    drawing=extract_drawing(image,size_mm=40,fill=False)
    fills=[s for s in drawing['strokes'] if s['kind'].startswith('water-')]
    assert {s['kind'] for s in fills}=={'water-shell','water-hatch'}
    assert all(s['kind']=='annotation' or s in fills for s in drawing['strokes'])
    scale=.04/max(w,h)
    island=Polygon([((x+.5-w/2)*scale,(h/2-y-.5)*scale) for x,y in ring(.4,.6)]).buffer(-.0002)
    assert all(not LineString(s['points_m']).intersects(island) for s in fills)


def test_curated_drawing_centers_geometry_without_rescaling(tmp_path):
    import hashlib,json,shutil,numpy as np
    from PIL import Image
    image=tmp_path/'centered.png';shutil.copy2(EXAMPLE,image)
    with Image.open(image) as source:w,h=source.size
    sidecar=image.with_suffix('.png.strokes.json')
    data={'schema_version':1,'image_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
          'replace_extracted':True,'strokes_px':[[[w*.2,h*.2],[w*.4,h*.3]]]}
    sidecar.write_text(json.dumps(data));before=extract_drawing(image,size_mm=40)
    data['center_on_bounds']=True;sidecar.write_text(json.dumps(data))
    after=extract_drawing(image,size_mm=40)
    a=np.array(before['strokes'][0]['points_m']);b=np.array(after['strokes'][0]['points_m'])
    assert np.allclose(np.ptp(a,axis=0),np.ptp(b,axis=0))
    assert np.allclose((b.min(axis=0)+b.max(axis=0))/2,0,atol=1e-9)
