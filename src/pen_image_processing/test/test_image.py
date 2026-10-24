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


@pytest.mark.parametrize('size',[0,-2,float('nan'),201])
def test_invalid_scale_refused(size):
    with pytest.raises(ValueError):extract_drawing(EXAMPLE,size_mm=size)
