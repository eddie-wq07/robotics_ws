from pathlib import Path
import pytest
import yaml
from pen_description.calibration import load_profile, compose, transform, xacro_mappings


def test_photo_board_height_and_calibration_boundary(tmp_path):
    root=Path(__file__).resolve().parents[1]
    profile=load_profile(root/'config/photo_board.yaml',mode='preview')
    assert profile['surface']['size_m']==[.12,.12,.00535]
    assert profile['T_jig_surface']['xyz_m'][2]-.075==pytest.approx(.00535)
    world=compose(compose(transform(profile,'T_world_base'),transform(profile,'T_base_jig')),
                  transform(profile,'T_jig_surface'))
    assert world[0]==pytest.approx(profile['T_world_surface']['xyz_m'])
    assert xacro_mappings(profile)['plane_visual_mesh'].endswith('/photo_board/photo_board.glb')
    assert load_profile(root/'config/photo_board.yaml',mode='real')['surface']['type']=='plane'
    profile['calibrated'] = False
    uncalibrated = tmp_path/'photo_board.yaml'
    uncalibrated.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='requires calibrated'):
        load_profile(uncalibrated,mode='real')
