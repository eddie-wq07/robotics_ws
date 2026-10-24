import json

import pytest

from pen_bringup.prepared_job import planner_revision, restore_prepared
from pen_bringup.preview_cache import REQUIRED, checksum


def test_prepared_job_checks_content_settings_and_live_start(tmp_path):
    source = tmp_path / 'assets/outputs/saved'
    source.mkdir(parents=True)
    output = tmp_path / 'loaded'
    output.mkdir()
    request = {'size_mm': 36, 'surface': {'type': 'cylinder'}, 'fill': False,
               'execution_mode': 'real', 'execution_backend': 'cartesian',
               'robot_preview': {'joints': ['kinematics'], 'initial_positions': {'j1': .1}}}
    for name in REQUIRED:
        (source / name).write_text('{}')
    manifest = {'request': {k: v for k, v in request.items() if k != 'robot_preview'},
                'input_sha256': 'input', 'profile_sha256': 'profile',
                'planner_revision': planner_revision(tmp_path),
                'joints': ['kinematics'], 'start_positions': {'j1': .1},
                'files': {name: checksum(source / name) for name in REQUIRED}}
    (source / 'prepared-manifest.json').write_text(json.dumps(manifest))
    assert restore_prepared(tmp_path, source, request, 'input', 'profile', output)
    assert not restore_prepared(tmp_path, source, request, 'changed', 'profile', output)
    assert not restore_prepared(tmp_path, source, request, 'input', 'changed', output)
    request['robot_preview']['initial_positions']['j1'] = .2
    assert not restore_prepared(tmp_path, source, request, 'input', 'profile', output)
    request['robot_preview']['initial_positions']['j1'] = .10001
    (source / 'job.json').write_text('{"changed":true}')
    with pytest.raises(ValueError, match='changed'):
        restore_prepared(tmp_path, source, request, 'input', 'profile', output)
