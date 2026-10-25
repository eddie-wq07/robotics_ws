from pathlib import Path
from unittest.mock import patch
import json

import pytest
import yaml

from pen_bringup.profiles import ProfileCatalog
from pen_bringup.service import Workflow, WorkflowError


def save(root, name, image, *, surface='plane', size=42, real=False):
    folder = root/'src/pen_description/config'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder/name
    data = {'profile_id': name, 'surface': {'type': surface},
        'calibrated': real, 'fake_only': not real,
        'selection': {'image': image, 'surface': surface, 'size_mm': size, 'fill': False}}
    if real:
        data['qualification'] = dict.fromkeys(('rt', 'clearance', 'pen', 'tool'), True)
        data['qualified'] = {'cartesian': True, 'rt': True}
    path.write_text(yaml.safe_dump(data))
    return path


def test_named_yaml_discovery_image_mapping_and_hot_reload(tmp_path):
    first = save(tmp_path, 'portrait.yml', 'family.png')
    save(tmp_path, 'ball.yaml', 'lantern.png', surface='sphere', size=36)
    catalog = ProfileCatalog(tmp_path, first)
    assert catalog.resolve({'image': 'family.png'})['id'] == 'portrait.yml'
    assert catalog.resolve({'image': 'lantern.png'})['surface'] == 'sphere'
    assert catalog.resolve({'profile': 'portrait.yml', 'image': 'lantern.png'})['surface'] == 'plane'
    old_hash = catalog.records()['portrait.yml']['sha256']
    save(tmp_path, 'portrait.yml', 'family.png', size=120)
    assert catalog.records()['portrait.yml']['selection']['size_mm'] == 120
    assert catalog.records()['portrait.yml']['sha256'] != old_hash
    save(tmp_path, 'new_photo.yaml', 'new.png', size=30)
    assert catalog.resolve({'image': 'new.png'})['id'] == 'new_photo.yaml'


def test_tylenol_images_select_their_own_profile_even_with_previous_selection():
    root=Path(__file__).resolve().parents[3]
    catalog=ProfileCatalog(root)
    for image,expected,previous in [('sun.png','tylenol_sun.yaml','tylenol_moon.yaml'),
                                    ('moon.png','tylenol_moon.yaml','tylenol_sun.yaml')]:
        assert catalog.resolve({'image':image,'fallback_profile':previous})['id']==expected


def test_ambiguity_and_no_arbitrary_profile_paths(tmp_path):
    save(tmp_path, 'one.yaml', 'same.png')
    save(tmp_path, 'two.yaml', 'same.png', surface='sphere')
    catalog = ProfileCatalog(tmp_path)
    with pytest.raises(ValueError, match='multiple saved profiles'):
        catalog.resolve({'image': 'same.png'})
    assert catalog.resolve({'image': 'same.png', 'surface': 'sphere'})['id'] == 'two.yaml'
    with pytest.raises(ValueError, match='Unknown saved'):
        catalog.resolve({'profile': '../secrets.yaml'})


def test_resolved_profile_defaults_reach_worker_and_job(tmp_path):
    first = save(tmp_path, 'one.yaml', 'one.png', size=45)
    second = save(tmp_path, 'two.yaml', 'two.png', surface='sphere', size=36)
    flow = Workflow(tmp_path, 'preview', first)
    (flow.inputs/'two.png').write_bytes(b'fixture; worker is mocked')
    with patch('pen_bringup.service.threading.Thread'):
        result = flow.prepare('user', {'image': 'two.png'})
    request = json.loads((Path(flow.jobs[result['id']]['folder'])/'request.json').read_text())
    assert result['profile'] == 'two.yaml'
    assert result['surface'] == 'sphere'
    assert result['size_mm'] == 36
    assert request['profile_path'] == str(second)
    assert flow.profile_for_job(result) == second
    assert flow.profile == first
    with pytest.raises(WorkflowError, match='does not match'):
        flow.prepare('user', {'profile': 'one.yaml', 'surface': 'sphere', 'image': 'two.png'})


def test_real_mode_cannot_be_enabled_by_a_profile(tmp_path):
    path = save(tmp_path, 'photo.yaml', 'photo.png', size=120)
    flow = Workflow(tmp_path, 'real', path)
    assert flow.profiles.public('real')[0]['available'] is False
    with pytest.raises(WorkflowError, match='preview-only'):
        flow.resolve_selection({'image': 'photo.png'})
    assert flow.mode == 'real'


def test_unmapped_photo_keeps_chosen_profile_and_explicit_surface_wins(tmp_path):
    portrait = save(tmp_path, 'portrait.yaml', 'family.png')
    ball = save(tmp_path, 'sphere.yaml', 'lantern.png', surface='sphere')
    catalog = ProfileCatalog(tmp_path, portrait)
    assert catalog.resolve({'image': 'new.png', 'fallback_profile': 'sphere.yaml'})['path'] == ball
    assert catalog.resolve({'image': 'family.png', 'surface': 'sphere'})['path'] == ball


def test_malformed_helpers_are_ignored_and_rename_is_discovered(tmp_path):
    path = save(tmp_path, 'photo.yaml', 'photo.png')
    (path.parent/'helper.yaml').write_text('selection: {}\nsurface: broken\n')
    (path.parent/'invalid.yaml').write_text('selection: [not, a, mapping]\n')
    catalog = ProfileCatalog(tmp_path, path)
    assert list(catalog.records()) == ['photo.yaml']
    path.rename(path.with_name('renamed.yaml'))
    assert catalog.resolve({'image': 'photo.png'})['id'] == 'renamed.yaml'


def test_execute_keeps_exact_yaml_when_two_profiles_share_a_surface(tmp_path):
    from pen_bringup.service import digest
    first = save(tmp_path, 'first.yaml', 'one.png', real=True)
    second = save(tmp_path, 'second.yaml', 'two.png', real=True)
    class Bridge:
        calls = []
        def ready(self): return True
        def snapshot(self): return {}
        def execute(self, *args): self.calls.append(args)
    bridge = Bridge()
    flow = Workflow(tmp_path, 'real', first, bridge)
    (flow.inputs/'two.png').write_bytes(b'image')
    with patch('pen_bringup.service.threading.Thread'):
        result = flow.prepare('user', {'profile': 'second.yaml'})
    job = flow.jobs[result['id']]
    path = Path(job['folder'])/'job.json'
    path.write_text('{}')
    job.update(state='ready', preview_token='token', sha256=digest(path))
    flow.session('user')['reviewed'] = job['id']
    first.write_text(first.read_text() + '\n# Changed unrelated startup profile\n')
    flow.execute('user', job['id'], 'token')
    assert len(bridge.calls) == 1
    assert flow.profile_for_job(job) == second


def test_flag_edit_is_atomic_minimal_and_does_not_change_qualification(tmp_path):
    from pen_bringup.service import digest
    path = save(tmp_path, 'photo.yaml', 'photo.png')
    with path.open('a') as f:
        f.write('# Keep operator notes\nqualification:\n  clearance: false\n')
    before = path.read_text()
    catalog = ProfileCatalog(tmp_path, path)
    updated = catalog.update_flags('photo.yaml', {'calibrated': True, 'fake_only': False}, digest(path))
    assert path.read_text() == before.replace('calibrated: false', 'calibrated: true').replace('fake_only: true', 'fake_only: false')
    assert updated['data']['qualification']['clearance'] is False
    assert 'clearance' in ' '.join(catalog.execution_blockers(updated, 'real'))
    with pytest.raises(ValueError, match='Profile changed'):
        catalog.update_flags('photo.yaml', {'fake_only': True}, 'old-revision')
    with pytest.raises(ValueError, match='Only boolean'):
        catalog.update_flags('photo.yaml', {'calibrated': 'true'}, digest(path))
    with pytest.raises(ValueError, match='Only boolean'):
        catalog.update_flags('photo.yaml', {'qualification': True}, digest(path))
    assert not list(path.parent.glob('.profile-flags-*'))


def test_flag_edit_invalidates_all_browser_reviews_and_rejects_active_job(tmp_path):
    from pen_bringup.service import digest
    path = save(tmp_path, 'photo.yaml', 'photo.png')
    flow = Workflow(tmp_path, 'preview', path)
    flow.session('first')['reviewed'] = 'job'
    flow.session('second')['reviewed'] = 'job'
    flow.jobs['job'] = {'id': 'job', 'profile': 'photo.yaml', 'state': 'ready'}
    request = {'profile': 'photo.yaml', 'profile_sha256': digest(path), 'flags': {'calibrated': True}}
    flow.active_job = 'job'
    with pytest.raises(WorkflowError, match='current operation'):
        flow.update_profile_flags('first', request)
    assert yaml.safe_load(path.read_text())['calibrated'] is False
    flow.active_job = None
    flow.update_profile_flags('first', request)
    assert flow.jobs['job']['state'] == 'stale'
    assert all(s['reviewed'] is None for s in flow.sessions.values())
    assert flow.mode == 'preview' and flow.bridge is None
