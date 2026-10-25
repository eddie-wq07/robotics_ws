import json
import time
from types import SimpleNamespace

from pen_bringup.preview_cache import PreviewCache, REQUIRED
from pen_bringup.service import Workflow
from copy import deepcopy


def test_cache_key_tracks_inputs_model_pose_and_code(tmp_path):
    cache = PreviewCache(tmp_path)
    request = {'size_mm': 36, 'robot_preview': {'initial_positions': {'joint1': .1},
                                             'initial_source': {'recorded_utc': 'old'}}}
    key = cache.key(request, 'image', 'profile')
    request['robot_preview']['initial_source']['recorded_utc'] = 'new'
    assert cache.key(request, 'image', 'profile') == key
    assert cache.key(request, 'changed-image', 'profile') != key
    assert cache.key(request, 'image', 'changed-profile') != key
    request['robot_preview']['initial_positions']['joint1'] = .2
    assert cache.key(request, 'image', 'profile') != key
    request['robot_preview']['initial_positions']['joint1'] = .1
    source = tmp_path / 'src/pen_toolpath/pen_toolpath/compiler.py'
    source.parent.mkdir(parents=True)
    source.write_text('# new planner version')
    assert cache.key(request, 'image', 'profile') != key


def test_cache_rejects_corruption(tmp_path):
    cache = PreviewCache(tmp_path)
    source, output = tmp_path / 'source', tmp_path / 'output'
    source.mkdir(); output.mkdir()
    for name in REQUIRED:
        (source / name).write_text('{}')
    cache.save('key', source)
    assert cache.restore('key', output)
    (cache.directory / 'key/job.json').write_text('{"corrupt": true}')
    assert not cache.restore('key', output)


class Generator:
    def __init__(self):
        self.calls = 0

    def generate(self, request, folder, cancel_event=None):
        self.calls += 1
        for name in REQUIRED:
            (folder / name).write_text(json.dumps({'samples': [1]}))


def wait_ready(flow, identifier):
    deadline = time.monotonic() + 3
    while flow.preparing and time.monotonic() < deadline:
        time.sleep(.01)
    assert flow.jobs[identifier]['state'] == 'ready', flow.jobs[identifier]
    return flow.jobs[identifier]


def test_verified_preview_reused_with_fresh_review_and_after_restart(tmp_path, monkeypatch):
    generator = Generator()
    def run_worker(command, **kwargs):
        assert command[1:3] == ['-m', 'pen_toolpath.worker']
        from pathlib import Path
        request = json.loads(Path(command[command.index('--request')+1]).read_text())
        generator.generate(request, Path(command[command.index('--out')+1]))
        return SimpleNamespace(returncode=0, poll=lambda: 0)
    monkeypatch.setattr('pen_bringup.service.subprocess.Popen', run_worker)
    flow = Workflow(tmp_path, 'preview')
    (flow.inputs / 'sun.png').write_bytes(b'unchanged image')
    first = wait_ready(flow, flow.prepare('owner', {'image': 'sun.png'})['id'])
    assert generator.calls == 1
    assert first['cache_saved']
    restarted = Workflow(tmp_path, 'preview')
    second = wait_ready(restarted, restarted.prepare('new-owner', {'image': 'sun.png'})['id'])
    assert generator.calls == 1
    assert second['cache_hit']
    assert second['preview_token'] != first['preview_token']
    assert second['sha256'] == first['sha256']
    (flow.inputs / 'sun.png').write_bytes(b'changed image')
    third = wait_ready(restarted, restarted.prepare('new-owner', {'image': 'sun.png'})['id'])
    assert not third['cache_hit']
    assert generator.calls == 2
    assert Workflow(tmp_path, 'fake').preview_cache is None


def test_partial_progress_exposes_fraction_and_eta(tmp_path):
    flow = Workflow(tmp_path)
    folder = tmp_path / 'job'
    folder.mkdir()
    job = {'id': 'job', 'state': 'preparing', 'folder': str(folder)}
    data = {'stage': 'trajectory', 'fraction': .4, 'completed': 40, 'total': 100,
            'eta_seconds': 60, 'elapsed_seconds': 40}
    (folder / 'progress.json').write_text(json.dumps(data))
    result = flow.public_job(job)
    assert result['preparation_stage'] == 'Checking robot trajectory'
    assert result['preparation_progress'] == data


def test_real_cache_reuses_only_matching_bounded_start(tmp_path):
    cache = PreviewCache(tmp_path)
    request = {'execution_mode': 'real', 'execution_backend': 'cartesian',
               'robot_preview': {'initial_positions': {'joint1': .1, 'joint2': .2}}}
    key = cache.key(request, 'input', 'profile')
    source, output = tmp_path/'source', tmp_path/'output'
    source.mkdir(); output.mkdir()
    for name in REQUIRED:
        (source/name).write_text('{"samples":[1]}')
    cache.save(key, source, request)
    close = deepcopy(request)
    close['robot_preview']['initial_positions']['joint1'] += .0001
    assert cache.key(close, 'input', 'profile') == key
    assert cache.restore(key, output, close)
    assert cache.key(close, 'changed', 'profile') != key
    assert cache.key(close, 'input', 'changed') != key
    far = deepcopy(request)
    far['robot_preview']['initial_positions']['joint1'] += .006
    assert not cache.restore(key, output, far)
    assert not cache.restore(key, output)
    missing = deepcopy(request)
    del missing['robot_preview']['initial_positions']['joint2']
    assert not cache.restore(key, output, missing)
    for value in (float('nan'), float('inf'), True):
        invalid = deepcopy(request)
        invalid['robot_preview']['initial_positions']['joint1'] = value
        assert not cache.restore(key, output, invalid)
    (cache.directory/key/'job.json').write_text('{"samples":[2]}')
    assert not cache.restore(key, output, request)


def test_real_preparation_cache_survives_home_encoder_noise(tmp_path):
    class LiveGenerator(Generator):
        positions = {'joint1': .1}

        def snapshot(self):
            return {'joints': self.positions, 'joints_received_at': time.time(),
                    'status': {'execution_backend': 'cartesian'}}

        def generate(self, request, folder, cancel_event=None):
            super().generate(request, folder, cancel_event)
            intermediate = {**request, 'drawing_path': str(folder/'drawing.json')}
            intermediate.pop('image_path')
            (folder/'request.json').write_text(json.dumps(intermediate))

    generator = LiveGenerator()
    profile = tmp_path/'real.yaml'
    profile.write_text(json.dumps({
        'calibrated': True, 'fake_only': False, 'synthetic': False,
        'surface': {'type': 'plane'}, 'qualified': {'cartesian': True, 'rt': True},
        'qualification': {k: True for k in ('rt', 'clearance', 'pen', 'tool')},
    }))

    def flow():
        result = Workflow(tmp_path, 'real', profile, generator)
        result.model_provider = lambda: {
            'available': True, 'robot_sn': 'Rizon4s-063112', 'joints': [],
            'flange_to_pen_tip': {}, 'model_revision': 'unchanged',
        }
        return result

    first_flow = flow()
    (first_flow.inputs/'sun.png').write_bytes(b'unchanged')
    first = wait_ready(first_flow, first_flow.prepare('one', {'image': 'sun.png'})['id'])
    assert first['cache_saved'] and generator.calls == 1
    saved_request = json.loads((tmp_path/'assets/outputs'/first['id']/'request.json').read_text())
    assert saved_request['image_path'].endswith('sun.png')
    assert 'drawing_path' not in saved_request
    generator.positions = {'joint1': .1001}
    second_flow = flow()
    second = wait_ready(second_flow, second_flow.prepare('two', {'image': 'sun.png'})['id'])
    assert second['cache_hit'] and generator.calls == 1
    assert first['preview_token'] != second['preview_token']
    generator.positions = {'joint1': .12}
    third_flow = flow()
    third = wait_ready(third_flow, third_flow.prepare('three', {'image': 'sun.png'})['id'])
    assert not third['cache_hit'] and generator.calls == 2
