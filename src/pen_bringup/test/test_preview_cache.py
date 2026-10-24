import json
import time

from pen_bringup.preview_cache import PreviewCache, REQUIRED
from pen_bringup.service import Workflow


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


def test_verified_preview_reused_with_fresh_review_and_after_restart(tmp_path):
    generator = Generator()
    flow = Workflow(tmp_path, 'preview', bridge=generator)
    (flow.inputs / 'sun.png').write_bytes(b'unchanged image')
    first = wait_ready(flow, flow.prepare('owner', {'image': 'sun.png'})['id'])
    assert generator.calls == 1
    assert first['cache_saved']
    restarted = Workflow(tmp_path, 'preview', bridge=generator)
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
