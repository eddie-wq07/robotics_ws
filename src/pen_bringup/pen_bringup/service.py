"""Bounded artifact workflow, independent of ROS and scientific dependencies."""
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import re
import secrets
import subprocess
import threading
import time


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class WorkflowError(ValueError):
    pass


class Workflow:
    def __init__(self, root, mode='preview', profile=None, bridge=None):
        if mode not in ('preview', 'fake', 'real'):
            raise ValueError('Unknown execution mode')
        self.root = Path(root).resolve()
        self.inputs = self.root / 'assets/inputs'
        self.outputs = self.root / 'assets/outputs'
        self.python = self.root / '.venv-pen/bin/python'
        if profile is None and mode in ('preview', 'fake'):
            default_profile = self.root / 'src/pen_description/config/fake.yaml'
            if default_profile.is_file():
                profile = default_profile
        if mode == 'real' and profile is None:
            raise WorkflowError('Real mode requires an explicit measured calibration profile')
        self.profile = Path(profile).resolve() if profile else None
        self.mode, self.bridge = mode, bridge
        self.model_provider = None
        self.inputs.mkdir(parents=True, exist_ok=True)
        self.outputs.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.jobs, self.sessions = {}, {}
        self.preparing = False
        self.preparing_job = None
        self.preparation_cancel = threading.Event()
        self.prepare_process = None
        self.active_job = None
        self.last_browser = 0.0

    def session(self, key):
        with self.lock:
            if key not in self.sessions:
                if len(self.sessions) >= 256:
                    raise WorkflowError('Too many browser sessions; restart the application')
                self.sessions[key] = {'generation': 0, 'reviewed': None}
            return self.sessions[key]

    def invalidate(self, key):
        with self.lock:
            session = self.session(key)
            session['generation'] += 1
            session['reviewed'] = None

    def input_path(self, name):
        if not isinstance(name, str) or Path(name).name != name:
            raise WorkflowError('Select an input image')
        path = (self.inputs / name).resolve()
        if path.parent != self.inputs or not path.is_file() or path.suffix.lower() not in ('.jpg', '.jpeg', '.png'):
            raise WorkflowError('Input image was not found')
        return path

    def upload(self, key, name, data):
        if not isinstance(name, str) or Path(name).suffix.lower() not in ('.jpg', '.jpeg', '.png'):
            raise WorkflowError('Choose a JPG or PNG image')
        try:
            content = base64.b64decode(data, validate=True)
        except (ValueError, TypeError) as exc:
            raise WorkflowError('Invalid image upload') from exc
        if not 0 < len(content) <= 12 * 1024 * 1024:
            raise WorkflowError('Image must be at most 12 MB')
        if not (content.startswith(b'\x89PNG\r\n\x1a\n') or content.startswith(b'\xff\xd8\xff')):
            raise WorkflowError('File does not contain a JPG or PNG image')
        stem = re.sub(r'[^a-zA-Z0-9_-]', '_', Path(name).stem)[:60] or 'image'
        identifier = f'{stem}-{hashlib.sha256(content).hexdigest()[:12]}{Path(name).suffix.lower()}'
        (self.inputs / identifier).write_bytes(content)
        self.invalidate(key)
        return identifier

    def prepare(self, key, request):
        image = self.input_path(request.get('image'))
        surface = request.get('surface', 'sphere')
        if surface not in ('sphere', 'plane'):
            raise WorkflowError('Choose sphere or plane')
        try:
            size = float(request.get('size_mm', 20))
        except (TypeError, ValueError) as exc:
            raise WorkflowError('Drawing size must be a number') from exc
        if not math.isfinite(size) or not 1 <= size <= 80:
            raise WorkflowError('Drawing size must be between 1 and 80 mm')
        preview_context = None
        if self.model_provider:
            model = self.model_provider()
            if not model.get('available'):
                raise WorkflowError('Robot description unavailable for motion preview')
            snapshot = model.get('visualization', {}).get('joint_snapshot')
            positions = snapshot['joints'] if snapshot else (self.bridge.snapshot().get('joints', {}) if self.bridge else {})
            preview_context = {
                'joints': model['joints'], 'flange_link': model['robot_sn']+'_flange',
                'initial_positions': positions, 'flange_to_visual_tip': model['flange_to_pen_tip'],
                'initial_source': snapshot or {'source': 'ROS joint states'},
                'model_revision': model['model_revision'],
            }
        with self.lock:
            if self.preparing or self.active_job:
                raise WorkflowError('Wait for the current job or stop it first')
            self.invalidate(key)
            generation = self.session(key)['generation']
            identifier = secrets.token_hex(8)
            folder = self.outputs / identifier
            folder.mkdir()
            job = {'id': identifier, 'owner': key, 'generation': generation,
                   'state': 'preparing', 'message': 'Extracting image and generating surface path',
                   'progress': 0.0, 'image': image.name, 'surface': surface,
                   'size_mm': size, 'folder': str(folder)}
            self.jobs[identifier] = job
            self.preparing = True
            self.preparing_job = identifier
            self.preparation_cancel = threading.Event()
            worker = {'image_path': str(image), 'size_mm': size, 'surface': surface,
                      'execution_mode': self.mode, 'fill': False}
            if self.profile:
                worker['profile_path'] = str(self.profile)
            if preview_context:
                worker['robot_preview'] = preview_context
            job['profile_sha256'] = digest(self.profile) if self.profile else None
            job['input_sha256'] = digest(image)
            (folder / 'request.json').write_text(json.dumps(worker, indent=2))
            threading.Thread(target=self._compile, args=(job,), daemon=True).start()
            return self.public_job(job)

    def _compile(self, job):
        folder = Path(job['folder'])
        try:
            if self.preparation_cancel.is_set():
                raise WorkflowError('Preparation canceled')
            env = os.environ.copy()
            env.pop('PYTHONPATH', None)
            env.pop('PYTHONHOME', None)
            if self.bridge is not None and hasattr(self.bridge, 'generate'):
                self.bridge.generate(json.loads((folder / 'request.json').read_text()), folder,
                                     cancel_event=self.preparation_cancel)
            else:
                with (folder / 'worker.log').open('w') as log:
                    process = subprocess.Popen([str(self.python), '-m', 'pen_toolpath.worker',
                                                '--request', str(folder / 'request.json'), '--out', str(folder)],
                                               stdout=log, stderr=subprocess.STDOUT, text=True, env=env)
                    with self.lock:
                        self.prepare_process = process
                    deadline = time.monotonic() + 180
                    while process.poll() is None:
                        if self.preparation_cancel.is_set() or time.monotonic() > deadline:
                            process.terminate()
                            try:
                                process.wait(timeout=2)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait()
                            raise WorkflowError('Preparation canceled' if self.preparation_cancel.is_set()
                                                else 'Path generation timed out')
                        time.sleep(0.02)
                    if process.returncode:
                        raise WorkflowError((folder / 'worker.log').read_text()[-1800:] or 'Path generation failed')
            path = folder / 'job.json'
            parsed = json.loads(path.read_text())
            if not parsed.get('samples'):
                raise WorkflowError('No executable path was generated')
            with self.lock:
                if self.preparation_cancel.is_set():
                    job.update(state='stopped', message='Preparation canceled')
                    return
                if self.session(job['owner'])['generation'] != job['generation']:
                    job.update(state='stale', message='Inputs changed. Preview again.')
                    return
                token = secrets.token_urlsafe(24)
                job.update(state='ready', message='Preview ready', progress=1.0,
                           sha256=digest(path), preview_token=token)
                self.session(job['owner'])['reviewed'] = job['id']
        except Exception as exc:
            with self.lock:
                job.update(state='stopped' if self.preparation_cancel.is_set() else 'error', message=str(exc))
        finally:
            with self.lock:
                self.preparing = False
                self.preparing_job = None
                self.prepare_process = None

    def public_job(self, job):
        return {k: v for k, v in job.items() if k not in ('owner', 'folder')}

    def get_job(self, key, identifier):
        with self.lock:
            job = self.jobs.get(identifier)
            if job is None or job['owner'] != key:
                raise WorkflowError('Job was not found in this browser session')
            return job

    def execute(self, key, identifier, token, air_run=False):
        with self.lock:
            job = self.get_job(key, identifier)
            if self.mode == 'preview' or self.bridge is None:
                raise WorkflowError('Application is in preview mode. Launch fake or commissioned real mode to execute.')
            if self.active_job:
                raise WorkflowError('Another execution is already active')
            if (job['state'] != 'ready' or self.session(key)['reviewed'] != identifier
                    or not secrets.compare_digest(str(token), job.get('preview_token', ''))):
                raise WorkflowError('A successful Preview of the current inputs is required')
            path = Path(job['folder']) / 'job.json'
            if digest(path) != job['sha256'] or digest(self.input_path(job['image'])) != job['input_sha256']:
                self.invalidate(key)
                raise WorkflowError('Input or job changed. Preview again.')
            if self.profile and digest(self.profile) != job['profile_sha256']:
                self.invalidate(key)
                raise WorkflowError('Calibration changed. Preview again.')
            if not self.bridge.ready():
                raise WorkflowError('ROS execution server is unavailable')
            self.active_job = identifier
            self.last_browser = time.monotonic()
            job.update(state='executing', message='Checking controller and hardware state', progress=0.0)
            self.session(key)['reviewed'] = None
            try:
                self.bridge.execute(str(path), job['sha256'], bool(air_run),
                                    lambda ok, msg: self.finish(identifier, ok, msg),
                                    lambda phase, progress: self.progress(identifier, phase, progress))
            except Exception as exc:
                self.finish(identifier, False, str(exc))
                raise WorkflowError(str(exc)) from exc
            return self.public_job(job)

    def progress(self, identifier, phase, progress):
        with self.lock:
            # Action feedback and its result travel independently. Late feedback
            # must not replace the terminal result displayed to the operator.
            job = self.jobs[identifier]
            if self.active_job == identifier and job['state'] == 'executing':
                job.update(message=phase, progress=float(progress))

    def finish(self, identifier, success, message):
        with self.lock:
            job = self.jobs[identifier]
            job.update(state='complete' if success else 'stopped', message=message,
                       progress=1.0 if success else job['progress'])
            (Path(job['folder']) / 'execution.json').write_text(json.dumps(self.public_job(job), indent=2))
            self.active_job = None

    def stop(self, key):
        with self.lock:
            identifier = self.active_job or self.preparing_job
            if identifier and self.jobs[identifier]['owner'] != key:
                raise WorkflowError('Job belongs to another browser session')
            if self.preparing_job:
                self.preparation_cancel.set()
                self.jobs[self.preparing_job].update(state='stopping', message='Canceling preparation')
                if self.bridge and hasattr(self.bridge, 'cancel_preparation'):
                    self.bridge.cancel_preparation()
                return {'message': 'Preparation cancellation requested'}
            if self.active_job and self.bridge:
                self.bridge.stop()
                return {'message': 'Stop requested; waiting for executor state'}
            return {'message': 'No active job'}

    def state(self, key):
        with self.lock:
            self.session(key)
            if self.active_job and self.jobs[self.active_job]['owner'] == key:
                self.last_browser = time.monotonic()
            return {'mode': self.mode, 'inputs': sorted(p.name for p in self.inputs.iterdir()
                    if p.is_file() and p.suffix.lower() in ('.png', '.jpg', '.jpeg')),
                    'active_job': self.active_job,
                    'jobs': [self.public_job(j) for j in self.jobs.values() if j['owner'] == key][-30:],
                    'robot': self.bridge.snapshot() if self.bridge else {}}
