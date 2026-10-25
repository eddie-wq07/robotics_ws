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
import yaml
from .preview_cache import PreviewCache
from .prepared_job import restore_prepared
from .profiles import ProfileCatalog


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def input_digest(path):
    path = Path(path)
    annotation = path.with_suffix(path.suffix + '.strokes.json')
    content = path.read_bytes()
    if annotation.is_file():
        if annotation.stat().st_size > 1_000_000:
            raise WorkflowError('Annotation file exceeds 1 MB')
        content += b'\0strokes\0' + annotation.read_bytes()
    return hashlib.sha256(content).hexdigest()


class WorkflowError(ValueError):
    pass


class Workflow:
    def __init__(self, root, mode='preview', profile=None, bridge=None, prepared_job=None):
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
        self.profiles = ProfileCatalog(self.root, self.profile)
        self.mode, self.bridge = mode, bridge
        self.prepared_job = Path(prepared_job) if prepared_job else None
        self.preview_cache = PreviewCache(self.root) if mode in ('preview', 'real') else None
        self.default_selection = {}
        if self.profile and self.profile.is_file():
            config = yaml.safe_load(self.profile.read_text())
            if isinstance(config, dict) and isinstance(config.get('selection'), dict):
                self.default_selection = config['selection']
        self.model_provider = None
        self.surface_profiles = {}
        self.model_for_surface = None
        self.model_for_profile = None
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
        self.clearance_check_path = None
        self.clearance_check_digest = None
        self.clearance_check_used = False

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

    def profile_for_surface(self, surface=None):
        if surface is not None and surface not in ('sphere', 'plane', 'cylinder', 'photo_board'):
            raise WorkflowError('Choose sphere, plane, Tylenol cylinder or photo board')
        if surface in self.surface_profiles:
            return self.surface_profiles[surface]
        if self.profiles.records():
            return self.profiles.resolve({'surface': surface})['path']
        return self.profile

    def resolve_selection(self, request, allow_preview=False):
        try:
            record = self.profiles.resolve(request)
        except ValueError as exc:
            raise WorkflowError(str(exc)) from exc
        reason = self.profiles.unavailable(record, self.mode)
        if reason and not allow_preview:
            raise WorkflowError(reason)
        return {**record['selection'], 'profile': record['id'], 'surface': record['surface'],
                'size_mm': request.get('size_mm', record['selection'].get('size_mm', 20)),
                'fill': request.get('fill', record['selection'].get('fill', False)),
                'image': request.get('image', record['selection'].get('image')),
                'profile_sha256': record['sha256']}

    def planning_mode(self, profile):
        if self.mode == 'real':
            record = next((r for r in self.profiles.records().values()
                           if r['path'] == Path(profile).resolve()), None) if profile else None
            if record and self.profiles.execution_blockers(record, self.mode):
                return 'preview'
        return self.mode

    def profile_for_job(self, job):
        if job.get('profile'):
            try:
                return self.profiles.resolve({'profile': job['profile']})['path']
            except ValueError as exc:
                raise WorkflowError(str(exc)) from exc
        return self.profile_for_surface(job.get('surface'))

    def update_profile_flags(self, key, request):
        with self.lock:
            if self.active_job or self.preparing:
                raise WorkflowError('Wait for the current operation or stop it before editing profile flags')
            try:
                record = self.profiles.update_flags(request.get('profile'), request.get('flags'),
                                                   request.get('profile_sha256'))
            except ValueError as exc:
                raise WorkflowError(str(exc)) from exc
            for session_key in list(self.sessions):
                self.invalidate(session_key)
            for job in self.jobs.values():
                if job.get('profile') == record['id'] and job['state'] == 'ready':
                    job.update(state='stale', message='Profile flags changed; prepare again')
            return {'profile': record['id'], 'sha256': record['sha256'],
                    'calibrated': record['data']['calibrated'], 'fake_only': record['data']['fake_only']}

    def load_clearance_check(self, key):
        with self.lock:
            if self.mode != 'real' or not self.bridge or not self.clearance_check_path or self.clearance_check_used:
                raise WorkflowError('No unused clearance check is armed')
            if self.active_job or self.preparing:
                raise WorkflowError('Wait for the current operation or stop it before loading the air check')
            path = self.clearance_check_path
            if digest(path) != self.clearance_check_digest:
                raise WorkflowError('The approved clearance check changed')
            data = json.loads(path.read_text())
            if (data.get('purpose') != 'air_clearance_check' or not data.get('air_run') or
                    any(s.get('extrude') is not False for s in data['samples'])):
                raise WorkflowError('Invalid extrusion-off clearance check')
            profile_path = Path(data['profile_path']).resolve()
            record = next((r for r in self.profiles.records().values() if r['path'] == profile_path), None)
            if not record or digest(profile_path) != data['profile_source_sha256']:
                raise WorkflowError('Clearance check profile changed; prepare and review it again')
            self.invalidate(key)
            identifier = secrets.token_hex(8)
            token = secrets.token_urlsafe(24)
            job = {'id': identifier, 'owner': key, 'generation': self.session(key)['generation'],
                   'state': 'ready', 'message': 'Air clearance check ready — extrusion OFF',
                   'progress': 1., 'image': record['selection']['image'], 'surface': record['surface'],
                   'size_mm': 5., 'fill': False, 'folder': str(path.parent), 'profile': record['id'],
                   'profile_sha256': record['sha256'], 'input_sha256': input_digest(self.input_path(record['selection']['image'])),
                   'sha256': self.clearance_check_digest, 'preview_token': token, 'clearance_check': True}
            self.jobs[identifier] = job
            self.session(key)['reviewed'] = identifier
            return self.public_job(job)

    def prepare(self, key, request):
        record = None
        if request.get('profile') or self.profiles.records():
            resolved = self.resolve_selection(request, allow_preview=True)
            if request.get('profile') and request.get('surface', resolved['surface']) != resolved['surface']:
                raise WorkflowError('Selected surface does not match the saved profile')
            request = {**request, **resolved}
            record = self.profiles.records()[resolved['profile']]
        image = self.input_path(request.get('image'))
        surface = request.get('surface', 'sphere')
        if surface not in ('sphere', 'plane', 'cylinder', 'photo_board'):
            raise WorkflowError('Choose sphere, plane, Tylenol cylinder or photo board')
        surface_spec = record['data']['surface'] if record else surface
        selected_profile = record['path'] if record else self.profile_for_surface(surface)
        planning_mode = self.planning_mode(selected_profile)
        if surface == 'photo_board':
            profile_data = yaml.safe_load(selected_profile.read_text()) if selected_profile else {}
            surface_spec = profile_data.get('surface', {})
            if surface_spec.get('type') != 'plane':
                raise WorkflowError('Photo board requires a planar profile')
        if surface == 'cylinder':
            profile_data = yaml.safe_load(selected_profile.read_text()) if selected_profile else {}
            surface_spec = profile_data.get('surface', {})
            if surface_spec.get('type') != 'cylinder':
                raise WorkflowError('Start the Tylenol preview with config/tylenol_sun.yaml')
            if planning_mode == 'real' and (profile_data.get('calibrated') is not True or
                                        profile_data.get('fake_only', True)):
                raise WorkflowError('Tylenol requires an approved real calibration profile')
        try:
            size = float(request.get('size_mm', 20))
        except (TypeError, ValueError) as exc:
            raise WorkflowError('Drawing size must be a number') from exc
        maximum_size = 200 if surface == 'photo_board' or (record and record['data'].get('workpiece', {}).get('plane_mesh')) else 100
        if not math.isfinite(size) or not 1 <= size <= maximum_size:
            raise WorkflowError(f'Drawing size must be between 1 and {maximum_size} mm')
        fill=request.get('fill',False)
        if type(fill) is not bool:
            raise WorkflowError('Fill must be true or false')
        preview_context = None
        if self.model_provider:
            model = (self.model_for_profile(selected_profile) if self.model_for_profile else
                     self.model_for_surface(surface) if self.model_for_surface else self.model_provider())
            if not model.get('available'):
                raise WorkflowError('Robot description unavailable for motion preview')
            snapshot = model.get('visualization', {}).get('joint_snapshot')
            live = self.bridge.snapshot() if self.bridge else {}
            live_joints = live.get('joints', {})
            live_fresh = bool(live_joints) and time.time()-live.get('joints_received_at', 0)<1
            positions = live_joints if live_fresh else (snapshot['joints'] if snapshot else {})
            preview_context = {
                'joints': model['joints'], 'flange_link': model['robot_sn']+'_flange',
                'initial_positions': positions, 'flange_to_visual_tip': model['flange_to_pen_tip'],
                'initial_source': {'source': 'ROS joint states'} if live_fresh else snapshot,
                'model_revision': model['model_revision'],
                'allow_simulation_joint_reset': self.mode == 'fake' and
                    live.get('status', {}).get('execution_backend') == 'joint_simulation',
            }
        with self.lock:
            if self.preparing_job and not self.active_job:
                pending = self.jobs[self.preparing_job]
                # Reloads invalidate the browser review, not identical work
                # already in progress. Reattach only the same owner and inputs.
                if (pending['owner'] == key and pending['state'] == 'preparing' and
                        pending['image'] == image.name and pending['surface'] == surface and
                        pending['size_mm'] == size and pending['fill'] == fill and
                        pending.get('profile') == request.get('profile') and
                        pending['input_sha256'] == input_digest(image) and
                        pending['profile_sha256'] == (digest(selected_profile) if selected_profile else None)):
                    pending['generation'] = self.session(key)['generation']
                    return self.public_job(pending)
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
                   'size_mm': size, 'fill':fill, 'folder': str(folder), 'planning_mode': planning_mode}
            if record:
                job['profile'] = record['id']
            self.jobs[identifier] = job
            self.preparing = True
            self.preparing_job = identifier
            self.preparation_cancel = threading.Event()
            worker = {'image_path': str(image), 'size_mm': size, 'surface': surface_spec,
                      'execution_mode': planning_mode, 'fill': fill}
            worker['execution_backend'] = live.get('status', {}).get('execution_backend', 'cartesian') if self.model_provider else 'cartesian'
            if selected_profile:
                worker['profile_path'] = str(selected_profile)
            if preview_context:
                worker['robot_preview'] = preview_context
            job['profile_sha256'] = digest(selected_profile) if selected_profile else None
            job['input_sha256'] = input_digest(image)
            if self.preview_cache:
                job['cache_key'] = self.preview_cache.key(worker, job['input_sha256'], job['profile_sha256'])
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
            request = json.loads((folder / 'request.json').read_text())
            saved = bool(self.prepared_job and restore_prepared(
                self.root, self.prepared_job, request, job['input_sha256'], job['profile_sha256'], folder))
            cached = saved or bool(self.preview_cache and self.preview_cache.restore(
                job['cache_key'], folder, request))
            job['cache_hit'] = cached
            if cached:
                job['message'] = 'Loading saved trajectory'
            elif (request.get('execution_mode') != 'preview' and
                  self.bridge is not None and hasattr(self.bridge, 'generate')):
                self.bridge.generate(json.loads((folder / 'request.json').read_text()), folder,
                                     cancel_event=self.preparation_cancel)
            else:
                with (folder / 'worker.log').open('w') as log:
                    process = subprocess.Popen([str(self.python), '-m', 'pen_toolpath.worker',
                                                '--request', str(folder / 'request.json'), '--out', str(folder)],
                                               stdout=log, stderr=subprocess.STDOUT, text=True, env=env)
                    with self.lock:
                        self.prepare_process = process
                    deadline = time.monotonic() + 600
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
            # Multi-stage ROS generation writes its intermediate drawing_path
            # request into this folder. Keep the original browser request for
            # cache identity; intermediate paths are unique to each job folder.
            (folder / 'request.json').write_text(json.dumps(request, indent=2))
            if (input_digest(self.input_path(job['image'])) != job['input_sha256'] or
                    (request.get('profile_path') and digest(request['profile_path']) != job['profile_sha256'])):
                job.update(state='stale', message='Inputs changed during preparation. Preview again.')
                return
            if self.preview_cache and not cached and not self.preparation_cancel.is_set():
                current_key = self.preview_cache.key(request, job['input_sha256'], job['profile_sha256'])
                if current_key == job['cache_key']:
                    try:
                        self.preview_cache.save(job['cache_key'], folder, request)
                        job['cache_saved'] = True
                    except OSError as error:
                        job['cache_warning'] = f'Preview ready but could not save cache: {error}'
            with self.lock:
                if self.preparation_cancel.is_set():
                    job.update(state='stopped', message='Preparation canceled')
                    return
                if self.session(job['owner'])['generation'] != job['generation']:
                    job.update(state='stale', message='Inputs changed. Preview again.')
                    return
                token = secrets.token_urlsafe(24)
                job.update(state='ready', message='Loaded saved trajectory' if cached else 'Preview ready', progress=1.0,
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
        public = {k: v for k, v in job.items() if k not in ('owner', 'folder')}
        if job.get('state') in ('preparing', 'error', 'stopped') and not job.get('action'):
            folder = Path(job['folder'])
            public['preview_files'] = [name for name in ('picture.png', 'drawing.json', 'toolpath.json')
                                       if (folder / name).is_file()]
            public['preparation_stage'] = ('Checking robot trajectory' if 'toolpath.json' in public['preview_files']
                                            else 'Mapping pen strokes to surface' if 'drawing.json' in public['preview_files']
                                            else 'Extracting image and pen strokes')
            if job.get('cache_hit'):
                public['preparation_stage'] = 'Loading saved trajectory'
            try:
                progress = json.loads((folder / 'progress.json').read_text())
                labels = {'image': 'Extracting image', 'surface': 'Mapping pen strokes to surface',
                          'approach': 'Checking approach motion', 'trajectory': 'Checking robot trajectory',
                          'return': 'Checking return motion', 'saving': 'Saving checked trajectory',
                          'complete': 'Loading checked preview'}
                if progress['stage'] in labels:
                    public['preparation_stage'] = labels[progress['stage']]
                    public['preparation_progress'] = progress
            except (OSError, ValueError, KeyError, TypeError):
                pass
        return public

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
            if job.get('clearance_check'):
                if self.clearance_check_used or job['sha256'] != self.clearance_check_digest:
                    raise WorkflowError('This clearance-check permit is used or changed')
            elif self.mode == 'real' and job.get('profile'):
                record = self.profiles.resolve({'profile': job['profile']})
                blockers = self.profiles.execution_blockers(record, self.mode)
                if blockers:
                    raise WorkflowError(' '.join(blockers))
                if job.get('planning_mode') == 'preview':
                    raise WorkflowError('This trajectory was prepared for preview; prepare it again for real execution.')
            if self.active_job:
                raise WorkflowError('Another execution is already active')
            if (job['state'] != 'ready' or self.session(key)['reviewed'] != identifier
                    or not secrets.compare_digest(str(token), job.get('preview_token', ''))):
                raise WorkflowError('A successful Preview of the current inputs is required')
            path = Path(job['folder']) / 'job.json'
            if digest(path) != job['sha256'] or input_digest(self.input_path(job['image'])) != job['input_sha256']:
                self.invalidate(key)
                raise WorkflowError('Input or job changed. Preview again.')
            selected_profile = self.profile_for_job(job)
            if selected_profile and digest(selected_profile) != job['profile_sha256']:
                self.invalidate(key)
                raise WorkflowError('Calibration changed. Preview again.')
            if not self.bridge.ready():
                raise WorkflowError('ROS execution server is unavailable')
            self.active_job = identifier
            self.last_browser = time.monotonic()
            job.update(state='executing', message='Checking controller and hardware state', progress=0.0)
            self.session(key)['reviewed'] = None
            try:
                if job.get('clearance_check'):
                    self.clearance_check_used = True
                self.bridge.execute(str(path), job['sha256'], bool(air_run or job.get('clearance_check')),
                                    lambda ok, msg: self.finish(identifier, ok, msg),
                                    lambda phase, progress, sample_index=None: self.progress(identifier, phase, progress, sample_index))
            except Exception as exc:
                self.finish(identifier, False, str(exc))
                raise WorkflowError(str(exc)) from exc
            return self.public_job(job)

    def warmup(self, key):
        with self.lock:
            if self.mode == 'preview' or self.bridge is None:
                raise WorkflowError('Warmup requires a connected fake or real executor')
            if self.active_job or self.preparing:
                raise WorkflowError('Wait for the current operation or stop it before warming the pen')
            identifier = secrets.token_hex(8)
            folder = self.outputs / identifier
            folder.mkdir()
            job = {'id': identifier, 'owner': key, 'folder': str(folder), 'action': 'warmup',
                   'state': 'executing', 'progress': 0.,
                   'message': 'Starting pen warmup; arm remains stationary'}
            self.jobs[identifier] = job
            self.active_job = identifier
            self.last_browser = time.monotonic()
            try:
                self.bridge.warmup(lambda ok, msg: self.finish(identifier, ok, msg),
                                   lambda phase, progress: self.progress(identifier, phase, progress))
            except Exception as exc:
                self.finish(identifier, False, str(exc))
                raise WorkflowError(str(exc)) from exc
            return self.public_job(job)

    def home(self, key, identifier=None, surface=None, profile=None):
        # Home uses the saved calibration, never a successfully compiled drawing.
        with self.lock:
            if self.mode != 'real' or self.bridge is None:
                raise WorkflowError('Go home requires the connected real robot')
            if self.active_job:
                raise WorkflowError('Stop the current operation before going home')
            if self.preparing:
                if self.preparing_job and self.jobs[self.preparing_job]['owner'] != key:
                    raise WorkflowError('Another browser is preparing a drawing; wait for it to finish')
                self.preparation_cancel.set()
                if hasattr(self.bridge, 'cancel_preparation'):
                    self.bridge.cancel_preparation()
        deadline=time.monotonic()+5
        while self.preparing and time.monotonic()<deadline:
            time.sleep(.02)
        with self.lock:
            if self.mode != 'real' or self.bridge is None:
                raise WorkflowError('Go home requires the connected real robot')
            if self.active_job or self.preparing:
                raise WorkflowError('Stop the current operation before going home')
            if profile:
                self.resolve_selection({'profile': profile})
            selected_profile = (self.profile_for_job({'profile': profile}) if profile else
                                self.profile_for_surface(surface))
            if selected_profile is None:
                raise WorkflowError('Home requires a saved calibration profile')
            profile_bytes=selected_profile.read_bytes()
            profile=yaml.safe_load(profile_bytes)
            if not isinstance(profile,dict):
                raise WorkflowError('Invalid Home calibration profile')
            profile_digest=hashlib.sha256(profile_bytes).hexdigest()
            data={'kind':'pen_job','schema_version':1,'frame':'world','units':'m',
                  'pose_format':'x,y,z,qx,qy,qz,qw','robot_sn':profile.get('robot_sn'),
                  'fake_only':profile.get('fake_only',True),'calibrated':profile.get('calibrated',False),
                  'profile':profile,'profile_path':str(selected_profile),'profile_sha256':profile_digest,
                  'profile_source_sha256':profile_digest,'qualification':profile.get('qualification',{}),
                  'pen':profile.get('pen',{})}
            if not self.bridge.ready():
                raise WorkflowError('Robot executor is unavailable')
            state=self.bridge.snapshot()
            if time.time()-state.get('joints_received_at',0)>1 or time.time()-state.get('pose_received_at',0)>1:
                raise WorkflowError('Fresh measured pose and joints are required')
            home=data.get('profile',{}).get('home',{})
            names=home.get('joint_names',[]);target=home.get('joint_positions_rad',[])
            if home.get('controller')!='rizon_arm_controller' or len(names)!=7 or len(target)!=7:
                raise WorkflowError('No recorded joint-controller Home is configured')
            joints=state.get('joints',{})
            if not all(name in joints for name in names):raise WorkflowError('Home joint names do not match robot feedback')
            if all(abs(joints[name]-target[i])<.005 for i,name in enumerate(names)):
                return {'state':'complete','message':'Robot is already at the recorded joint Home'}
            model_surface=surface or profile.get('surface',{}).get('type')
            model=(self.model_for_profile(selected_profile) if self.model_for_profile else
                   self.model_for_surface(model_surface) if self.model_for_surface and model_surface
                   else self.model_provider() if self.model_provider else None)
            if not model or not model.get('available'):raise WorkflowError('Robot geometry is required to check the joint Home path')
            home_id=secrets.token_hex(8);folder=self.outputs/home_id;folder.mkdir()
            home_path=folder/'job.json';request=folder/'home_request.json'
            request.write_text(json.dumps({'source':data,'state':state,'model':model}))
            env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
            try:
                result=subprocess.run([str(self.python),'-m','pen_toolpath.joint_home','--request',str(request),
                    '--out',str(home_path)],capture_output=True,text=True,env=env,timeout=15)
            except subprocess.TimeoutExpired as exc:
                raise WorkflowError('Home path checking timed out; no motion requested') from exc
            if result.returncode:
                lines=result.stderr.strip().splitlines()
                raise WorkflowError(lines[-1].removeprefix('ValueError: ') if lines else 'Home path check failed')
            job={'id':home_id,'owner':key,'folder':str(folder),'state':'executing',
                 'action':'home','message':'Returning to recorded joint Home at 3× speed','progress':0.,'sha256':digest(home_path)}
            self.jobs[home_id]=job;self.active_job=home_id;self.last_browser=time.monotonic()
            try:
                self.bridge.execute(str(home_path),job['sha256'],False,
                    lambda ok,msg:self.finish(home_id,ok,msg),
                    lambda phase,progress,sample_index=None:self.progress(home_id,phase,progress,sample_index))
            except Exception as exc:
                self.finish(home_id,False,str(exc))
                raise WorkflowError(str(exc)) from exc
            return self.public_job(job)

    def progress(self, identifier, phase, progress, sample_index=None):
        with self.lock:
            # Action feedback and its result travel independently. Late feedback
            # must not replace the terminal result displayed to the operator.
            job = self.jobs[identifier]
            if self.active_job == identifier and job['state'] == 'executing':
                job.update(message=phase, progress=float(progress))
                if sample_index is not None:
                    job['sample_index'] = int(sample_index)

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
                    'default_selection': {**self.default_selection, **({'profile': self.profile.name} if self.profile else {})},
                    'profiles': self.profiles.public(self.mode),
                    'executor_ready': bool(self.bridge and self.bridge.ready()),
                    'clearance_check_available': bool(self.clearance_check_path and not self.clearance_check_used),
                    'active_job': self.active_job,
                    'jobs': [self.public_job(j) for j in self.jobs.values() if j['owner'] == key][-30:],
                    'robot': self.bridge.snapshot() if self.bridge else {}}
