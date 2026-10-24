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
        if not math.isfinite(size) or not 1 <= size <= 100:
            raise WorkflowError('Drawing size must be between 1 and 100 mm')
        fill=request.get('fill',False)
        if type(fill) is not bool:
            raise WorkflowError('Fill must be true or false')
        preview_context = None
        if self.model_provider:
            model = self.model_provider()
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
                   'size_mm': size, 'fill':fill, 'folder': str(folder)}
            self.jobs[identifier] = job
            self.preparing = True
            self.preparing_job = identifier
            self.preparation_cancel = threading.Event()
            worker = {'image_path': str(image), 'size_mm': size, 'surface': surface,
                      'execution_mode': self.mode, 'fill': fill}
            worker['execution_backend'] = live.get('status', {}).get('execution_backend', 'cartesian') if self.model_provider else 'cartesian'
            if self.profile:
                worker['profile_path'] = str(self.profile)
            if preview_context:
                worker['robot_preview'] = preview_context
            job['profile_sha256'] = digest(self.profile) if self.profile else None
            job['input_sha256'] = input_digest(image)
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
        public = {k: v for k, v in job.items() if k not in ('owner', 'folder')}
        if job.get('state') in ('preparing', 'error', 'stopped') and not job.get('action'):
            folder = Path(job['folder'])
            public['preview_files'] = [name for name in ('picture.png', 'drawing.json', 'toolpath.json')
                                       if (folder / name).is_file()]
            public['preparation_stage'] = ('Checking robot trajectory' if 'toolpath.json' in public['preview_files']
                                            else 'Mapping pen strokes to surface' if 'drawing.json' in public['preview_files']
                                            else 'Extracting image and pen strokes')
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
            if self.active_job:
                raise WorkflowError('Another execution is already active')
            if (job['state'] != 'ready' or self.session(key)['reviewed'] != identifier
                    or not secrets.compare_digest(str(token), job.get('preview_token', ''))):
                raise WorkflowError('A successful Preview of the current inputs is required')
            path = Path(job['folder']) / 'job.json'
            if digest(path) != job['sha256'] or input_digest(self.input_path(job['image'])) != job['input_sha256']:
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

    def home(self, key, identifier=None):
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
            if self.profile is None:
                raise WorkflowError('Home requires a saved calibration profile')
            profile_bytes=self.profile.read_bytes()
            profile=yaml.safe_load(profile_bytes)
            if not isinstance(profile,dict):
                raise WorkflowError('Invalid Home calibration profile')
            profile_digest=hashlib.sha256(profile_bytes).hexdigest()
            data={'kind':'pen_job','schema_version':1,'frame':'world','units':'m',
                  'pose_format':'x,y,z,qx,qy,qz,qw','robot_sn':profile.get('robot_sn'),
                  'fake_only':profile.get('fake_only',True),'calibrated':profile.get('calibrated',False),
                  'profile':profile,'profile_path':str(self.profile),'profile_sha256':profile_digest,
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
            model=self.model_provider() if self.model_provider else None
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
                    'active_job': self.active_job,
                    'jobs': [self.public_job(j) for j in self.jobs.values() if j['owner'] == key][-30:],
                    'robot': self.bridge.snapshot() if self.bridge else {}}
