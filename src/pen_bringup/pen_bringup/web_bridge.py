"""Local browser bridge. Motion is delegated to the sole ROS action executor."""
import argparse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
import signal
import threading
import time
from urllib.parse import unquote, urlparse, parse_qs
from urllib.request import urlopen
from urllib.error import URLError, HTTPError

from .service import Workflow, WorkflowError


class RosBridge:
    def __init__(self, node_name='pen_web_bridge'):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from pen_interfaces.action import ExecuteDrawing, PrepareDrawing, GeneratePath
        from std_msgs.msg import Empty
        from std_srvs.srv import Trigger
        from diagnostic_msgs.msg import DiagnosticArray
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import JointState
        self.node = Node(node_name)
        self.action_type = ExecuteDrawing
        self.empty_type = Empty
        self.stop_type = Trigger
        self.client = ActionClient(self.node, ExecuteDrawing, '/pen/execute')
        self.prepare_type, self.generate_type = PrepareDrawing, GeneratePath
        self.prepare_client = ActionClient(self.node, PrepareDrawing, '/pen/prepare_drawing')
        self.generate_client = ActionClient(self.node, GeneratePath, '/pen/generate_path')
        self.stop_client = self.node.create_client(Trigger, '/pen/stop')
        self.warmup_client = self.node.create_client(Trigger, '/pen/warmup')
        self.warmup_request = None
        self.heartbeat = self.node.create_publisher(Empty, '/pen/heartbeat', 10)
        self.goal = None
        self.preparation_goal = None
        self.preparation_cancel = threading.Event()
        self.execution_cancel = threading.Event()
        self.execution_requested = False
        self.workflow = None
        self.state = {'joints': {}, 'pose': None, 'status': {}, 'received_at': None}
        self.lock = threading.Lock()
        self.node.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.node.create_subscription(PoseStamped, '/pen/execution_pose', self.pose, qos_profile_sensor_data)
        self.node.create_subscription(DiagnosticArray, '/pen/execution_status', self.status, 10)
        self.node.create_timer(0.2, self.tick)
        def spin():
            from rclpy.executors import ExternalShutdownException
            try:
                rclpy.spin(self.node)
            except ExternalShutdownException:
                pass
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()

    def joints(self, msg):
        with self.lock:
            self.state['joints'] = dict(zip(msg.name, msg.position))
            self.state['joints_received_at'] = time.time()

    def pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self.lock:
            self.state['pose'] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
            self.state['pose_received_at'] = time.time()

    def status(self, msg):
        with self.lock:
            for status in msg.status:
                self.state['status'] = {'phase': status.message, **{p.key: p.value for p in status.values}}
            self.state['received_at'] = time.time()
        self.update_warmup()

    def update_warmup(self):
        request = self.warmup_request
        if not request or request['id'] is None:
            return
        status = self.snapshot()['status']
        if status.get('warmup_id') != request['id']:
            return
        phase = status.get('phase', '')
        if phase in ('WARMUP_COMPLETE', 'WARMUP_STOPPED', 'WARMUP_FAILED'):
            self.warmup_request = None
            self.execution_requested = False
            message = ('Warmup complete — check pen temperature. Click Execute when ready.'
                       if phase == 'WARMUP_COMPLETE' else status.get('error') or 'Warmup stopped; servo released')
            request['done'](phase == 'WARMUP_COMPLETE', message)
        elif phase.startswith('WARMUP_'):
            request['feedback'](phase, float(status.get('progress', 0)))

    def warmup(self, done, feedback):
        if not self.warmup_client.service_is_ready():
            raise WorkflowError('Warmup service unavailable; restart the updated executor')
        self.execution_cancel.clear()
        self.execution_requested = True
        request = {'id': None, 'done': done, 'feedback': feedback, 'started': time.monotonic()}
        self.warmup_request = request
        self.heartbeat.publish(self.empty_type())

        def accepted(future):
            try:
                result = future.result()
                if self.warmup_request is not request:
                    return
                if not result.success:
                    raise WorkflowError(result.message)
                request['id'] = result.message
                if self.execution_cancel.is_set():
                    self.stop_client.call_async(self.stop_type.Request())
                self.update_warmup()
            except Exception as exc:
                self.warmup_request = None
                self.execution_requested = False
                done(False, str(exc))
        self.warmup_client.call_async(self.stop_type.Request()).add_done_callback(accepted)

    def snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.state))

    def tick(self):
        flow = self.workflow
        if flow and flow.active_job and time.monotonic() - flow.last_browser < 1.5:
            self.heartbeat.publish(self.empty_type())
        request = self.warmup_request
        if request:
            elapsed = time.monotonic() - request['started']
            stale = (request['id'] is not None and elapsed>2 and
                     time.time()-(self.snapshot().get('received_at') or 0)>2)
            if elapsed>40 or (request['id'] is None and elapsed>5) or stale:
                self.warmup_request = None
                if self.stop_client.service_is_ready():
                    self.stop_client.call_async(self.stop_type.Request())
                self.execution_requested = False
                request['done'](False, 'Warmup feedback timed out; stop requested, pen state unconfirmed')

    def ready(self):
        return self.client.server_is_ready()

    def cancel_preparation(self):
        self.preparation_cancel.set()
        if self.preparation_goal and self.preparation_goal.accepted:
            self.preparation_goal.cancel_goal_async()

    def generate(self, request, folder, cancel_event=None):
        self.preparation_cancel = cancel_event if cancel_event is not None else threading.Event()

        def wait(future, timeout, handle=None):
            deadline = time.monotonic() + timeout
            while not future.done():
                if self.preparation_cancel.is_set():
                    if handle is not None:
                        handle.cancel_goal_async()
                    raise WorkflowError('Preparation canceled')
                if time.monotonic() > deadline:
                    if handle is not None:
                        handle.cancel_goal_async()
                    raise WorkflowError('Preparation request timed out')
                time.sleep(0.02)
            return future.result()

        def run(client, kind, data):
            if self.preparation_cancel.is_set():
                raise WorkflowError('Preparation canceled')
            if not client.wait_for_server(timeout_sec=8.0):
                raise WorkflowError('Image/path action server is unavailable')
            goal = kind.Goal()
            goal.request_json, goal.output_directory = json.dumps(data), str(folder)
            future = client.send_goal_async(goal)
            # A cancel can precede goal acceptance; cancel the late accepted goal.
            cancellation = self.preparation_cancel
            def accepted(f):
                handle = f.result()
                if cancellation.is_set() and handle and handle.accepted:
                    handle.cancel_goal_async()
            future.add_done_callback(accepted)
            handle = wait(future, 10)
            if not handle.accepted:
                raise WorkflowError('Preparation server is busy')
            self.preparation_goal = handle
            try:
                result = wait(handle.get_result_async(), 600, handle).result
                if self.preparation_cancel.is_set():
                    raise WorkflowError('Preparation canceled')
                if not result.success:
                    raise WorkflowError(result.message)
                return result.artifact_path
            finally:
                self.preparation_goal = None
        drawing = run(self.prepare_client, self.prepare_type, request)
        path_request = {**request, 'drawing_path': drawing}
        path_request.pop('image_path', None)
        run(self.generate_client, self.generate_type, path_request)

    def execute(self, path, sha, air_run, done, feedback):
        self.execution_cancel.clear()
        self.execution_requested = True
        # Begin the liveness handshake immediately; the periodic timer maintains it.
        self.heartbeat.publish(self.empty_type())
        goal = self.action_type.Goal()
        goal.job_path, goal.expected_sha256, goal.air_run = path, sha, air_run

        def accepted(future):
            try:
                self.goal = future.result()
                if not self.goal.accepted:
                    self.execution_requested = False
                    done(False, 'Executor rejected the job; check its status and calibration')
                    return
                if self.execution_cancel.is_set():
                    self.goal.cancel_goal_async()
                def completed(result_future):
                    try:
                        result = result_future.result().result
                        done(result.success, result.message)
                    except Exception as exc:
                        done(False, str(exc))
                    finally:
                        self.goal = None
                        self.execution_requested = False
                self.goal.get_result_async().add_done_callback(completed)
            except Exception as exc:
                self.execution_requested = False
                done(False, str(exc))
        future = self.client.send_goal_async(goal, feedback_callback=lambda msg:
                                            feedback(msg.feedback.phase, msg.feedback.progress, msg.feedback.sample_index))
        future.add_done_callback(accepted)

    def stop(self):
        self.cancel_preparation()
        self.execution_cancel.set()
        if self.goal and self.goal.accepted:
            self.goal.cancel_goal_async()
        if self.execution_requested and self.stop_client.service_is_ready():
            self.stop_client.call_async(self.stop_type.Request())


def make_handler(flow, web_root, model, camera_url='http://127.0.0.1:8082'):
    web_root = Path(web_root).resolve()

    class Handler(BaseHTTPRequestHandler):
        server_version = 'PenBrowser/1.0'

        def log_message(self, fmt, *args):
            if not any(path in (args[0] if args else '') for path in ('/api/state', '/api/camera/frame.jpg')):
                super().log_message(fmt, *args)

        def session_key(self):
            cookie = SimpleCookie()
            try:
                cookie.load(self.headers.get('Cookie', ''))
            except Exception:
                pass
            key = cookie['pen_session'].value if 'pen_session' in cookie else ''
            if len(key) != 32 or any(c not in '0123456789abcdef' for c in key):
                key = secrets.token_hex(16)
            self.cookie = key
            return key

        def send(self, body, kind='application/json', code=200):
            if not isinstance(body, bytes):
                body = json.dumps(body, allow_nan=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Set-Cookie', f'pen_session={self.cookie}; HttpOnly; SameSite=Strict; Path=/')
            self.end_headers()
            self.wfile.write(body)

        def file(self, path):
            path = Path(path)
            if not path.is_file():
                raise WorkflowError('File not found')
            self.send(path.read_bytes(), mimetypes.guess_type(path.name)[0] or 'application/octet-stream')

        def do_GET(self):
            key = self.session_key()
            path = unquote(urlparse(self.path).path)
            try:
                if path == '/api/camera/frame.jpg':
                    try:
                        with urlopen(camera_url.rstrip('/') + '/frame.jpg', timeout=1) as response:
                            body = response.read(2_000_000)
                            self.send_response(200)
                            self.send_header('Content-Type', 'image/jpeg')
                            self.send_header('Content-Length', str(len(body)))
                            self.send_header('Cache-Control', 'no-store')
                            for header in ('X-Frame-Id', 'X-Frame-Age-Ms'):
                                self.send_header(header, response.headers.get(header, ''))
                            self.end_headers()
                            self.wfile.write(body)
                        return
                    except (URLError, HTTPError, TimeoutError):
                        return self.send({'error': 'Robot camera unavailable; reconnecting'}, code=503)
                if path == '/api/state':
                    return self.send(flow.state(key))
                if path == '/api/model':
                    surface = parse_qs(urlparse(self.path).query).get('surface', [None])[0]
                    if surface is not None and flow.model_for_surface:
                        return self.send(flow.model_for_surface(surface))
                    return self.send(model())
                if path.startswith('/api/input/'):
                    return self.file(flow.input_path(path[len('/api/input/'):]))
                if path.startswith('/api/job/'):
                    parts = path.strip('/').split('/')
                    if len(parts) not in (3, 4):
                        raise WorkflowError('Unknown job resource')
                    job = flow.get_job(key, parts[2])
                    if len(parts) == 3:
                        return self.send(flow.public_job(job))
                    allowed = {'job.json', 'drawing.json', 'relief.json', 'toolpath.json',
                               'report.json', 'picture.png', 'relief.png', 'drawing.png', 'execution.json',
                               'robot_preview.json'}
                    if parts[3] not in allowed:
                        raise WorkflowError('Unknown job file')
                    return self.file(Path(job['folder']) / parts[3])
                if path.startswith('/api/mesh/'):
                    relative = Path(path[len('/api/mesh/'):])
                    roots = {'pen_description': flow.root / 'assets/meshes',
                             'flexiv_description': flow.root / 'src/flexiv_description',
                             'realsense2_description': flow.root / 'src/realsense-ros/realsense2_description'}
                    if not relative.parts or relative.parts[0] not in roots:
                        raise WorkflowError('Unknown mesh package')
                    root = roots[relative.parts[0]].resolve()
                    tail = Path(*relative.parts[1:])
                    if relative.parts[0] == 'pen_description' and tail.parts and tail.parts[0] == 'meshes':
                        tail = Path(*tail.parts[1:])
                    candidate = (root / tail).resolve()
                    if not candidate.is_relative_to(root):
                        raise WorkflowError('Invalid mesh path')
                    return self.file(candidate)
                if path.startswith('/api/asset/'):
                    root = (flow.root / 'src/pen_description/assets').resolve()
                    candidate = (root / path[len('/api/asset/'):]).resolve()
                    if not candidate.is_relative_to(root):
                        raise WorkflowError('Invalid asset path')
                    if candidate.suffix.lower() not in ('.glb', '.gltf', '.stl', '.json'):
                        raise WorkflowError('Unsupported asset type')
                    return self.file(candidate)
                candidate = (web_root / (path.lstrip('/') or 'index.html')).resolve()
                if not candidate.is_relative_to(web_root):
                    raise WorkflowError('Invalid path')
                return self.file(candidate)
            except (WorkflowError, OSError, ValueError) as exc:
                self.send({'error': str(exc)}, code=400)

        def do_POST(self):
            key = self.session_key()
            try:
                origin = self.headers.get('Origin')
                if origin and urlparse(origin).netloc != self.headers.get('Host'):
                    raise WorkflowError('Cross-origin commands are not allowed')
                if self.headers.get('X-Pen-Request') != '1':
                    raise WorkflowError('Missing request marker')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 17 * 1024 * 1024:
                    raise WorkflowError('Request is empty or too large')
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise WorkflowError('Expected a JSON object')
                if self.path == '/api/upload':
                    return self.send({'image': flow.upload(key, data.get('name'), data.get('data'))})
                if self.path == '/api/preview':
                    return self.send(flow.prepare(key, data), code=202)
                if self.path == '/api/invalidate':
                    flow.invalidate(key)
                    return self.send({'ok': True})
                if self.path == '/api/execute':
                    return self.send(flow.execute(key, data.get('job_id'), data.get('preview_token'),
                                                  data.get('air_run', False)), code=202)
                if self.path == '/api/home':
                    return self.send(flow.home(key, data.get('job_id')), code=202)
                if self.path == '/api/warmup':
                    return self.send(flow.warmup(key), code=202)
                if self.path == '/api/stop':
                    return self.send(flow.stop(key))
                raise WorkflowError('Unknown command')
            except (WorkflowError, OSError, ValueError, TypeError) as exc:
                self.send({'error': str(exc)}, code=400)

    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', default=str(Path.home() / 'robotics_ws'))
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--mode', choices=('preview', 'fake', 'real'), default='preview')
    parser.add_argument('--profile')
    parser.add_argument('--prepared-job', help='Reuse this checked job when its inputs and starting pose match')
    parser.add_argument('--no-ros', action='store_true')
    parser.add_argument('--camera-url', default='http://127.0.0.1:8082')
    args, ros_args = parser.parse_known_args()
    bridge = None
    if not args.no_ros:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        bridge = RosBridge(f'pen_web_bridge_{args.port}')
    flow = Workflow(args.root, args.mode, args.profile, bridge, prepared_job=args.prepared_job)
    if bridge:
        bridge.workflow = flow
    try:
        from ament_index_python.packages import get_package_share_directory
        web = Path(get_package_share_directory('pen_bringup')) / 'web'
    except (ImportError, LookupError):
        web = Path(__file__).resolve().parents[1] / 'web'
    from .model import Model
    model = Model(flow.root, flow.profile, flow.mode)
    flow.model_provider = model.get
    if args.mode == 'preview' and flow.profile and flow.profile.name == 'tylenol.yaml':
        config = flow.root / 'src/pen_description/config'
        flow.surface_profiles = {kind: config / name for kind, name in
                                 [('plane', 'flat.yaml'), ('sphere', 'sphere.yaml'), ('cylinder', 'tylenol.yaml')]}
        models = {kind: Model(flow.root, path, 'preview') for kind, path in flow.surface_profiles.items()}
        def model_for_surface(kind):
            if kind not in models:
                raise WorkflowError('Unknown workpiece')
            return models[kind].get()
        flow.model_for_surface = model_for_surface
    server = ThreadingHTTPServer((args.host, args.port), make_handler(flow, web, model.get, args.camera_url))
    # ROS launch/background shells may pass an ignored SIGINT disposition.
    # Own shutdown in the HTTP thread; ROS spinning uses the same cleanup path.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    print(f'Pen browser: http://{args.host}:{args.port} ({args.mode})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        if bridge:
            bridge.stop()
            time.sleep(0.15)
    finally:
        server.server_close()
        if bridge:
            rclpy.try_shutdown()
            bridge.thread.join(timeout=2)
            bridge.node.destroy_node()


if __name__ == '__main__':
    main()
