"""Shared physical D405 feed, independent of ROS domains and robot execution."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import signal
import threading
import time


class FrameStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.received = 0.
        self.sequence = 0
        self.serial = None
        self.error = 'Waiting for the robot camera'

    def update(self, jpeg, serial):
        with self.lock:
            self.jpeg, self.serial = jpeg, serial
            self.received = time.monotonic()
            self.sequence += 1
            self.error = None

    def failed(self, error):
        with self.lock:
            self.error = str(error)
            self.jpeg = None

    def snapshot(self):
        with self.lock:
            age = time.monotonic() - self.received
            live = self.jpeg is not None and age < 2
            return (self.jpeg if live else None), {
                'live': live, 'sequence': self.sequence, 'serial': self.serial,
                'age_ms': round(age * 1000) if self.received else None,
                'message': 'Live robot camera' if live else (self.error or 'Camera frames stopped'),
            }


def capture(store, stop, serial=None):
    # Only this service owns the camera. Every UI mode reads its cached frames.
    import cv2
    import numpy as np
    import pyrealsense2 as rs
    while not stop.is_set():
        pipeline = None
        try:
            devices = list(rs.context().query_devices())
            device = next((d for d in devices if
                (d.get_info(rs.camera_info.serial_number) == serial if serial else
                 'D405' in d.get_info(rs.camera_info.name))), None)
            if device is None:
                raise RuntimeError('Robot D405 is not connected')
            selected = device.get_info(rs.camera_info.serial_number)
            config = rs.config()
            config.enable_device(selected)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
            pipeline = rs.pipeline()
            pipeline.start(config)
            while not stop.is_set():
                frame = pipeline.wait_for_frames(1500).get_color_frame()
                if not frame:
                    continue
                pixels = cv2.cvtColor(np.asanyarray(frame.get_data()), cv2.COLOR_RGB2BGR)
                ok, jpeg = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    store.update(jpeg.tobytes(), selected)
        except Exception as error:
            store.failed(error)
        finally:
            if pipeline is not None:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
        stop.wait(2)


def make_handler(store):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            jpeg, status = store.snapshot()
            if self.path.split('?')[0] == '/frame.jpg' and jpeg is not None:
                body, kind, code = jpeg, 'image/jpeg', 200
            elif self.path.split('?')[0] in ('/status', '/frame.jpg'):
                body, kind, code = json.dumps(status).encode(), 'application/json', (200 if self.path == '/status' else 503)
            else:
                self.send_error(404)
                return
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Id', str(status['sequence']))
            self.send_header('X-Frame-Age-Ms', str(status['age_ms']))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8082)
    parser.add_argument('--serial')
    args, _ = parser.parse_known_args()
    store = FrameStore()
    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(store))
    except OSError as error:
        if error.errno == 98:
            print(f'Camera feed already owns port {args.port}; reusing it.', flush=True)
            return
        raise
    stop = threading.Event()
    thread = threading.Thread(target=capture, args=(store, stop, args.serial), daemon=True)
    thread.start()
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    print(f'Robot camera feed: http://127.0.0.1:{args.port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join(timeout=3)


if __name__ == '__main__':
    main()
