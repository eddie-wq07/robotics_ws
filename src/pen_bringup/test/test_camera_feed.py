import json
import threading
import sys
from pathlib import Path
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import urlopen
from unittest.mock import patch

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pen_bringup.camera_feed import FrameStore, make_handler


def test_camera_frame_freshness_and_disconnect():
    store=FrameStore()
    assert store.snapshot()[0] is None
    with patch('pen_bringup.camera_feed.time.monotonic',return_value=10):
        store.update(b'jpeg','camera-serial')
    with patch('pen_bringup.camera_feed.time.monotonic',return_value=10.1):
        data,status=store.snapshot()
        assert data==b'jpeg' and status['live'] and status['sequence']==1
    with patch('pen_bringup.camera_feed.time.monotonic',return_value=13):
        data,status=store.snapshot()
        assert data is None and not status['live']
    store.failed('unplugged')
    assert store.snapshot()[1]['message']=='unplugged'


def test_camera_http_never_serves_stale_frame_as_live():
    store=FrameStore();store.update(b'jpeg-data','test-camera')
    server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(store))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base+'/frame.jpg') as response:
            assert response.read()==b'jpeg-data'
            assert response.headers['X-Frame-Id']=='1'
            assert response.headers['Cache-Control']=='no-store'
        store.failed('unplugged')
        with pytest.raises(HTTPError) as error:urlopen(base+'/frame.jpg')
        assert error.value.code==503
        assert not json.loads(urlopen(base+'/status').read())['live']
    finally:
        server.shutdown();server.server_close();thread.join()
