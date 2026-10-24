import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pen_bringup.web_bridge import RosBridge


class WarmupBridgeTest(unittest.TestCase):
    def make_bridge(self):
        bridge = object.__new__(RosBridge)
        self.finished, self.feedback, self.stops = [], [], []
        bridge.warmup_request = {'id': '42', 'started': time.monotonic(),
                                 'done': lambda *args: self.finished.append(args),
                                 'feedback': lambda *args: self.feedback.append(args)}
        bridge.execution_requested = True
        bridge.state = {'received_at': time.time(), 'status': {}}
        bridge.snapshot = lambda: bridge.state
        bridge.workflow = None
        bridge.stop_type = SimpleNamespace(Request=lambda: object())
        bridge.stop_client = SimpleNamespace(service_is_ready=lambda: True,
                                             call_async=lambda _: self.stops.append(True))
        return bridge

    def test_only_matching_request_can_finish_and_completion_is_once(self):
        bridge = self.make_bridge()
        bridge.state['status'] = {'warmup_id': '41', 'phase': 'WARMUP_COMPLETE'}
        bridge.update_warmup()
        self.assertEqual(self.finished, [])
        bridge.state['status'] = {'warmup_id': '42', 'phase': 'WARMUP_HEATING', 'progress': '.5'}
        bridge.update_warmup()
        self.assertEqual(self.feedback, [('WARMUP_HEATING', .5)])
        bridge.state['status']['phase'] = 'WARMUP_COMPLETE'
        bridge.update_warmup(); bridge.update_warmup()
        self.assertEqual(len(self.finished), 1)
        self.assertTrue(self.finished[0][0])
        self.assertFalse(bridge.execution_requested)

    def test_stale_feedback_requests_stop_and_reports_unknown_pen_state(self):
        bridge = self.make_bridge()
        bridge.state['received_at'] -= 3
        bridge.warmup_request['started'] -= 3
        bridge.tick()
        self.assertEqual(self.stops, [True])
        self.assertFalse(self.finished[0][0])
        self.assertIn('unconfirmed', self.finished[0][1])
        self.assertIsNone(bridge.warmup_request)
