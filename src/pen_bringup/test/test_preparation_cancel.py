import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pen_bringup.service import Workflow, WorkflowError


class BlockingPreparation:
    def __init__(self):
        self.started=threading.Event()
        self.cancel_calls=0
        self.motion_stop_calls=0
    def generate(self,request,folder,cancel_event=None):
        self.started.set()
        if not cancel_event.wait(2):
            raise RuntimeError('test preparation was never canceled')
        raise WorkflowError('Preparation canceled')
    def cancel_preparation(self):self.cancel_calls+=1
    def stop(self):self.motion_stop_calls+=1


class CancelTest(unittest.TestCase):
    def test_stop_cancels_preparation_and_never_unlocks_execute(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge=BlockingPreparation();flow=Workflow(directory,'fake',bridge=bridge)
            (flow.inputs/'test.png').write_bytes(b'input')
            result=flow.prepare('owner',{'image':'test.png'})
            self.assertTrue(bridge.started.wait(1))
            with self.assertRaises(WorkflowError):flow.stop('other-session')
            flow.stop('owner')
            deadline=time.monotonic()+2
            while flow.preparing and time.monotonic()<deadline:time.sleep(.01)
            job=flow.get_job('owner',result['id'])
            self.assertFalse(flow.preparing)
            self.assertEqual(job['state'],'stopped')
            self.assertNotIn('preview_token',job)
            self.assertEqual(bridge.cancel_calls,1)
            self.assertEqual(bridge.motion_stop_calls,0)

    def test_idle_stop_does_not_stop_unowned_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge=BlockingPreparation();flow=Workflow(directory,'fake',bridge=bridge)
            self.assertEqual(flow.stop('owner')['message'],'No active job')
            self.assertEqual(bridge.motion_stop_calls,0)


if __name__=='__main__':unittest.main()
