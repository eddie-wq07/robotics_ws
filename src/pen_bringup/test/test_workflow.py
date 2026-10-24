import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pen_bringup.service import Workflow, WorkflowError, digest


class Bridge:
    def __init__(self): self.calls=[]
    def ready(self): return True
    def execute(self,*args): self.calls.append(args)
    def snapshot(self): return {}
    def stop(self): self.calls.append('stop')


class WorkflowTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        self.bridge=Bridge()
        self.flow=Workflow(self.root,'fake',bridge=self.bridge)
        (self.flow.inputs/'apple.png').write_bytes(b'original image')
        folder=self.flow.outputs/'job';folder.mkdir()
        (folder/'job.json').write_text(json.dumps({'samples':[1]}))
        self.job={'id':'job','owner':'owner','state':'ready','folder':str(folder),'image':'apple.png',
                  'sha256':digest(folder/'job.json'),'input_sha256':digest(self.flow.inputs/'apple.png'),
                  'preview_token':'review-token','progress':0.0}
        self.flow.jobs['job']=self.job
        self.flow.session('owner')['reviewed']='job'

    def tearDown(self): self.temp.cleanup()

    def test_successful_review_is_consumed_and_duplicate_execute_rejected(self):
        self.flow.execute('owner','job','review-token')
        self.assertEqual(len(self.bridge.calls),1)
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_changed_input_invalidates_execution(self):
        (self.flow.inputs/'apple.png').write_bytes(b'changed image')
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')
        self.assertEqual(self.bridge.calls,[])

    def test_changed_artifact_cannot_use_previous_review(self):
        (Path(self.job['folder'])/'job.json').write_text('{"changed":true}')
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_edit_invalidates_and_other_session_cannot_execute(self):
        with self.assertRaises(WorkflowError): self.flow.execute('other','job','review-token')
        self.flow.invalidate('owner')
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_preview_only_never_executes(self):
        self.flow.mode='preview'
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_wrong_token_and_unreviewed_job_rejected(self):
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','wrong')
        self.job['state']='preparing'
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_paths_cannot_escape_inputs(self):
        for name in ('../secret.png','/etc/passwd','missing.png'):
            with self.assertRaises(WorkflowError): self.flow.input_path(name)

    def test_replaced_profile_requires_another_preview(self):
        profile=self.root/'calibration.yaml';profile.write_text('one')
        self.flow.profile=profile;self.job['profile_sha256']=digest(profile)
        profile.write_text('two')
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')

    def test_late_feedback_cannot_overwrite_terminal_result(self):
        self.flow.execute('owner','job','review-token')
        self.flow.finish('job',False,'Client heartbeat expired; commands stopped')
        self.flow.progress('job','Checking controller and hardware state',0.0)
        self.assertEqual(self.job['state'],'stopped')
        self.assertEqual(self.job['message'],'Client heartbeat expired; commands stopped')
        self.assertIsNone(self.flow.active_job)

    def test_missing_robot_model_does_not_claim_preparation_slot(self):
        self.flow.model_provider=lambda:{'available':False}
        with self.assertRaisesRegex(WorkflowError,'Robot description unavailable'):
            self.flow.prepare('owner',{'image':'apple.png'})
        self.assertFalse(self.flow.preparing)
        self.assertIsNone(self.flow.preparing_job)


if __name__=='__main__': unittest.main()
