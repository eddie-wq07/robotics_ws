import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pen_bringup.service import Workflow, WorkflowError, digest, input_digest


class Bridge:
    def __init__(self): self.calls=[]
    def ready(self): return True
    def execute(self,*args): self.calls.append(args)
    def snapshot(self): return {}
    def stop(self): self.calls.append('stop')
    def warmup(self,*args):self.calls.append(('warmup',args))


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

    def test_fill_selection_is_forwarded_to_image_worker(self):
        with patch('pen_bringup.service.threading.Thread'):
            result=self.flow.prepare('owner',{'image':'apple.png','fill':True})
        request=json.loads((Path(self.flow.jobs[result['id']]['folder'])/'request.json').read_text())
        self.assertTrue(request['fill'])
        self.assertTrue(result['fill'])

    def test_fill_selection_requires_boolean(self):
        with self.assertRaisesRegex(WorkflowError,'true or false'):
            self.flow.prepare('owner',{'image':'apple.png','fill':'yes'})

    def test_reload_reattaches_to_identical_pending_preparation(self):
        with patch('pen_bringup.service.threading.Thread') as worker:
            first = self.flow.prepare('owner', {'image': 'apple.png'})
            self.flow.invalidate('owner')
            second = self.flow.prepare('owner', {'image': 'apple.png'})
        self.assertEqual(first['id'], second['id'])
        self.assertEqual(worker.call_count, 1)
        self.assertEqual(second['generation'], self.flow.session('owner')['generation'])
        with self.assertRaisesRegex(WorkflowError, 'current job'):
            self.flow.prepare('someone-else', {'image': 'apple.png'})
        (self.flow.inputs/'apple.png').write_bytes(b'changed image')
        with self.assertRaisesRegex(WorkflowError, 'current job'):
            self.flow.prepare('owner', {'image': 'apple.png'})

    def test_tylenol_requires_its_profile_and_real_approval(self):
        with self.assertRaisesRegex(WorkflowError, 'tylenol.yaml'):
            self.flow.prepare('owner', {'image': 'apple.png', 'surface': 'cylinder'})
        self.flow.mode = 'real'
        with self.assertRaisesRegex(WorkflowError, 'tylenol.yaml'):
            self.flow.prepare('owner', {'image': 'apple.png', 'surface': 'cylinder'})
        self.assertEqual(self.bridge.calls, [])
        profile = self.root / 'tylenol.yaml'
        profile.write_text(json.dumps({'surface': {'type': 'cylinder'}}))
        self.flow.profile = profile
        with self.assertRaisesRegex(WorkflowError, 'approved real calibration'):
            self.flow.prepare('owner', {'image': 'apple.png', 'surface': 'cylinder'})
        profile.write_text(json.dumps({'surface': {'type': 'cylinder'}, 'calibrated': True, 'fake_only': False}))
        with patch('pen_bringup.service.threading.Thread'):
            result = self.flow.prepare('owner', {'image': 'apple.png', 'surface': 'cylinder'})
        self.assertEqual(result['state'], 'preparing')
        self.assertEqual(self.bridge.calls, [])

    def test_tylenol_preview_uses_selected_profile_and_full_cylinder_spec(self):
        profile = self.root / 'tylenol.yaml'
        spec = {'type': 'cylinder', 'radius_m': .0291, 'height_m': .03, 'lying': True}
        profile.write_text(json.dumps({'surface': spec}))
        self.flow.mode = 'preview'
        self.flow.surface_profiles = {'cylinder': profile}
        with patch('pen_bringup.service.threading.Thread'):
            result = self.flow.prepare('owner', {'image': 'apple.png', 'surface': 'cylinder'})
        request = json.loads((Path(self.flow.jobs[result['id']]['folder']) / 'request.json').read_text())
        self.assertEqual(request['surface'], spec)
        self.assertEqual(request['profile_path'], str(profile))
        self.assertEqual(request['execution_mode'], 'preview')

    def test_warmup_is_separate_and_preserves_review_without_starting_drawing(self):
        warmup=self.flow.warmup('owner')
        self.assertEqual(warmup['action'],'warmup')
        self.assertEqual(self.flow.session('owner')['reviewed'],'job')
        self.assertEqual(self.job['state'],'ready')
        done,feedback=self.bridge.calls[-1][1]
        feedback('WARMUP_HEATING',.5)
        self.assertEqual(self.flow.get_job('owner',warmup['id'])['progress'],.5)
        with self.assertRaisesRegex(WorkflowError,'active'):
            self.flow.execute('owner','job','review-token')
        with self.assertRaisesRegex(WorkflowError,'current operation'):
            self.flow.warmup('owner')
        with self.assertRaisesRegex(WorkflowError,'another browser'):
            self.flow.stop('other-owner')
        self.flow.stop('owner')
        self.assertEqual(self.bridge.calls[-1],'stop')
        done(False,'Warmup canceled; servo released')
        self.assertIsNone(self.flow.active_job)
        self.assertEqual(self.flow.get_job('owner',warmup['id'])['state'],'stopped')
        self.assertEqual(self.job['state'],'ready')

    def test_completed_warmup_does_not_execute_drawing(self):
        warmup=self.flow.warmup('owner')
        done,_=self.bridge.calls[-1][1]
        done(True,'Warmup complete — check pen temperature')
        self.assertEqual(self.flow.get_job('owner',warmup['id'])['state'],'complete')
        self.assertIsNone(self.flow.active_job)
        self.assertEqual(len(self.bridge.calls),1)
        self.assertEqual(self.flow.session('owner')['reviewed'],'job')

    def test_partial_preview_is_visible_without_authorizing_execution(self):
        self.job['state']='preparing'
        folder=Path(self.job['folder'])
        (folder/'toolpath.json').write_text('{}')
        public=self.flow.public_job(self.job)
        self.assertEqual(public['preview_files'],['toolpath.json'])
        self.assertEqual(public['preparation_stage'],'Checking robot trajectory')
        with self.assertRaisesRegex(WorkflowError,'successful Preview'):
            self.flow.execute('owner','job','review-token')
        self.assertEqual(self.bridge.calls,[])

    def test_warmup_rejects_preview_and_preparation(self):
        self.flow.mode='preview'
        with self.assertRaisesRegex(WorkflowError,'connected'):
            self.flow.warmup('owner')
        self.flow.mode='fake';self.flow.preparing=True
        with self.assertRaisesRegex(WorkflowError,'current operation'):
            self.flow.warmup('owner')
        self.assertEqual(self.bridge.calls,[])

    def test_execution_feedback_preserves_exact_sample_index(self):
        self.flow.execute('owner','job','review-token')
        self.flow.progress('job','DRAW',.25,123)
        self.assertEqual(self.job['sample_index'],123)
        self.assertEqual(self.job['progress'],.25)

    def test_home_rejects_preview_and_busy_robot(self):
        self.flow.mode='preview'
        with self.assertRaisesRegex(WorkflowError,'real robot'):
            self.flow.home('owner','job')
        self.flow.mode='real';self.flow.active_job='job'
        with self.assertRaisesRegex(WorkflowError,'Stop'):
            self.flow.home('owner','job')
        self.assertEqual(self.bridge.calls,[])

    def test_already_at_recorded_joint_home_does_not_send_motion(self):
        self.flow.mode='real'
        names=[f'test_joint{i}' for i in range(1,8)]
        path=Path(self.job['folder'])/'job.json'
        path.write_text(json.dumps({'profile':{'home':{'controller':'rizon_arm_controller',
            'joint_names':names,'joint_positions_rad':[0]*7}}}))
        self.job['sha256']=digest(path)
        self.flow.profile=self.root/'home-profile.yaml'
        self.flow.profile.write_text(json.dumps(json.loads(path.read_text())['profile']))
        import time
        self.bridge.snapshot=lambda:{'joints_received_at':time.time(),'pose_received_at':time.time(),
            'joints':dict.fromkeys(names,0),'pose':[0,0,.3,0,0,0,1]}
        result=self.flow.home('owner','job')
        self.assertEqual(result['state'],'complete')
        self.assertIn('recorded joint Home',result['message'])
        self.assertEqual(self.bridge.calls,[])

    def test_home_requires_fresh_joint_state(self):
        self.flow.mode='real'
        self.flow.profile=self.root/'home-profile.yaml';self.flow.profile.write_text('{}')
        with self.assertRaisesRegex(WorkflowError,'Fresh measured'):
            self.flow.home('owner','job')
        self.assertEqual(self.bridge.calls,[])

    def test_home_at_target_works_without_any_drawing_job(self):
        import time
        self.flow.mode='real';self.flow.jobs.clear()
        names=[f'test_joint{i}' for i in range(1,8)]
        profile={'home':{'controller':'rizon_arm_controller','joint_names':names,'joint_positions_rad':[0]*7}}
        self.flow.profile=self.root/'home-profile.yaml'
        self.flow.profile.write_text(json.dumps(profile))
        self.bridge.snapshot=lambda:{'joints_received_at':time.time(),'pose_received_at':time.time(),
            'joints':dict.fromkeys(names,0),'pose':[0,0,.3,0,0,0,1]}
        result=self.flow.home('owner')
        self.assertEqual(result['state'],'complete')
        self.assertEqual(self.bridge.calls,[])

    def test_home_generates_from_profile_without_a_ready_drawing(self):
        import time
        from types import SimpleNamespace
        self.flow.mode='real';self.flow.jobs.clear()
        names=[f'test_joint{i}' for i in range(1,8)]
        profile={'robot_sn':'test','calibrated':True,'fake_only':False,'pen':{},
                 'home':{'controller':'rizon_arm_controller','joint_names':names,'joint_positions_rad':[.2]*7}}
        self.flow.profile=self.root/'home-profile.yaml';self.flow.profile.write_text(json.dumps(profile))
        self.flow.model_provider=lambda:{'available':True}
        self.bridge.snapshot=lambda:{'joints_received_at':time.time(),'pose_received_at':time.time(),
            'joints':dict.fromkeys(names,0),'pose':[0,0,.3,0,0,0,1]}
        def generate(command,**kwargs):
            request=json.loads(Path(command[command.index('--request')+1]).read_text())
            self.assertEqual(request['source']['profile'],profile)
            self.assertEqual(request['source']['profile_source_sha256'],digest(self.flow.profile))
            Path(command[command.index('--out')+1]).write_text('{}')
            return SimpleNamespace(returncode=0)
        with patch('pen_bringup.service.subprocess.run',side_effect=generate):
            result=self.flow.home('owner')
        self.assertEqual(result['state'],'executing')
        self.assertEqual(result['action'],'home')
        self.assertEqual(len(self.bridge.calls),1)

    def test_changed_input_invalidates_execution(self):
        (self.flow.inputs/'apple.png').write_bytes(b'changed image')
        with self.assertRaises(WorkflowError): self.flow.execute('owner','job','review-token')
        self.assertEqual(self.bridge.calls,[])

    def test_changed_annotation_invalidates_execution(self):
        image=self.flow.inputs/'apple.png'
        annotation=image.with_suffix('.png.strokes.json')
        annotation.write_text('original annotation')
        self.job['input_sha256']=input_digest(image)
        annotation.write_text('changed annotation')
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
