"""Thin ROS boundary. Scientific imports execute only in the isolated worker."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time


def main(stage='image'):
    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from pen_interfaces.action import PrepareDrawing, GeneratePath

    kind = PrepareDrawing if stage=='image' else GeneratePath

    class WorkerNode(Node):
        def __init__(self):
            super().__init__('image_processing_node' if stage=='image' else 'path_generation_node')
            self.declare_parameter('worker_python', str(Path.home()/'robotics_ws/.venv-pen/bin/python'))
            self.declare_parameter('worker_timeout_s',600.0)
            self.declare_parameter('jobs_root',str(Path.home()/'robotics_ws/assets/outputs'))
            self.lock=threading.Lock()
            self.server=ActionServer(self,kind,'/pen/prepare_drawing' if stage=='image' else '/pen/generate_path',
                execute_callback=self.execute,goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,
                callback_group=ReentrantCallbackGroup())
        def goal(self,_):
            return GoalResponse.ACCEPT if self.lock.acquire(blocking=False) else GoalResponse.REJECT
        def execute(self,goal):
            result=kind.Result(); process=None
            try:
                request=json.loads(goal.request.request_json)
                if stage=='image':request['stage']='image'
                else:request.pop('stage',None)
                if not goal.request.output_directory:
                    raise ValueError('output_directory is required')
                output=Path(goal.request.output_directory).resolve()
                root=Path(self.get_parameter('jobs_root').value).resolve()
                if not output.is_relative_to(root) or output == root:
                    raise ValueError('output_directory must be a job subdirectory of jobs_root')
                output.mkdir(parents=True,exist_ok=True)
                env=dict(os.environ);env.pop('PYTHONPATH',None);env.pop('PYTHONHOME',None)
                with tempfile.TemporaryDirectory(prefix='pen-request-') as temporary:
                    path=Path(temporary)/'request.json';path.write_text(json.dumps(request))
                    with tempfile.TemporaryFile(mode='w+') as stdout, tempfile.TemporaryFile(mode='w+') as stderr:
                        process=subprocess.Popen([self.get_parameter('worker_python').value,'-m',('pen_image_processing.worker' if stage=='image' else 'pen_toolpath.worker'),
                            '--request',str(path),'--out',str(output)],env=env,stdout=stdout,stderr=stderr)
                        deadline=time.monotonic()+float(self.get_parameter('worker_timeout_s').value)
                        feedback=kind.Feedback();feedback.stage='EXTRACTING' if stage=='image' else 'GENERATING';feedback.progress=0.1
                        goal.publish_feedback(feedback)
                        while process.poll() is None:
                            if goal.is_cancel_requested or time.monotonic()>deadline:
                                process.terminate()
                                try:process.wait(timeout=2)
                                except subprocess.TimeoutExpired:process.kill();process.wait()
                                result.message='Canceled' if goal.is_cancel_requested else 'Worker timeout'
                                if goal.is_cancel_requested:goal.canceled()
                                else:goal.abort()
                                return result
                            time.sleep(.02)
                        stdout.seek(0);stderr.seek(0)
                        if goal.is_cancel_requested:
                            result.message='Canceled';goal.canceled();return result
                        if process.returncode:
                            result.message=stderr.read()[-4000:];goal.abort();return result
                        data=json.loads(stdout.read().splitlines()[-1]);result.success=True
                        result.artifact_path=data['artifact_path'];result.message='Completed'
                        goal.succeed();return result
            except Exception as error:
                result.message=str(error);goal.abort();return result
            finally:
                if process is not None and process.poll() is None:process.kill();process.wait()
                self.lock.release()

    rclpy.init();node=WorkerNode();executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:executor.shutdown();node.destroy_node();rclpy.try_shutdown()
