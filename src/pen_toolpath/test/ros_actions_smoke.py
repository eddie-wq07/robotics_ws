"""Run with ROS system Python after building; starts only processing nodes."""
import json
import os
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pen_interfaces.action import PrepareDrawing, GeneratePath

ROOT=Path(__file__).resolve().parents[3]
OUT=Path('/tmp/pen-action-check/job')
processes=[]
logs=[]
try:
 for package,exe in [('pen_image_processing','image_processing_node'),('pen_toolpath','path_generation_node')]:
  log=open('/tmp/'+exe+'.log','w');logs.append(log)
  processes.append(subprocess.Popen([str(ROOT/'install'/package/'lib'/package/exe),'--ros-args','-p','jobs_root:=/tmp/pen-action-check'],stdout=log,stderr=log))
 rclpy.init();node=Node('pipeline_action_test')
 def call(kind,topic,data):
  client=ActionClient(node,kind,topic)
  assert client.wait_for_server(timeout_sec=10),topic
  goal=kind.Goal();goal.request_json=json.dumps(data);goal.output_directory=str(OUT)
  f=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,f,timeout_sec=10)
  assert f.done() and f.result().accepted,topic
  result=f.result().get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=40)
  assert result.done(),topic
  value=result.result().result
  assert value.success,value.message
  return value.artifact_path
 image=call(PrepareDrawing,'/pen/prepare_drawing',{'image_path':str(ROOT/'assets/inputs/apple.png'),'size_mm':20})
 job=call(GeneratePath,'/pen/generate_path',{'drawing_path':image,'surface':'sphere','profile_path':str(ROOT/'src/pen_description/config/fake.yaml'),'execution_mode':'fake'})
 value=json.loads(Path(job).read_text());assert len(value['samples'])>100
 assert (OUT/'drawing.png').exists()
 print(json.dumps({'success':True,'drawing':image,'job':job,'samples':len(value['samples'])}))
 node.destroy_node();rclpy.shutdown()
finally:
 for process in processes:
  process.terminate()
 for process in processes:
  try:process.wait(timeout=5)
  except subprocess.TimeoutExpired:process.kill();process.wait()
 for log in logs:log.close()
