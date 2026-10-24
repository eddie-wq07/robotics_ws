#!/usr/bin/env python3
"""Bounded integration test; starts ONLY explicit fake hardware on domain 91."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "91"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListControllers, SwitchController
from flexiv_msgs.msg import GPIOStates
from pen_interfaces.action import ExecuteDrawing
from std_msgs.msg import Empty


def main():
    directory = Path(tempfile.mkdtemp(prefix="pen-fake-integration-"))
    processes = []
    handles = []
    rclpy.init()
    node = rclpy.create_node("pen_integration_test")
    publisher = node.create_publisher(Empty, "/pen/heartbeat", 10)
    timer = node.create_timer(0.1, lambda: publisher.publish(Empty()))
    timer.cancel()  # Exercise action delivery before the first client heartbeat.
    outputs = []
    subscription = node.create_subscription(
        GPIOStates, "/Rizon4s_000000/gpio_outputs",
        lambda message: outputs.extend((entry.pin, entry.state) for entry in message.states), 10)

    def start(name, command):
        output = (directory / (name + ".log")).open("w")
        handles.append(output)
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=os.environ.copy())
        processes.append(process)
        return process

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def future_result(future, seconds=10):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        if not future.done():
            raise TimeoutError("ROS operation exceeded test deadline")
        return future.result()

    try:
        start("driver", ["ros2", "launch", "flexiv_cartesian_controller", "bringup.launch.py",
                         "robot_sn:=Rizon4s-000000", "use_fake_hardware:=true", "start_rviz:=false"])
        controllers = node.create_client(ListControllers, "/controller_manager/list_controllers")
        assert controllers.wait_for_service(timeout_sec=15), "Fake controller manager unavailable"
        end = time.monotonic() + 15
        state = {}
        while time.monotonic() < end:
            result = future_result(controllers.call_async(ListControllers.Request()))
            state = {item.name: item.state for item in result.controller}
            if state.get("flexiv_cartesian_controller") == "inactive" and state.get("rizon_arm_controller") == "active":
                break
            pump(0.2)
        assert state.get("flexiv_cartesian_controller") == "inactive", state
        config = directory / "gpio.yaml"
        config.write_text('/**/gpio_controller:\n  ros__parameters:\n    robot_sn: "Rizon4s-000000"\n')
        gpio = start("gpio", ["ros2", "run", "controller_manager", "spawner", "gpio_controller", "-p", str(config)])
        assert gpio.wait(timeout=15) == 0
        start("executor", ["ros2", "run", "pen_execution", "executor", "--ros-args",
                           "-p", "execution_mode:=fake", "-p", "robot_sn:=Rizon4s-000000",
                           "-p", "jobs_root:=" + str(directory)])
        action = ActionClient(node, ExecuteDrawing, "/pen/execute")
        assert action.wait_for_server(timeout_sec=10), "Execution action unavailable"
        pump(0.5)
        samples = []
        for i in range(201):
            t = i / 100
            if i < 30:
                phase, on, x, z = "HOLD", False, 0, 0
            elif i < 80:
                phase, on, x, z = "DRAW", True, (i - 30) / 50 * .001, 0
            elif i < 100:
                phase, on, x, z = "RELEASE", False, .001, 0
            elif i < 150:
                phase, on, x, z = "LIFT", False, .001, (i - 100) / 50 * .001
            else:
                phase, on, x, z = "TRAVEL", False, .001 * (200 - i) / 50, .001
            samples.append(dict(time_s=t, pose=[x, 0, z, 0, 0, 0, 1], phase=phase, extrude=on))
        job = dict(schema_version=1, kind="pen_job", frame="world", units="m",
                   pose_format="x,y,z,qx,qy,qz,qw", fake_only=True, calibrated=False,
                   profile_sha256="a" * 64, samples=samples,
                   pen=dict(release_delay_s=.1,press_delay_s=0,max_press_s=30),
                   pen_events=[dict(time_s=0, extrude=False), dict(time_s=.3, extrude=True),
                               dict(time_s=.8, extrude=False), dict(time_s=1, extrude=False)])
        if os.environ.get("PEN_TEST_JOB"):
            job = json.loads(Path(os.environ["PEN_TEST_JOB"]).read_text())
            assert job["fake_only"], "Only fake jobs can be supplied to this test"
        path = directory / "job.json"
        path.write_text(json.dumps(job))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()

        def request(checksum):
            goal = ExecuteDrawing.Goal()
            goal.job_path, goal.expected_sha256 = str(path), checksum
            return future_result(action.send_goal_async(goal))

        no_client = request(digest)
        assert no_client.accepted
        no_client_result = future_result(no_client.get_result_async(), 5).result
        assert not no_client_result.success and "initial preflight" in no_client_result.message
        assert not outputs, "Missing initial heartbeat must not change GPIO"
        waiting_cancel = request(digest)
        assert waiting_cancel.accepted
        pump(.15)
        future_result(waiting_cancel.cancel_goal_async())
        waiting_cancel_result = future_result(waiting_cancel.get_result_async(), 3).result
        assert not waiting_cancel_result.success and "canceled" in waiting_cancel_result.message.lower()
        assert not outputs, "Canceling heartbeat wait must not change GPIO"
        initial_state = future_result(controllers.call_async(ListControllers.Request()))
        initial_state = {item.name: item.state for item in initial_state.controller}
        assert initial_state["rizon_arm_controller"] == "active", initial_state
        assert initial_state["flexiv_cartesian_controller"] == "inactive", initial_state
        goal = request(digest)
        assert goal.accepted
        pump(.25)
        assert not outputs, "Initial heartbeat wait must not send hardware commands"
        timer.reset()
        result = future_result(goal.get_result_async(), min(180, job["samples"][-1]["time_s"] + 30)).result
        assert result.success, result.message
        assert any(pin == 0 and value for pin, value in outputs), outputs
        assert outputs[-1] == (0, False), outputs
        assert all(pin == 0 for pin, _ in outputs), outputs
        states = future_result(controllers.call_async(ListControllers.Request()))
        state = {item.name: item.state for item in states.controller}
        assert state["rizon_arm_controller"] == "active", state
        assert state["flexiv_cartesian_controller"] == "inactive", state
        initial_count = len(outputs)
        bad = request("0" * 64)
        assert bad.accepted, "Bad checksum must be accepted for validation then aborted"
        bad_result = future_result(bad.get_result_async()).result
        assert not bad_result.success and "digest" in bad_result.message.lower(), bad_result
        pump(.1)
        assert len(outputs) == initial_count, "Rejected job changed digital outputs"
        canceled = request(digest)
        assert canceled.accepted
        pump(.8)
        future_result(canceled.cancel_goal_async())
        cancel_result = future_result(canceled.get_result_async()).result
        assert not cancel_result.success
        states = future_result(controllers.call_async(ListControllers.Request()))
        state = {item.name: item.state for item in states.controller}
        assert state["rizon_arm_controller"] == "inactive", state
        assert state["flexiv_cartesian_controller"] == "inactive", state
        assert outputs[-1] == (0, False)
        switch = node.create_client(SwitchController, "/controller_manager/switch_controller")
        restart = SwitchController.Request()
        restart.activate_controllers = ["rizon_arm_controller"]
        restart.strictness = SwitchController.Request.STRICT
        restart.timeout.sec = 3
        assert future_result(switch.call_async(restart)).ok
        pump(.2)
        lost_client = request(digest)
        assert lost_client.accepted
        pump(.3)
        timer.cancel()
        heartbeat_result = future_result(lost_client.get_result_async(), 8).result
        assert not heartbeat_result.success and "heartbeat" in heartbeat_result.message.lower(), heartbeat_result
        assert outputs[-1] == (0, False)
        states = future_result(controllers.call_async(ListControllers.Request()))
        state = {item.name: item.state for item in states.controller}
        assert state["rizon_arm_controller"] == "inactive", state
        assert state["flexiv_cartesian_controller"] == "inactive", state
        report = dict(success=True, completed=result.message, canceled=cancel_result.message,
                      digest_rejected=bad_result.message, final_controllers=state,
                      heartbeat_loss=heartbeat_result.message,
                      delayed_initial_heartbeat=True,
                      initial_heartbeat_timeout=no_client_result.message,
                      initial_heartbeat_cancel=waiting_cancel_result.message,
                      gpio_requests=outputs, evidence=str(directory))
        (directory / "result.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
        for handle in handles:
            handle.close()
        node.destroy_node()
        rclpy.shutdown()
        print("Evidence:", directory)


if __name__ == "__main__":
    main()
