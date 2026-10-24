#!/usr/bin/env python3
"""Copyright 2026 Industrial Next. All rights reserved.

Supervised Cartesian offset publisher. No physical motion without --execute.
The native controller, not this Python process, owns the 1 kHz SDK loop.
"""
import argparse
import copy
import math
import os
import signal
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions


class OffsetTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("cartesian_offset_test")
        self.args = args
        self.feedback = None
        self.feedback_at = 0.0
        self.feedback_error = None
        self.target = None
        self.origin = None
        self.publishing = False
        self.interrupted = False
        self.owns_switch = False
        self.monitor_motion = False
        self.last_publish = 0.0
        self.last_status = None
        self.activation_at = float("inf")
        self.samples = 0
        self.max_error = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            PoseStamped, "/flexiv_cartesian_controller/target_pose", qos
        )
        self.create_subscription(PoseStamped, args.state_topic, self.receive_pose, qos)
        self.create_subscription(
            DiagnosticArray, "/flexiv_cartesian_controller/status", self.receive_status, 10
        )
        self.create_timer(0.01, self.publish_target)
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

    def receive_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        values = [getattr(msg.pose.position, a) for a in "xyz"] + [q.w, q.x, q.y, q.z]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age = self.get_clock().now().nanoseconds / 1e9 - stamp
        if (msg.header.frame_id != self.args.frame
                or not all(math.isfinite(value) for value in values)
                or abs(sum(value * value for value in values[3:]) - 1.0) > 0.002
                or stamp <= 0 or age < -0.05 or age > 0.25):
            self.feedback_error = "Invalid or stale measured pose; stopping test"
            return
        self.feedback = msg
        self.feedback_at = time.monotonic()

    def receive_status(self, msg: DiagnosticArray) -> None:
        self.last_status = msg

    def publish_target(self) -> None:
        if not self.publishing or self.target is None or self.interrupted:
            return
        now = time.monotonic()
        if self.last_publish and now - self.last_publish > 0.08:
            self.publishing = False
            return
        msg = PoseStamped()
        msg.header.frame_id = self.args.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = copy.deepcopy(self.target)
        self.publisher.publish(msg)
        self.last_publish = now

    def check(self) -> None:
        if self.interrupted:
            raise KeyboardInterrupt
        if self.feedback_error:
            raise RuntimeError(self.feedback_error)
        if not self.monitor_motion:
            return
        if not self.publishing:
            raise RuntimeError("Publisher stalled; stopping test")
        if time.monotonic() - self.feedback_at > 0.25:
            raise RuntimeError("Measured pose feedback is stale; stopping test")
        error = math.sqrt(sum(
            (getattr(self.feedback.pose.position, axis) - getattr(self.target.position, axis)) ** 2
            for axis in "xyz"
        ))
        self.max_error = max(self.max_error, error)
        self.samples += 1
        if error > 0.015:
            raise RuntimeError(f"Tracking error {error:.3f} m exceeds 0.015 m; stopping test")
        if self.last_status is not None:
            stamp = self.last_status.header.stamp
            stamp_s = stamp.sec + stamp.nanosec / 1e9
            if stamp_s >= self.activation_at and any(s.level in (2, 3, bytes([2]), bytes([3])) for s in self.last_status.status):
                raise RuntimeError("Cartesian controller reports a latched stop")

    def pump(self, seconds: float, check: bool = True) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)
            if check:
                self.check()

    def call(self, client, request, check: bool = True):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f"Service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self.pump(0.01, check)
        if not future.done():
            raise TimeoutError(client.srv_name)
        return future.result()

    def controllers(self, check: bool = True) -> dict:
        result = self.call(self.list_client, ListControllers.Request(), check)
        return {controller.name: controller.state for controller in result.controller}

    def switch(self, activate: list, deactivate: list, check: bool = True) -> None:
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.timeout.sec = 3
        if not self.call(self.switch_client, request, check).ok:
            raise RuntimeError("Controller switch rejected")

    def run(self) -> None:
        deadline = time.monotonic() + 5.0
        while self.feedback is None and time.monotonic() < deadline:
            self.pump(0.05)
        if self.feedback is None or time.monotonic() - self.feedback_at > 0.25:
            raise RuntimeError(f"No fresh measured TCP pose on {self.args.state_topic}. Verify bringup completed, controllers are loaded, and the robot is in AUTO (REMOTE).")
        if self.feedback.header.frame_id != self.args.frame:
            raise RuntimeError(f"Measured frame is {self.feedback.header.frame_id!r}, expected {self.args.frame!r}")
        self.origin = copy.deepcopy(self.feedback.pose)
        q = self.origin.orientation
        values = [getattr(self.origin.position, a) for a in "xyz"] + [q.w, q.x, q.y, q.z]
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError("Non-finite initial pose")
        norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if abs(norm - 1.0) > 0.001:
            raise RuntimeError("Initial orientation is not a unit quaternion")
        axes = "xyz" if self.args.axis == "all" else self.args.axis
        print("Start pose [m]:", [round(v, 5) for v in values[:3]], flush=True)
        print(f"Each axis {axes}: start -> +{self.args.distance:.3f} m -> start -> -{self.args.distance:.3f} m -> start", flush=True)
        print(f"Each leg {self.args.seconds_per_leg:.1f} s; orientation held; targets published at 100 Hz", flush=True)
        if not self.args.execute:
            print("Preview only. Add --execute to switch controllers and physically move the arm.", flush=True)
            return
        state = self.controllers()
        if state.get("rizon_arm_controller") != "active" or state.get("flexiv_cartesian_controller") != "inactive":
            raise RuntimeError("Start with joint controller active and Cartesian controller inactive")
        self.target = copy.deepcopy(self.origin)
        self.publishing = True
        self.activation_at = self.get_clock().now().nanoseconds / 1e9
        self.owns_switch = True
        self.switch(["flexiv_cartesian_controller"], ["rizon_arm_controller"])
        self.monitor_motion = True
        self.pump(0.5)
        if self.controllers().get("flexiv_cartesian_controller") != "active":
            raise RuntimeError("Cartesian controller did not remain active")
        for axis in axes:
            offsets = (0.0, self.args.distance, 0.0, -self.args.distance, 0.0)
            for start, end in zip(offsets, offsets[1:]):
                print(f"{axis}: {start:+.3f} -> {end:+.3f} m", flush=True)
                began = time.monotonic()
                while True:
                    ratio = min((time.monotonic() - began) / self.args.seconds_per_leg, 1.0)
                    blend = (1.0 - math.cos(math.pi * ratio)) / 2.0
                    offset = start + (end - start) * blend
                    setattr(self.target.position, axis, getattr(self.origin.position, axis) + offset)
                    self.pump(0.005)
                    if ratio >= 1.0:
                        break
                self.pump(0.25)
        self.pump(0.5)
        self.switch(["rizon_arm_controller"], ["flexiv_cartesian_controller"])
        self.publishing = False
        self.monitor_motion = False
        self.owns_switch = False
        print(f"Completed. Joint control restored. Max observed tracking error: {self.max_error:.4f} m", flush=True)

    def stop_after_error(self) -> None:
        self.publishing = False
        self.monitor_motion = False
        if not self.owns_switch:
            return
        try:
            if self.controllers(check=False).get("flexiv_cartesian_controller") == "active":
                self.switch([], ["flexiv_cartesian_controller"], check=False)
            print("Test stopped; no automatic return motion or joint reactivation.", flush=True)
        except Exception as error:
            print(f"Could not confirm deactivation: {error}. Target publishing has stopped; use the physical stop if needed.", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=["x", "y", "z", "all"], default="x")
    parser.add_argument("--distance", type=float, default=0.03, help="Magnitude in metres, maximum 0.03")
    parser.add_argument("--seconds-per-leg", type=float, default=4.0, help="At least 3 seconds per leg")
    serial = os.environ.get("FLEXIV_ROBOT_SN", "Rizon4s-063112").replace("-", "_")
    parser.add_argument("--state-topic", default=f"/{serial}/tcp_pose")
    parser.add_argument("--frame", default="world")
    parser.add_argument("--execute", action="store_true", help="Authorize physical motion and controller switching")
    args = parser.parse_args()
    if not math.isfinite(args.distance) or not 0 < args.distance <= 0.03:
        parser.error("distance must be positive and at most 0.03 m")
    if not math.isfinite(args.seconds_per_leg) or args.seconds_per_leg < 3.0:
        parser.error("seconds-per-leg must be at least 3")
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OffsetTest(args)
    signal.signal(signal.SIGINT, lambda *_: setattr(node, "interrupted", True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "interrupted", True))
    try:
        node.run()
        return 0
    except (Exception, KeyboardInterrupt) as error:
        print(f"Aborted: {error or 'interrupted'}", flush=True)
        node.stop_after_error()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
