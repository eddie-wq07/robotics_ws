#!/usr/bin/env python3
"""Load a job into MoveIt, plan it, and display it in RViz. Never execute motion.

Runs under Ubuntu's ROS Python; the image pipeline stays in its own venv.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             DisplayTrajectory, OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetMotionPlan
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

SERIAL = "Rizon4s-000000"  # Matches the mock-only launch in rviz.sh.
PREFIX = SERIAL + "_"
BASE, FLANGE, GROUP = PREFIX + "base_link", PREFIX + "flange", PREFIX + "arm"


def pose(values=None):
    result = Pose()
    if values is None:
        result.orientation.w = 1.0
    else:
        result.position.x, result.position.y, result.position.z = values[:3]
        result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = values[3:]
    return result


class Preview(Node):
    def __init__(self, job):
        super().__init__("tactile_preview")
        self.job, self.joints = job, None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(MarkerArray, "/tactile/path", qos)
        self.plan_pub = self.create_publisher(DisplayTrajectory, "/display_planned_path", qos)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.markers, self.display, self.plan_subscribers = None, None, 0
        self.create_timer(1.0, self.refresh_displays)

    def refresh_displays(self):
        # RViz may subscribe after planning; volatile subscribers miss old messages.
        if self.markers is not None:
            self.path_pub.publish(self.markers)
        subscribers = self.plan_pub.get_subscription_count()
        if self.display is not None and subscribers > self.plan_subscribers:
            self.plan_pub.publish(self.display)
        self.plan_subscribers = subscribers

    def on_joints(self, message):
        if all(PREFIX + f"joint{i}" in message.name for i in range(1, 8)):
            self.joints = message

    def call(self, service_type, name, request, timeout=60):
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"MoveIt service unavailable: {name}")
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"MoveIt service did not return: {name}")
            return future.result()
        finally:
            self.destroy_client(client)

    def load_scene(self):
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        tool = AttachedCollisionObject()
        tool.link_name = FLANGE
        tool.touch_links = [FLANGE, PREFIX + "link7"]
        tool.object.id, tool.object.header.frame_id = "tactile/tool", FLANGE
        tool.object.operation = CollisionObject.ADD
        for part in self.job["scene"]:
            obj = CollisionObject()
            obj.id = "tactile/" + part["id"]
            obj.header.frame_id = FLANGE if part["frame"] == "flange" else BASE
            mesh = Mesh()
            mesh.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in part["vertices"]]
            mesh.triangles = [MeshTriangle(vertex_indices=list(face)) for face in part["triangles"]]
            obj.meshes, obj.mesh_poses, obj.operation = [mesh], [pose()], CollisionObject.ADD
            if part["frame"] == "flange":
                # One rigid tool assembly; don't collision-check its touching
                # subparts against one another.
                tool.object.meshes.append(mesh)
                tool.object.mesh_poses.append(pose())
            else:
                request.scene.world.collision_objects.append(obj)
        request.scene.robot_state.attached_collision_objects.append(tool)
        if not self.call(ApplyPlanningScene, "/apply_planning_scene", request).success:
            raise RuntimeError("MoveIt rejected the scene")

    def show_path(self):
        markers = MarkerArray()
        reset = Marker()
        reset.action = Marker.DELETEALL
        markers.markers.append(reset)
        for index, segment in enumerate(self.job["segments"]):
            marker = Marker()
            marker.header.frame_id = BASE
            marker.ns, marker.id = "tactile_path", index
            marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.001 if segment["kind"] == "DRAW" else 0.0004
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                (0.3, 0.5, 0.9, 1.) if segment["kind"] == "DRAW" else (0.6, 0.65, 0.7, 0.6))
            marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in segment["tcp_points_m"]]
            markers.markers.append(marker)
        self.markers = markers
        self.path_pub.publish(markers)

    def plan(self):
        deadline = time.monotonic() + 60
        while self.joints is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.joints is None:
            raise RuntimeError("No Rizon4s mock joint state received")
        waypoints = [pose(p) for segment in self.job["segments"] for p in segment["flange_poses"]]
        # MoveIt plans the approach in joint space, then follows the curved
        # drawing/lift waypoints with collision-aware Cartesian interpolation.
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = GROUP
        motion.start_state.joint_state = copy.deepcopy(self.joints)
        motion.start_state.is_diff = True
        motion.allowed_planning_time, motion.num_planning_attempts = 5.0, 3
        motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.1
        position = PositionConstraint()
        position.header.frame_id, position.link_name, position.weight = BASE, FLANGE, 1.0
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.0001])
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [waypoints[0]]
        orientation = OrientationConstraint()
        orientation.header.frame_id, orientation.link_name, orientation.weight = BASE, FLANGE, 1.0
        orientation.orientation = waypoints[0].orientation
        orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = 0.001
        constraint = Constraints(position_constraints=[position], orientation_constraints=[orientation])
        motion.goal_constraints = [constraint]
        approach = self.call(GetMotionPlan, "/plan_kinematic_path", request).motion_plan_response
        if approach.error_code.val != 1 or not approach.trajectory.joint_trajectory.points:
            raise RuntimeError(f"Approach planning failed: MoveIt code {approach.error_code.val}")
        request = GetCartesianPath.Request()
        request.header.frame_id, request.group_name, request.link_name = BASE, GROUP, FLANGE
        request.start_state = copy.deepcopy(approach.trajectory_start)
        request.start_state.is_diff = True
        trajectory = approach.trajectory.joint_trajectory
        request.start_state.joint_state.name = list(trajectory.joint_names)
        request.start_state.joint_state.position = list(trajectory.points[-1].positions)
        request.start_state.joint_state.velocity = []
        request.start_state.joint_state.effort = []
        request.waypoints, request.max_step = waypoints, 0.001
        request.jump_threshold, request.avoid_collisions = 2.0, True
        # Optional fields differ between MoveIt message releases.
        if hasattr(request, "revolute_jump_threshold"):
            request.revolute_jump_threshold = 0.1
        for field in ("max_velocity_scaling_factor", "max_acceleration_scaling_factor"):
            if hasattr(request, field):
                setattr(request, field, 0.1)
        result = self.call(GetCartesianPath, "/compute_cartesian_path", request)
        if result.error_code.val != 1 or result.fraction < 1.0 - 1e-9:
            raise RuntimeError(f"Drawing path refused: {result.fraction:.1%} complete, MoveIt code {result.error_code.val}")
        if not result.solution.joint_trajectory.points:
            raise RuntimeError("MoveIt returned an empty drawing trajectory")
        display = DisplayTrajectory()
        display.trajectory_start = approach.trajectory_start
        display.trajectory = [approach.trajectory, result.solution]
        self.display = display
        self.plan_pub.publish(display)
        self.plan_subscribers = self.plan_pub.get_subscription_count()
        return {"status": "PLANNED", "fraction": result.fraction,
                "preview_only": True, "executed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("job", type=Path)
    args = parser.parse_args()
    job_bytes = args.job.read_bytes()
    job = json.loads(job_bytes)
    if (job.get("schema_version") != 1 or job.get("kind") != "tactile_ros_job"
            or job.get("robot") != "Rizon4s" or job.get("preview_only") is not True
            or job.get("units") != "m" or job.get("frame") != "robot_base"):
        parser.error("Expected a preview-only Rizon4s job in meters and robot-base coordinates")
    report_path = args.job.parent / "moveit_report.json"
    report_path.unlink(missing_ok=True)
    rclpy.init()
    node = Preview(job)
    try:
        try:
            node.load_scene()
            node.show_path()
            report = node.plan()
            node.get_logger().info("Full path planned. RViz playback only; no execution commands sent.")
        except RuntimeError as error:
            report = {"status": "REFUSED", "reason": str(error), "preview_only": True, "executed": False}
            node.get_logger().error(str(error))
        report["job_sha256"] = hashlib.sha256(job_bytes).hexdigest()
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        node.get_logger().info("To see pen waypoints: RViz Add → MarkerArray → /tactile/path")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
