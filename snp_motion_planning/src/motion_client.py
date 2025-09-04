#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml

from snp_msgs.srv import GenerateMotionPlan
from snp_msgs.msg import ToolPath
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


def create_dummy_tool_path() -> ToolPath:
    """Create a dummy ToolPath message for testing purposes."""
    pose1 = Pose()

    pose1.position.x = 0.1
    pose1.position.y = 0.5
    pose1.position.z = 0.3
    pose1.orientation.w = 1.0  # Neutral orientation

    pose2 = Pose()
    pose2.position.x = 1.0
    pose2.position.y = 0.5
    pose2.position.z = 0.3
    pose2.orientation.w = 1.0

    segment = PoseArray()
    segment.header.frame_id = "ur10e_base_link"
    segment.poses.append(pose1)
    segment.poses.append(pose2)

    tool_path = ToolPath()
    tool_path.segments.append(segment)

    return tool_path


def create_toolpath_from_yaml(file_path) -> ToolPath:
    """Create a ToolPath message from a YAML file."""
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)

    toolpath_segments = []

    for segment in data:
        pose_array = PoseArray()
        pose_array.header.frame_id = "composite_panel"

        for group in segment:
            for pose_data in group[0]:
                pose = Pose()
                pose.position.x = pose_data["x"]
                pose.position.y = pose_data["y"]
                pose.position.z = pose_data["z"]
                pose.orientation.x = pose_data["qx"]
                pose.orientation.y = pose_data["qy"]
                pose.orientation.z = pose_data["qz"]
                pose.orientation.w = pose_data["qw"]
                pose_array.poses.append(pose)

        toolpath_segments.append(pose_array)
    toolpath_msg = ToolPath()
    toolpath_msg.segments = toolpath_segments

    return toolpath_msg


class MotionClient(Node):
    """A ROS2 node for motion planning and robot control."""

    def __init__(self):
        super().__init__("motion_client")
        self.planner = self.create_client(GenerateMotionPlan, "/generate_motion_plan")
        self.controller = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

    def request_plan(self):
        """Request a motion plan from the motion planning service."""
        req = GenerateMotionPlan.Request()
        inspection_toolpath = create_toolpath_from_yaml(
            "/workspace/src/scan_n_plan_workshop/snp_motion_planning/config/toolpath.yaml"
        )
        # print(inspection_toolpath)
        req.tool_paths = [inspection_toolpath]
        req.motion_group = "manipulator"
        req.tcp_frame = "tcp"

        while not self.planner.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service /generate_motion_plan...")

        future = self.planner.call_async(req)
        return future

    def request_control(self, trajectory: JointTrajectory):
        """Request control of the robot's by sending a Joint trajectory to the action server"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.controller.wait_for_server()
        send_goal_future = self.controller.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        ctrl_result_future = self.request_control_result(send_goal_future)
        return ctrl_result_future

    def request_control_result(self, future):
        """Wait for the result from the action server."""

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        ctrl_result_future = goal_handle.get_result_async()
        ctrl_result_future.add_done_callback(self.get_result_callback)
        return ctrl_result_future

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.error_code}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.partial_sequence}")


def main(args=None):
    rclpy.init(args=args)

    client = MotionClient()
    client.get_logger().info("[plan] Requesting motion plan...")
    plan_future = client.request_plan()
    rclpy.spin_until_future_complete(client, plan_future)

    if plan_future.exception():
        client.get_logger().error(f"[plan] Request failed: {plan_future.exception()}")
        client.destroy_node()
        rclpy.shutdown()
        return

    response = plan_future.result()

    if not response or not getattr(response, "success", False):
        client.get_logger().error(f"[plan] Request failed: {plan_future.exception()}")
        client.destroy_node()
        rclpy.shutdown()
        return

    client.get_logger().info("[plan] Received motion plan successfully!")
    client.get_logger().info(f"[plan] Approach: {len(response.approach.points)} points")
    client.get_logger().info(f"[plan] Process: {len(response.process.points)} points")
    client.get_logger().info(f"[plan] Departure: {len(response.departure.points)} points")

    # remove effort values because scaled_joint_trajectory_controller does not support it.
    for pt in response.approach.points:
        pt.effort = []
    for pt in response.process.points:
        pt.effort = []
    for pt in response.departure.points:
        pt.effort = []

    client.get_logger().info("\n[exec] Starting Execution...")

    # Synchronus control
    approach_future = client.request_control(response.approach)
    rclpy.spin_until_future_complete(client, approach_future)
    process_future = client.request_control(response.process)
    rclpy.spin_until_future_complete(client, process_future)
    departure_future = client.request_control(response.departure)
    rclpy.spin_until_future_complete(client, departure_future)

    client.get_logger().info("[exec] Execution complete. Exiting.")
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
