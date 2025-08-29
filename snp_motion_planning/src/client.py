#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from snp_msgs.srv import GenerateMotionPlan  # Adjust to your package name
from snp_msgs.msg import ToolPath
from geometry_msgs.msg import PoseArray, Pose
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory


def create_dummy_tool_path():
    pose1 = Pose()

    pose1.position.x = 0.5
    pose1.position.y = 0.0
    pose1.position.z = 0.3
    pose1.orientation.w = 1.0  # Neutral orientation

    pose2 = Pose()
    pose2.position.x = 0.8
    pose2.position.y = 0.0
    pose2.position.z = 0.3
    pose2.orientation.w = 1.0

    segment = PoseArray()
    segment.header.frame_id = "ur10e_base_link"
    segment.poses.append(pose1)
    segment.poses.append(pose2)

    tool_path = ToolPath()
    tool_path.segments.append(segment)

    return tool_path

class MotionPlanClient(Node):
    def __init__(self):
        super().__init__('motion_plan_client')
        self.cli = self.create_client(GenerateMotionPlan, '/generate_motion_plan')

        self.controller = ActionClient(self, FollowJointTrajectory, '/scaled_joint_trajectory_controller/follow_joint_trajectory')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /generate_motion_plan...')
        
        self.req = GenerateMotionPlan.Request()

    def send_request(self):
        # Example: fill in tool paths (empty array here; fill as needed)
        self.req.tool_paths = [create_dummy_tool_path()]
        
        # Example: set motion group and TCP frame
        self.req.motion_group = 'manipulator'
        self.req.tcp_frame = 'tcp'

        self.future = self.cli.call_async(self.req)
        return self.future

    def send_goal(self, trajectory):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.controller.wait_for_server()
        self._send_goal_future = self.controller.send_goal_async(goal_msg)
        # self._send_goal_future.add_done_callback(self.goal_response_callback)
        rclpy.spin_until_future_complete(self, self._send_goal_future)

        self.ctrl_result_future = self.goal_response_callback(self._send_goal_future)
        rclpy.spin_until_future_complete(self, self.ctrl_result_future)
        # return self.ctrl_result_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(f"Goal rejected :(")
            return

        self.get_logger().info('Goal accepted :)')

        self.ctrl_result_future = goal_handle.get_result_async()
        self.ctrl_result_future.add_done_callback(self.get_result_callback)
        return self.ctrl_result_future

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.error_code))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

        
def main(args=None):
    rclpy.init(args=args)

    client = MotionPlanClient()
    try:
        plan_future  = client.send_request()
        rclpy.spin_until_future_complete(client, plan_future)

        response = plan_future.result()
        if response.success:
            client.get_logger().info('Received motion plan successfully!')
            # Example: print number of points in each trajectory
            client.get_logger().info(f'Approach: {len(response.approach.points)} points')
            client.get_logger().info(f'Process: {len(response.process.points)} points')
            # print(response.approach.points)
            client.get_logger().info(f'Departure: {len(response.departure.points)} points')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')

    # remove effort values because scaled_joint_trajectory_controller does not support it.
    for pt in response.approach.points:
     pt.effort=[]
    for pt in response.process.points:
     pt.effort=[]
    for pt in response.departure.points:
     pt.effort=[]

    client.send_goal(response.approach)
    client.send_goal(response.process)
    client.send_goal(response.departure)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
