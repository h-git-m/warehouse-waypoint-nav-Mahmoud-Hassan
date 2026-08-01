#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        # Create an action client for the /navigate_to_pose action
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, yaw=0.0):
        # Wait until the Nav2 action server is available
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()

        # Build the target pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Convert a yaw angle (radians) into a quaternion.
        # For a flat robot, only z and w are non-zero.
        import math
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')

        # Send the goal, register a feedback callback
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted, navigating...')

        # Ask for the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Feedback streams continuously while the robot drives
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f} m')

    def result_callback(self, future):
        # Called once when navigation finishes
        self.get_logger().info('Navigation finished')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GoalSender()

    # Change these numbers to move the robot somewhere else
    node.send_goal(x=1.5, y=0.5, yaw=0.0)

    rclpy.spin(node)


if __name__ == '__main__':
    main()
