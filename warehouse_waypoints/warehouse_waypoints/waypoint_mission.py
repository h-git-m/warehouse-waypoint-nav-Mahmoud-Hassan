#!/usr/bin/env python3
"""
warehouse_waypoints / waypoint_mission.py

Runs the required autonomous warehouse mission:
    Home -> Loading (wait 30s) -> Storage -> Shipping -> Home

Publishes a visualization_msgs/MarkerArray on /waypoint_markers showing
all four named locations. The currently active Nav2 goal is shown in
GREEN; all other waypoints stay BLUE. The array is republished every
time the active goal changes.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose


WAYPOINTS = {
    'home': {
        'label': 'Charging Station (Home)',
        'x': 0.0, 'y': 0.0, 'z': 0.0,
        'qz': 0.0, 'qw': 1.0,
    },
    'loading': {
        'label': 'Loading Station',
        'x': 7.782053842839245, 'y': 7.420772652392393, 'z': 0.0,
        'qz': 0.0, 'qw': 1.0,
    },
    'storage': {
        'label': 'Storage Area',
        'x': 20.000934662165207, 'y': -2.415622611057596, 'z': 0.0,
        'qz': 0.0, 'qw': 1.0,
    },
    'shipping': {
        'label': 'Shipping Station',
        'x': -5.930928476565102, 'y': -5.406362903539103, 'z': 0.0,
        'qz': 0.0, 'qw': 1.0,
    },
}

# Order and marker indices are dict-insertion order above: home, loading,
# storage, shipping. This fixed order is reused both for marker IDs and
# for iterating "all waypoints" when publishing the marker array.
WAYPOINT_ORDER = ['home', 'loading', 'storage', 'shipping']

# Required mission sequence (Home is the start; not itself a "goal").
MISSION_SEQUENCE = ['loading', 'storage', 'shipping', 'home']

# Wait times applied *after* successfully reaching a given waypoint.
WAIT_AFTER_ARRIVAL = {
    'loading': 30.0,
}

BLUE = (0.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0)
MARKER_FRAME = 'map'


class WaypointMission(Node):

    def __init__(self):
        super().__init__('waypoint_mission')

        self.marker_pub = self.create_publisher(
            MarkerArray, 'waypoint_markers', 10)

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._mission_index = 0
        self._active_name = None

    # ------------------------------------------------------------------
    # Marker publishing
    # ------------------------------------------------------------------
    def make_pose_stamped(self, name: str) -> PoseStamped:
        wp = WAYPOINTS[name]
        pose = PoseStamped()
        pose.header.frame_id = MARKER_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp['x']
        pose.pose.position.y = wp['y']
        pose.pose.position.z = wp['z']
        pose.pose.orientation.z = wp['qz']
        pose.pose.orientation.w = wp['qw']
        return pose

    def publish_markers(self, active_name: str | None):
        """Publish sphere + text markers for every waypoint. The waypoint
        matching active_name is GREEN, all others are BLUE."""
        self._active_name = active_name
        array = MarkerArray()
        marker_id = 0
        now = self.get_clock().now().to_msg()

        for name in WAYPOINT_ORDER:
            wp = WAYPOINTS[name]
            color = GREEN if name == active_name else BLUE

            sphere = Marker()
            sphere.header.frame_id = MARKER_FRAME
            sphere.header.stamp = now
            sphere.ns = 'waypoints'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = 0.15
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 1.0
            array.markers.append(sphere)

            label = Marker()
            label.header.frame_id = MARKER_FRAME
            label.header.stamp = now
            label.ns = 'waypoint_labels'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = wp['x']
            label.pose.position.y = wp['y']
            label.pose.position.z = 0.7
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 1.0
            label.text = wp['label']
            array.markers.append(label)

        self.marker_pub.publish(array)
        self.get_logger().info(
            f'Published waypoint markers (active goal: {active_name})')

    # ------------------------------------------------------------------
    # Mission sequencing
    # ------------------------------------------------------------------
    def publish_initial_markers(self):
        """Show all four waypoints (blue, none active) immediately on
        startup, before the mission has been triggered."""
        self.publish_markers(active_name=None)

    def begin_mission(self):
        """Called once the operator types 'start' in the terminal.
        Waits for the Nav2 action server, then sends the first goal."""
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server available. Starting mission.')
        self._send_next_goal()

    def _send_next_goal(self):
        if self._mission_index >= len(MISSION_SEQUENCE):
            self.get_logger().info('Mission complete. Robot is home.')
            self.publish_markers(active_name=None)
            rclpy.shutdown()
            return

        name = MISSION_SEQUENCE[self._mission_index]
        self.get_logger().info(f'Navigating to: {WAYPOINTS[name]["label"]}')
        self.publish_markers(active_name=name)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(name)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_goal_future.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        # Optional: log distance remaining periodically. Kept quiet by
        # default to avoid flooding the terminal.
        pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        name = MISSION_SEQUENCE[self._mission_index]

        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal to {WAYPOINTS[name]["label"]} was REJECTED by Nav2. '
                f'Stopping mission at this location.')
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future):
        status = future.result().status
        name = MISSION_SEQUENCE[self._mission_index]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached: {WAYPOINTS[name]["label"]}')

            wait_s = WAIT_AFTER_ARRIVAL.get(name)
            if wait_s:
                self.get_logger().info(
                    f'Waiting {wait_s:.0f} seconds at '
                    f'{WAYPOINTS[name]["label"]}...')
                time.sleep(wait_s)
                self.get_logger().info('Wait complete. Continuing mission.')

            self._mission_index += 1
            self._send_next_goal()
        else:
            self.get_logger().error(
                f'FAILED to reach {WAYPOINTS[name]["label"]} '
                f'(status code: {status}). Stopping mission and reporting '
                f'location.')
            self.get_logger().error(
                f'Last attempted location: {WAYPOINTS[name]["label"]} '
                f'at x={WAYPOINTS[name]["x"]:.2f}, '
                f'y={WAYPOINTS[name]["y"]:.2f}')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()

    # Spin on a background thread so /waypoint_markers keeps being
    # servable, and Nav2 goal callbacks keep being processed, while the
    # main thread blocks on terminal input below.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Markers appear immediately -- before any goal is sent.
    node.publish_initial_markers()

    try:
        print('\nWaypoint markers published on /waypoint_markers.')
        print("Type 'start' and press Enter to begin the mission.\n")
        while rclpy.ok():
            user_input = input().strip().lower()
            if user_input == 'start':
                node.begin_mission()
                break
            else:
                print("Not recognized. Type 'start' to begin the mission.")

        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
