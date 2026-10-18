#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0

        # Action client for 'waypoint_navigation'
        self.action_client = ActionClient(self, NavToWaypoint, 'waypoint_navigation')

        # Client for 'GetWaypoints' service
        self.cli = self.create_client(GetWaypoints, 'waypoints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = GetWaypoints.Request()

    ### action client functions

    def send_goal(self, waypoint):
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint.position.x = float(waypoint[0])
        goal_msg.waypoint.position.y = float(waypoint[1])
        goal_msg.waypoint.position.z = float(waypoint[2])
        goal_msg.hover = True

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        self.get_logger().info(f'Sending goal [{self.goal_index + 1}/{len(self.goals)}]: {waypoint}')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result (hov_time): {0}s'.format(result.hov_time))

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! The current whycon position is: {x:.2f}, {y:.2f}, {z:.2f}')
        self.get_logger().info(f'Max time inside sphere: {t}s')

    ### service client functions

    def send_request(self):
        self.req.get_waypoints = True
        return self.cli.call_async(self.req)

    def receive_goals(self):
        future = self.send_request()
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info('Waypoints received by the action client')

        for pose in response.waypoints.poses:
            waypoints = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoints)
            self.get_logger().info(f'Waypoints: {waypoints}')

        if self.goals:
            self.send_goal(self.goals[0])


def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
