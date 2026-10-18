#!/usr/bin/env python3

import time
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from std_msgs.msg import Header
from whycode_interfaces.msg import MarkerArray
from error_msg.msg import Error


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0.0
        self.max_time_inside_sphere = 0.0
        self.point_in_sphere_start_time = None
        self.duration = 0.0

        self.yaw = 0.0              # current yaw in degrees (updated from whycode)
        self.desired_yaw = -0.5     # target yaw angle (degrees)
        self.curr_state = [0.0, 0.0, 13.0]
        self.desired_state = [-2.41, -2.41, 13.0]
        self.drone_position = [0.0, 0.0, 13.0]
        self.prev_drone_position = [0.0, 0.0, 13.0]

        self.actual_vel_x = 0.0
        self.actual_vel_y = 0.0
        self.actual_vel_z = 0.0

        self.dtime = 0.0
        self.first_marker_received = False

        # Declaring cmd message and initializing
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 1500

        # Cascaded PID parameters (from validated Task 2A pico_controller_PID.cpp)
        # Lateral position outer loop (WhyCode Y -> Roll, WhyCode X -> Pitch)
        self.Kp_pos = [66.0, 66.0]
        self.Ki_pos = [0.016, 0.016]
        self.Kd_pos = [90.0, 90.0]
        self.pos_error_sum = [0.0, 0.0]
        self.prev_pos_error = [0.0, 0.0]
        self.max_horizontal_vel = 1.5

        # Lateral velocity inner loop
        self.Kp_vel = [25.0, 25.0]
        self.Ki_vel = [0.0, 0.0]
        self.Kd_vel = [0.0, 0.0]
        self.vel_error_sum = [0.0, 0.0]
        self.prev_vel_error = [0.0, 0.0]

        # Altitude outer loop (position to vertical velocity)
        self.Kp_pos_z = 3.0
        self.Ki_pos_z = 0.045
        self.Kd_pos_z = 2.2
        self.pos_z_error_sum = 0.0
        self.prev_pos_z_error = 0.0
        self.max_vertical_vel = 5.0

        # Altitude inner loop (velocity to throttle)
        self.Kp_vel_z = 22.0
        self.Ki_vel_z = 2.0
        self.Kd_vel_z = 0.0
        self.vel_z_error_sum = 0.0
        self.prev_vel_z_error = 0.0

        self.windup_guard = 40.0
        self.min_values = [1000, 1000, 1000]
        self.max_values = [2000, 2000, 2000]

        self.sample_time = 0.033
        self.dt = self.sample_time

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)
        self.heartbeat_pub = self.create_publisher(Header, '/controller_heartbeat', 10)

        # Subscribers
        self.create_subscription(
            MarkerArray,
            '/whycode_node/markers',
            self.whycode_callback,
            1,
            callback_group=self.pid_callback_group
        )

        # Action Server
        self.action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            execute_callback=self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.publish_heartbeat()
        self.create_timer(0.1, self.publish_heartbeat)

        time.sleep(0.5)
        self.arm()

        # PID timer callback running at sample_time (33ms)
        self.timer = self.create_timer(
            self.sample_time,
            self.pid,
            callback_group=self.pid_callback_group
        )

    def publish_heartbeat(self):
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = 'pico_controller_active'
        self.heartbeat_pub.publish(hb)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycode_callback(self, msg):
        if not msg.markers:
            return

        marker = msg.markers[0]
        position = marker.position.position
        self.curr_state = [position.x, position.y, position.z]
        self.drone_position = self.curr_state[:]

        if not self.first_marker_received:
            self.prev_drone_position = self.curr_state[:]
            self.first_marker_received = True

        # Marker rotation z converted to degrees
        self.yaw = math.degrees(marker.rotation.z)
        self.dtime = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def pid(self):
        if not self.first_marker_received:
            return

        # Position error in marker coordinates
        err_roll = self.desired_state[1] - self.drone_position[1]     # index 0: Roll (Y axis)
        err_pitch = self.desired_state[0] - self.drone_position[0]    # index 1: Pitch (X axis)
        err_throttle = self.desired_state[2] - self.drone_position[2] # index 2: Throttle (Z axis)

        # Publish /pos_error
        err_msg = Error()
        err_msg.roll_error = float(err_roll)
        err_msg.pitch_error = float(err_pitch)
        err_msg.throttle_error = float(err_throttle)
        self.pos_error_pub.publish(err_msg)

        # Raw velocities
        raw_vel_x = (self.drone_position[0] - self.prev_drone_position[0]) / self.dt
        raw_vel_y = (self.drone_position[1] - self.prev_drone_position[1]) / self.dt
        self.prev_drone_position[0] = self.drone_position[0]
        self.prev_drone_position[1] = self.drone_position[1]

        # Low-pass EMA filter for lateral velocity estimates
        self.actual_vel_x = 0.35 * raw_vel_x + 0.65 * self.actual_vel_x
        self.actual_vel_y = 0.35 * raw_vel_y + 0.65 * self.actual_vel_y

        actual_vel_lat = [self.actual_vel_y, self.actual_vel_x]
        errors = [err_roll, err_pitch]

        out = [0.0, 0.0]
        for i in range(2):
            # Outer loop: position error to desired velocity
            # Conditional integration: only accumulate integral near setpoint to prevent overshoot
            if abs(errors[i]) < 0.6:
                self.pos_error_sum[i] += errors[i] * self.dt
                self.pos_error_sum[i] = clamp(self.pos_error_sum[i], -self.windup_guard, self.windup_guard)
            else:
                self.pos_error_sum[i] = 0.0

            pos_deriv = (errors[i] - self.prev_pos_error[i]) / self.dt
            self.prev_pos_error[i] = errors[i]

            desired_vel = (self.Kp_pos[i] * errors[i]) + (self.Ki_pos[i] * self.pos_error_sum[i]) + (self.Kd_pos[i] * pos_deriv)
            desired_vel = clamp(desired_vel, -self.max_horizontal_vel, self.max_horizontal_vel)

            # Inner loop: velocity error to control command
            vel_err = desired_vel - actual_vel_lat[i]
            self.vel_error_sum[i] += vel_err * self.dt
            self.vel_error_sum[i] = clamp(self.vel_error_sum[i], -self.windup_guard, self.windup_guard)

            vel_deriv = (vel_err - self.prev_vel_error[i]) / self.dt
            self.prev_vel_error[i] = vel_err

            out[i] = (self.Kp_vel[i] * vel_err) + (self.Ki_vel[i] * self.vel_error_sum[i]) + (self.Kd_vel[i] * vel_deriv)

        # Altitude cascaded control
        raw_vel_z = (self.drone_position[2] - self.prev_drone_position[2]) / self.dt
        self.prev_drone_position[2] = self.drone_position[2]
        self.actual_vel_z = 0.25 * raw_vel_z + 0.75 * self.actual_vel_z

        if abs(err_throttle) < 1.0:
            self.pos_z_error_sum += err_throttle * self.dt
            self.pos_z_error_sum = clamp(self.pos_z_error_sum, -self.windup_guard, self.windup_guard)
        else:
            self.pos_z_error_sum = 0.0

        pos_z_deriv = (err_throttle - self.prev_pos_z_error) / self.dt
        self.prev_pos_z_error = err_throttle

        desired_vel_z = (self.Kp_pos_z * err_throttle) + (self.Ki_pos_z * self.pos_z_error_sum) + (self.Kd_pos_z * pos_z_deriv)
        desired_vel_z = clamp(desired_vel_z, -self.max_vertical_vel, self.max_vertical_vel)

        vel_z_err = desired_vel_z - self.actual_vel_z
        self.vel_z_error_sum += vel_z_err * self.dt
        self.vel_z_error_sum = clamp(self.vel_z_error_sum, -self.windup_guard, self.windup_guard)

        vel_z_deriv = (vel_z_err - self.prev_vel_z_error) / self.dt
        self.prev_vel_z_error = vel_z_err

        out_throttle = (self.Kp_vel_z * vel_z_err) + (self.Ki_vel_z * self.vel_z_error_sum) + (self.Kd_vel_z * vel_z_deriv)

        # Yaw control: Hold rc_yaw = 1500 to keep yaw rate at 0.0 in the simulator interface (prevents bang-bang 45 deg/s spin)
        target_yaw = 1500

        # Command mapping
        target_roll = int(1500.0 + out[0])
        target_pitch = int(1500.0 + out[1])
        target_throttle = int(1500.0 - out_throttle)

        self.cmd.rc_roll = int(clamp(target_roll, self.min_values[0], self.max_values[0]))
        self.cmd.rc_pitch = int(clamp(target_pitch, self.min_values[1], self.max_values[1]))
        self.cmd.rc_throttle = int(clamp(target_throttle, self.min_values[2], self.max_values[2]))
        self.cmd.rc_yaw = target_yaw
        self.cmd.rc_aux4 = 2000

        self.command_pub.publish(self.cmd)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.desired_state[0] = goal_handle.request.waypoint.position.x
        self.desired_state[1] = goal_handle.request.waypoint.position.y
        self.desired_state[2] = goal_handle.request.waypoint.position.z

        # Reset cascaded loop integrals and errors to ensure clean transition without integral carryover
        self.pos_error_sum = [0.0, 0.0]
        self.vel_error_sum = [0.0, 0.0]
        self.pos_z_error_sum = 0.0
        self.vel_z_error_sum = 0.0
        self.prev_pos_error = [
            self.desired_state[1] - self.drone_position[1],
            self.desired_state[0] - self.drone_position[0]
        ]
        self.prev_pos_z_error = self.desired_state[2] - self.drone_position[2]
        self.prev_vel_error = [0.0, 0.0]
        self.prev_vel_z_error = 0.0

        self.get_logger().info(f'New Waypoint Set: {self.desired_state}')
        self.max_time_inside_sphere = 0.0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0.0
        self.duration = self.dtime

        feedback_msg = NavToWaypoint.Feedback()

        while rclpy.ok():
            feedback_msg.current_waypoint.pose.position.x = float(self.curr_state[0])
            feedback_msg.current_waypoint.pose.position.y = float(self.curr_state[1])
            feedback_msg.current_waypoint.pose.position.z = float(self.curr_state[2])
            feedback_msg.current_waypoint.header.stamp.sec = int(self.time_inside_sphere)

            goal_handle.publish_feedback(feedback_msg)

            # Check if drone is holding within ±0.4 whycode units on all 3 axes
            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4)

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                pass
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                self.point_in_sphere_start_time = self.dtime
                self.get_logger().info('Drone inside tolerance for 1st time')
            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.get_logger().info('Drone drifted out of tolerance')
                self.point_in_sphere_start_time = None
                self.time_inside_sphere = 0.0

            if self.time_inside_sphere > self.max_time_inside_sphere:
                self.max_time_inside_sphere = self.time_inside_sphere

            if self.time_inside_sphere >= 3.0:
                self.get_logger().info('Hovered inside tolerance for 3 seconds! Waypoint reached.')
                break

            time.sleep(0.05)

        goal_handle.succeed()

        result = NavToWaypoint.Result()
        result.hov_time = int(round(self.dtime - self.duration))
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius=0.4):
        # ±0.4 whycode units on every axis (X, Y, and Z)
        dx = abs(drone_pos[0] - sphere_center.request.waypoint.position.x)
        dy = abs(drone_pos[1] - sphere_center.request.waypoint.position.y)
        dz = abs(drone_pos[2] - sphere_center.request.waypoint.position.z)
        return dx <= radius and dy <= radius and dz <= radius


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
