#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class CorridorCentering(object):
    def __init__(self):
        rospy.init_node('corridor_centering')

        self.side_min_angle = math.radians(
            rospy.get_param('~side_min_angle_deg', 70.0))
        self.side_max_angle = math.radians(
            rospy.get_param('~side_max_angle_deg', 110.0))
        self.side_min_distance = rospy.get_param(
            '~side_min_distance', 0.35)
        self.side_max_distance = rospy.get_param(
            '~side_max_distance', 2.30)
        self.corridor_min_width = rospy.get_param(
            '~corridor_min_width', 2.20)
        self.corridor_max_width = rospy.get_param(
            '~corridor_max_width', 2.50)
        self.expected_corridor_width = rospy.get_param(
            '~expected_corridor_width', 2.36)
        self.corridor_width_tolerance = rospy.get_param(
            '~corridor_width_tolerance', 0.15)
        self.acquisition_width_spread = rospy.get_param(
            '~acquisition_width_spread', 0.10)
        self.wall_flatness_tolerance = rospy.get_param(
            '~wall_flatness_tolerance', 0.10)
        self.normal_confirmation_samples = int(rospy.get_param(
            '~normal_confirmation_samples', 8))
        self.single_wall_match_tolerance = rospy.get_param(
            '~single_wall_match_tolerance', 0.30)
        self.single_wall_selection_margin = rospy.get_param(
            '~single_wall_selection_margin', 0.03)
        self.width_learning_alpha = rospy.get_param(
            '~width_learning_alpha', 0.02)
        self.minimum_samples = int(rospy.get_param(
            '~minimum_samples', 8))
        self.minimum_forward_speed = rospy.get_param(
            '~minimum_forward_speed', 0.04)
        self.maximum_input_angular_speed = rospy.get_param(
            '~maximum_input_angular_speed', 0.08)
        self.maximum_output_angular_speed = rospy.get_param(
            '~maximum_output_angular_speed', 0.10)
        self.centering_gain = rospy.get_param(
            '~centering_gain', 0.22)
        self.centering_deadband = rospy.get_param(
            '~centering_deadband', 0.04)
        self.maximum_correction = rospy.get_param(
            '~maximum_correction', 0.10)
        self.large_center_error = rospy.get_param(
            '~large_center_error', 0.20)
        self.large_error_linear_speed = rospy.get_param(
            '~large_error_linear_speed', 0.05)
        self.centering_priority_error = max(
            0.01,
            rospy.get_param('~centering_priority_error', 0.30))
        self.minimum_path_tracking_weight = self.clamp(
            rospy.get_param('~minimum_path_tracking_weight', 0.25),
            0.0,
            1.0)
        self.distance_filter_alpha = rospy.get_param(
            '~distance_filter_alpha', 0.35)
        self.scan_timeout = rospy.get_param(
            '~scan_timeout', 0.50)
        self.minimum_edge_clearance = rospy.get_param(
            '~minimum_edge_clearance', 0.25)
        self.near_wall_linear_speed = rospy.get_param(
            '~near_wall_linear_speed', 0.04)
        self.near_wall_angular_speed = rospy.get_param(
            '~near_wall_angular_speed', 0.08)

        self.robot_front_from_lidar = rospy.get_param(
            '~robot_front_from_lidar', 0.11)
        self.robot_rear_from_lidar = rospy.get_param(
            '~robot_rear_from_lidar', 0.52)
        self.robot_half_width = rospy.get_param(
            '~robot_half_width', 0.385)
        self.self_filter_margin = rospy.get_param(
            '~self_filter_margin', 0.03)

        self.lock = threading.Lock()
        self.left_distance = None
        self.right_distance = None
        self.safety_left_distance = None
        self.safety_right_distance = None
        self.current_corridor_width = None
        self.nominal_corridor_width = self.expected_corridor_width
        self.normal_confirmation_count = 0
        self.normal_width_samples = []
        self.normal_corridor_seen = False
        self.wall_mode = 'none'
        self.last_scan_wall_time = None

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback, queue_size=1)
        self.cmd_sub = rospy.Subscriber(
            '/cmd_vel_nav', Twist, self.cmd_callback, queue_size=10)
        self.reset_sub = rospy.Subscriber(
            '/corridor_centering/reset', Empty, self.reset_callback,
            queue_size=1)

        rospy.loginfo(
            "corridor_centering: side sectors %.0f..%.0f deg, "
            "width %.2f..%.2f m, gain %.2f",
            math.degrees(self.side_min_angle),
            math.degrees(self.side_max_angle),
            self.corridor_min_width,
            self.corridor_max_width,
            self.centering_gain)

    def reset_callback(self, _msg):
        with self.lock:
            self.left_distance = None
            self.right_distance = None
            self.safety_left_distance = None
            self.safety_right_distance = None
            self.current_corridor_width = None
            self.wall_mode = 'none'
            self.last_scan_wall_time = None
        rospy.loginfo(
            "corridor_centering: cleared direction-dependent wall state")

    @staticmethod
    def median(values):
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) * 0.5

    @staticmethod
    def central_spread(values):
        ordered = sorted(values)
        low_index = int(round((len(ordered) - 1) * 0.10))
        high_index = int(round((len(ordered) - 1) * 0.90))
        return ordered[high_index] - ordered[low_index]

    @staticmethod
    def clamp(value, minimum, maximum):
        return min(maximum, max(minimum, value))

    def is_robot_self_return(self, measured_range, angle):
        point_x = measured_range * math.cos(angle)
        point_y = measured_range * math.sin(angle)
        return (
            -self.robot_rear_from_lidar - self.self_filter_margin <=
            point_x <=
            self.robot_front_from_lidar + self.self_filter_margin and
            abs(point_y) <=
            self.robot_half_width + self.self_filter_margin)

    def filtered_distance(self, previous, measured):
        if previous is None:
            return measured
        alpha = self.distance_filter_alpha
        return alpha * measured + (1.0 - alpha) * previous

    def scan_callback(self, msg):
        left_samples = []
        right_samples = []
        left_safety_samples = []
        right_safety_samples = []
        angle = msg.angle_min

        for measured_range in msg.ranges:
            if (not math.isnan(measured_range) and
                    not math.isinf(measured_range) and
                    measured_range >= msg.range_min and
                    measured_range <= msg.range_max and
                    not self.is_robot_self_return(measured_range, angle)):
                absolute_angle = abs(angle)
                if (self.side_min_angle <= absolute_angle <=
                        self.side_max_angle):
                    lateral_distance = (
                        measured_range * abs(math.sin(angle)))
                    if (0.05 <= lateral_distance <=
                            self.side_max_distance):
                        if angle > 0.0:
                            left_safety_samples.append(lateral_distance)
                        else:
                            right_safety_samples.append(lateral_distance)
                    if (self.side_min_distance <= lateral_distance <=
                            self.side_max_distance):
                        if angle > 0.0:
                            left_samples.append(lateral_distance)
                        else:
                            right_samples.append(lateral_distance)
            angle += msg.angle_increment

        left = None
        right = None
        if len(left_samples) >= self.minimum_samples:
            left = self.median(left_samples)
        if len(right_samples) >= self.minimum_samples:
            right = self.median(right_samples)
        left_safety = None
        right_safety = None
        if len(left_safety_samples) >= self.minimum_samples:
            left_safety = self.median(left_safety_samples)
        if len(right_safety_samples) >= self.minimum_samples:
            right_safety = self.median(right_safety_samples)

        left_flat = (
            left is not None and
            self.central_spread(left_samples) <=
            self.wall_flatness_tolerance)
        right_flat = (
            right is not None and
            self.central_spread(right_samples) <=
            self.wall_flatness_tolerance)

        with self.lock:
            nominal_width = self.nominal_corridor_width
            current_width = None
            if left is not None and right is not None:
                current_width = left + right

            self.safety_left_distance = left_safety
            self.safety_right_distance = right_safety
            self.current_corridor_width = current_width
            self.wall_mode = 'none'
            self.last_scan_wall_time = time.time()

            corridor_width_in_range = (
                current_width is not None and
                self.corridor_min_width <= current_width <=
                self.corridor_max_width)
            normal_geometry = (
                corridor_width_in_range and
                abs(current_width - nominal_width) <=
                self.corridor_width_tolerance)

            if not self.normal_corridor_seen:
                acquisition_geometry = (
                    corridor_width_in_range and
                    left_flat and
                    right_flat)
                if acquisition_geometry:
                    self.normal_width_samples.append(current_width)
                    if (max(self.normal_width_samples) -
                            min(self.normal_width_samples) <=
                            self.acquisition_width_spread):
                        self.normal_confirmation_count = len(
                            self.normal_width_samples)
                    else:
                        self.normal_width_samples = [current_width]
                        self.normal_confirmation_count = 1
                else:
                    self.normal_confirmation_count = 0
                    self.normal_width_samples = []

                if (self.normal_confirmation_count >=
                        self.normal_confirmation_samples):
                    self.normal_corridor_seen = True
                    self.nominal_corridor_width = self.median(
                        self.normal_width_samples)
                    self.left_distance = left
                    self.right_distance = right
                    self.wall_mode = 'both'
                    rospy.loginfo(
                        "corridor_centering acquired normal corridor: "
                        "left %.3f right %.3f width %.3f",
                        left,
                        right,
                        self.nominal_corridor_width)
                return

            if normal_geometry:
                self.left_distance = self.filtered_distance(
                    self.left_distance, left)
                self.right_distance = self.filtered_distance(
                    self.right_distance, right)
                self.wall_mode = 'both'
                if (abs(current_width - nominal_width) <=
                        self.corridor_width_tolerance * 0.5):
                    alpha = self.width_learning_alpha
                    self.nominal_corridor_width = (
                        alpha * current_width +
                        (1.0 - alpha) * nominal_width)
                return

            opening_geometry = (
                (left is None) != (right is None) or
                (current_width is not None and
                 (current_width > self.corridor_max_width or
                  current_width > nominal_width +
                  self.corridor_width_tolerance)))
            if not opening_geometry:
                return

            wall_candidates = []
            if left is not None and self.left_distance is not None:
                left_change = abs(left - self.left_distance)
                if left_change <= self.single_wall_match_tolerance:
                    wall_candidates.append(('left', left_change))
            if right is not None and self.right_distance is not None:
                right_change = abs(right - self.right_distance)
                if right_change <= self.single_wall_match_tolerance:
                    wall_candidates.append(('right', right_change))

            if not wall_candidates:
                return

            wall_candidates.sort(key=lambda item: item[1])
            if (len(wall_candidates) > 1 and
                    wall_candidates[1][1] - wall_candidates[0][1] <
                    self.single_wall_selection_margin):
                return

            intact_side = wall_candidates[0][0]
            if intact_side == 'left':
                self.left_distance = self.filtered_distance(
                    self.left_distance, left)
                self.wall_mode = 'left'
            else:
                self.right_distance = self.filtered_distance(
                    self.right_distance, right)
                self.wall_mode = 'right'

    def copy_command(self, source):
        command = Twist()
        command.linear.x = source.linear.x
        command.linear.y = source.linear.y
        command.linear.z = source.linear.z
        command.angular.x = source.angular.x
        command.angular.y = source.angular.y
        command.angular.z = source.angular.z
        return command

    def cmd_callback(self, msg):
        command = self.copy_command(msg)
        now = time.time()
        with self.lock:
            left = self.left_distance
            right = self.right_distance
            safety_left = self.safety_left_distance
            safety_right = self.safety_right_distance
            current_width = self.current_corridor_width
            nominal_width = self.nominal_corridor_width
            wall_mode = self.wall_mode
            normal_corridor_seen = self.normal_corridor_seen
            confirmation_count = self.normal_confirmation_count
            scan_time = self.last_scan_wall_time

        scan_is_fresh = (
            scan_time is not None and
            now - scan_time <= self.scan_timeout)
        minimum_lidar_wall_distance = (
            self.robot_half_width + self.minimum_edge_clearance)
        left_too_close = (
            safety_left is not None and
            safety_left < minimum_lidar_wall_distance)
        right_too_close = (
            safety_right is not None and
            safety_right < minimum_lidar_wall_distance)

        if (msg.linear.x >= self.minimum_forward_speed and
                scan_is_fresh and
                (left_too_close or right_too_close)):
            if left_too_close and right_too_close:
                command.linear.x = 0.0
                command.angular.z = 0.0
                action = 'stop'
            elif left_too_close:
                command.linear.x = min(
                    command.linear.x, self.near_wall_linear_speed)
                command.angular.z = -self.near_wall_angular_speed
                action = 'steer right'
            else:
                command.linear.x = min(
                    command.linear.x, self.near_wall_linear_speed)
                command.angular.z = self.near_wall_angular_speed
                action = 'steer left'
            rospy.logwarn_throttle(
                1.0,
                "corridor_centering wall guard: left %s right %s, %s",
                ("%.3f" % safety_left
                 if safety_left is not None else "open"),
                ("%.3f" % safety_right
                 if safety_right is not None else "open"),
                action)
            self.cmd_pub.publish(command)
            return

        can_center = (
            msg.linear.x >= self.minimum_forward_speed and
            abs(msg.angular.z) <=
            self.maximum_input_angular_speed + 1e-6 and
            normal_corridor_seen and
            wall_mode != 'none' and
            scan_is_fresh)

        if can_center:
            if wall_mode == 'both':
                center_error = left - right
            elif wall_mode == 'left':
                center_error = 2.0 * left - nominal_width
            else:
                center_error = nominal_width - 2.0 * right

            if abs(center_error) <= self.centering_deadband + 1e-6:
                correction = 0.0
            else:
                effective_error = math.copysign(
                    abs(center_error) - self.centering_deadband,
                    center_error)
                correction = self.clamp(
                    self.centering_gain * effective_error,
                    -self.maximum_correction,
                    self.maximum_correction)
            centering_priority = self.clamp(
                abs(center_error) / self.centering_priority_error,
                0.0,
                1.0)
            path_tracking_weight = max(
                self.minimum_path_tracking_weight,
                1.0 - centering_priority)
            command.angular.z = self.clamp(
                correction + path_tracking_weight * msg.angular.z,
                -self.maximum_output_angular_speed,
                self.maximum_output_angular_speed)
            if abs(center_error) >= self.large_center_error:
                command.linear.x = min(
                    command.linear.x,
                    self.large_error_linear_speed)
            if wall_mode == 'both':
                left_text = "%.3f" % left
                right_text = "%.3f" % right
            elif wall_mode == 'left':
                left_text = "%.3f" % left
                right_text = "open"
            else:
                left_text = "open"
                right_text = "%.3f" % right
            rospy.loginfo_throttle(
                2.0,
                "corridor_centering active (%s): left %s right %s "
                "width %s error %.3f correction %.3f path %.3f "
                "output %.3f speed %.3f",
                wall_mode,
                left_text,
                right_text,
                ("%.3f" % current_width
                 if current_width is not None else "open"),
                center_error,
                correction,
                path_tracking_weight * msg.angular.z,
                command.angular.z,
                command.linear.x)
        elif (msg.linear.x >= self.minimum_forward_speed and
              abs(msg.angular.z) <=
              self.maximum_input_angular_speed + 1e-6):
            rospy.loginfo_throttle(
                2.0,
                "corridor_centering unavailable (%s): left %s right %s "
                "width %s confirmation %d/%d; passing input command",
                ("normal corridor not acquired"
                 if not normal_corridor_seen else
                 "no trustworthy corridor wall"),
                ("%.3f" % safety_left
                 if safety_left is not None else "open"),
                ("%.3f" % safety_right
                 if safety_right is not None else "open"),
                ("%.3f" % current_width
                 if current_width is not None else "unknown"),
                confirmation_count,
                self.normal_confirmation_samples)

        self.cmd_pub.publish(command)


if __name__ == '__main__':
    try:
        CorridorCentering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
