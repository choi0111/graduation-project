#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


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
            '~side_max_distance', 1.65)
        self.corridor_min_width = rospy.get_param(
            '~corridor_min_width', 1.60)
        self.corridor_max_width = rospy.get_param(
            '~corridor_max_width', 2.40)
        self.expected_corridor_width = rospy.get_param(
            '~expected_corridor_width', 2.00)
        self.corridor_width_tolerance = rospy.get_param(
            '~corridor_width_tolerance', 0.12)
        self.normal_confirmation_samples = int(rospy.get_param(
            '~normal_confirmation_samples', 8))
        self.initial_side_tolerance = rospy.get_param(
            '~initial_side_tolerance', 0.40)
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
            '~centering_gain', 0.16)
        self.centering_deadband = rospy.get_param(
            '~centering_deadband', 0.04)
        self.maximum_correction = rospy.get_param(
            '~maximum_correction', 0.06)
        self.distance_filter_alpha = rospy.get_param(
            '~distance_filter_alpha', 0.35)
        self.scan_timeout = rospy.get_param(
            '~scan_timeout', 0.50)

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

        rospy.loginfo(
            "corridor_centering: side sectors %.0f..%.0f deg, "
            "width %.2f..%.2f m, gain %.2f",
            math.degrees(self.side_min_angle),
            math.degrees(self.side_max_angle),
            self.corridor_min_width,
            self.corridor_max_width,
            self.centering_gain)

    @staticmethod
    def median(values):
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) * 0.5

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

        with self.lock:
            nominal_width = self.nominal_corridor_width
            target_side_distance = nominal_width * 0.5
            current_width = None
            if left is not None and right is not None:
                current_width = left + right

            self.current_corridor_width = current_width
            self.wall_mode = 'none'
            self.last_scan_wall_time = time.time()

            normal_geometry = (
                current_width is not None and
                self.corridor_min_width <= current_width <=
                self.corridor_max_width and
                abs(current_width - nominal_width) <=
                self.corridor_width_tolerance)

            if not self.normal_corridor_seen:
                initial_normal_geometry = (
                    normal_geometry and
                    abs(left - target_side_distance) <=
                    self.initial_side_tolerance and
                    abs(right - target_side_distance) <=
                    self.initial_side_tolerance)
                if initial_normal_geometry:
                    self.normal_confirmation_count += 1
                    self.normal_width_samples.append(current_width)
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
                 current_width > nominal_width +
                 self.corridor_width_tolerance))
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
            current_width = self.current_corridor_width
            nominal_width = self.nominal_corridor_width
            wall_mode = self.wall_mode
            normal_corridor_seen = self.normal_corridor_seen
            scan_time = self.last_scan_wall_time

        can_center = (
            msg.linear.x >= self.minimum_forward_speed and
            abs(msg.angular.z) <= self.maximum_input_angular_speed and
            normal_corridor_seen and
            wall_mode != 'none' and
            scan_time is not None and
            now - scan_time <= self.scan_timeout)

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
            command.angular.z = self.clamp(
                msg.angular.z + correction,
                -self.maximum_output_angular_speed,
                self.maximum_output_angular_speed)
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
                "width %s error %.3f correction %.3f",
                wall_mode,
                left_text,
                right_text,
                ("%.3f" % current_width
                 if current_width is not None else "open"),
                center_error,
                correction)
        elif (msg.linear.x >= self.minimum_forward_speed and
              abs(msg.angular.z) <= self.maximum_input_angular_speed):
            rospy.loginfo_throttle(
                2.0,
                "corridor_centering unavailable (%s); passing DWA command",
                ("normal corridor not acquired"
                 if not normal_corridor_seen else
                 "no trustworthy corridor wall"))

        self.cmd_pub.publish(command)


if __name__ == '__main__':
    try:
        CorridorCentering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
