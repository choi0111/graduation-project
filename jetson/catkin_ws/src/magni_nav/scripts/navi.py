#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import sys
import json
import threading
import time
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# =========================================================
# 1. [완벽 복구된 좌표 데이터베이스]
# =========================================================
locations = {
    # --- 문 앞 최종 목적지 좌표 ---
    "544호":   (-12.944165, 7.656571, 0.454100, 0.890950),
    "542호":   (-5.589854, 2.422153, 0.458113, 0.888893),
    "542호_대형": (-5.763933, 2.177825, 0.458113, 0.888893),
    "540호":   (1.640977, -2.824352, 0.450952, 0.892547),
    "545호":   (-8.620758, 2.768894, -0.887396, 0.461006),
    "543호":   (-1.434130, -2.615065, -0.886152, 0.463393),
    "541호":   (5.798706, -7.764325, -0.890077, 0.455810),
    "539호":   (13.208591, -12.786077, -0.885521, 0.464598),
    "537호":   (20.447883, -17.955123, -0.881675, 0.471855),
    "536호":   (23.571399, -18.135211, 0.471217, 0.882017),
    "535호":   (27.781724, -23.014583, -0.885415, 0.464799),
    "534호":   (30.959341, -23.152675, 0.458781, 0.888549),
    "533호":   (35.040370, -28.062911, -0.888468, 0.458936),
    "532b호":  (37.838409, -27.975978, 0.471912, 0.881645),
    "532a호":  (41.791103, -30.718851, 0.456097, 0.889929),
    "531호":   (42.315155, -33.115097, -0.877425, 0.479712),
    "엘베":    (-14.523725, 10.793320, -0.884044, 0.467402),

    # --- 복도 중앙(경유지) 좌표 ---
    "엘베_중앙":  (-15.885816, 8.888187, -0.884044, 0.467402),
    "544호_중앙": (-13.241020, 6.963121, 0.454100, 0.890950),
    "545호_중앙": (-8.185434, 3.287473, -0.887396, 0.461006),
    "542호_중앙": (-5.934249, 1.697414, 0.458113, 0.888893),
    "542호_대형_중앙": (-5.934249, 1.697414, 0.458113, 0.888893),
    "543호_중앙": (-0.987668, -1.818605, -0.886152, 0.463393),
    "540호_중앙": (1.298922, -3.393200, 0.450952, 0.892547),
    "541호_중앙": (6.338689, -7.008862, 0.004209, 0.999991), 
    "539호_중앙": (13.693975, -12.199325, -0.000112, 1.000000) 
}

cmd_vel_pub = None

CENTER_XY_GOAL_TOLERANCE = 1.00
FINAL_XY_GOAL_TOLERANCE = 0.15

# The large platform cannot safely use the old Magni room poses near the wall.
# Use the front laser for the final approach. These values are only hard
# travel limits; reaching the configured door clearance stops the robot first.
LIDAR_APPROACH_MAX_DISTANCES = {
    u"542호": 1.20,
    u"542호_대형": 1.20,
    u"544호": 1.20,
    u"545호": 1.20,
    u"543호": 1.00,
    u"540호": 1.00,
    u"541호": 1.00,
    u"539호": 1.00,
}
# move_base can prune the final center-plan pose before completing its yaw
# check. Accept the measured staging position here; navi performs the precise
# room-facing rotation immediately afterward.
CENTER_POSITION_TOLERANCES = {
    u"542호": 0.25,
    u"542호_대형": 0.25,
    u"544호": 0.25,
    u"545호": 0.25,
    u"543호": 0.90,
    u"540호": 0.90,
    u"541호": 0.90,
    u"539호": 0.90,
}
COARSE_GOAL_FALLBACK_MARGIN = 0.10
STAGING_LINE_ALONG_TOLERANCE = 0.20
STAGING_LINE_CROSS_TOLERANCE = 0.20
STAGING_LINE_MISS_STOP_TOLERANCE = 0.10
GOAL_PROGRESS_LOG_INTERVAL = 1.0
LIDAR_TO_FRONT_EDGE = 0.11
DOOR_FRONT_CLEARANCE = 0.30
LIDAR_DOOR_STOP_DISTANCE = LIDAR_TO_FRONT_EDGE + DOOR_FRONT_CLEARANCE
FRONT_SCAN_HALF_ANGLE = math.radians(10.0)
FRONT_SCAN_WAIT_TIMEOUT = 2.0
FRONT_SCAN_STALE_TIMEOUT = 0.5
FRONT_SCAN_REQUIRED_STOPS = 3
ROBOT_HALF_WIDTH = 0.385
ROBOT_REAR_FROM_LIDAR = 0.52
ROTATION_CLEARANCE_MARGIN = 0.03
ROTATION_CLEARANCE_RADIUS = (
    math.hypot(ROBOT_HALF_WIDTH, ROBOT_REAR_FROM_LIDAR) +
    ROTATION_CLEARANCE_MARGIN)
ROTATION_CLEARANCE_REQUIRED_POINTS = 5
LIDAR_APPROACH_MAX_START_RANGE = 1.50
LIDAR_APPROACH_LIMIT_MARGIN = 0.12
LIDAR_APPROACH_SPEED = 0.05
LIDAR_APPROACH_SLOW_SPEED = 0.03
LIDAR_APPROACH_SLOW_MARGIN = 0.20
LIDAR_APPROACH_TIMEOUT = 35.0
NEXT_GOAL_BACKUP_DISTANCE = 0.50
NEXT_GOAL_BACKUP_SPEED = 0.05
NEXT_GOAL_BACKUP_TIMEOUT = 18.0
ODOM_WAIT_TIMEOUT = 3.0
ODOM_STALE_TIMEOUT = 1.0
AMCL_WAIT_TIMEOUT = 3.0
AMCL_STALE_TIMEOUT = 2.0
ALIGN_YAW_TOLERANCE = 0.035
ALIGN_MIN_ANGULAR_SPEED = 0.025
ALIGN_MAX_ANGULAR_SPEED = 0.08
ALIGN_ANGULAR_KP = 0.8
ALIGN_TIMEOUT = 30.0
NEXT_GOAL_MIN_ANGULAR_SPEED = 0.03
NEXT_GOAL_MAX_ANGULAR_SPEED = 0.08
NEXT_GOAL_ALIGN_TIMEOUT = 50.0
INITIAL_GOAL_BEHIND_ANGLE = math.pi * 0.5
ITEM_RECEIPT_TIMEOUT = 20.0
HOME_LOCATION_NAME = u"initial_home"
HOME_XY_GOAL_TOLERANCE = 0.50
MISSION_REPLACE_AFTER_RESUME_WINDOW = 3.0
MISSION_REPLACE_JOIN_TIMEOUT = 3.0

try:
    text_type = unicode
    binary_type = str
except NameError:
    text_type = str
    binary_type = bytes


def as_text(value):
    if isinstance(value, text_type):
        return value
    if isinstance(value, binary_type):
        return value.decode('utf-8')
    return text_type(value)


def console_text(value):
    value = as_text(value)
    value = value.replace(u"호", "ho")
    value = value.replace(u"_중앙", "_center")
    value = value.replace(u"엘베", "elevator")
    return value.encode('ascii', 'ignore')


locations = dict((as_text(key), value) for key, value in locations.items())


def normalize_room_name(value):
    if value is None:
        return None
    room = as_text(value).strip().lower().replace(" ", "")
    if not room:
        return None
    if room in [u"엘베", u"엘리베이터", "elevator"]:
        return u"엘베"
    if room in locations:
        return room
    if room.endswith(u"호"):
        return room
    return room + u"호"


def room_for_status(location_name):
    room = location_name.replace(u"_중앙", "")
    if room.endswith(u"호"):
        return room[:-1]
    return room


def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class DeliveryNavigator(object):
    def __init__(self):
        global cmd_vel_pub
        rospy.init_node('navi_cmd_node')
        self.odom_position = None
        self.odom_yaw = None
        self.last_odom_wall_time = None
        self.amcl_position = None
        self.amcl_yaw = None
        self.last_amcl_wall_time = None
        self.front_scan_distance = None
        self.rotation_clearance_distance = None
        self.last_front_scan_wall_time = None
        self.front_scan_sequence = 0
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/llm_command', String, self.command_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.amcl_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.dwa_client = None
        self.mission_lock = threading.Lock()
        self.active_thread = None
        self.paused = False
        self.resume_status = "IDLE"
        self.last_resume_command_wall_time = None
        self.cancel_mission = False
        self.item_received = False
        self.waiting_for_item = False
        self.current_state = "IDLE"
        self.current_target = ""
        self.shutdown_started = False
        home_x = rospy.get_param(
            '/amcl/initial_pose_x', -15.5441206585)
        home_y = rospy.get_param(
            '/amcl/initial_pose_y', 8.32480237477)
        home_yaw = rospy.get_param(
            '/amcl/initial_pose_a', -0.6435763485)
        self.home_pose = (
            home_x,
            home_y,
            math.sin(home_yaw * 0.5),
            math.cos(home_yaw * 0.5))
        self.home_yaw = home_yaw
        locations[HOME_LOCATION_NAME] = self.home_pose

        rospy.on_shutdown(self.shutdown)

        rospy.sleep(1.0)
        print("[navi] waiting for move_base server...")
        self.client.wait_for_server()
        self.dwa_client = DynamicReconfigureClient(
            '/move_base/DWAPlannerROS', timeout=5.0)
        self.publish_status("IDLE")

    def publish_status(self, status):
        self.current_state = status
        self.status_pub.publish(status)
        rospy.loginfo("[robot_status] %s", status)

    def stop_robot(self):
        twist = Twist()
        cmd_vel_pub.publish(twist)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.odom_position = (position.x, position.y)
        self.odom_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_odom_wall_time = time.time()

    def amcl_callback(self, msg):
        position = msg.pose.pose.position
        self.amcl_position = (position.x, position.y)
        self.amcl_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.last_amcl_wall_time = time.time()

    def scan_callback(self, msg):
        forward_distances = []
        clearance_distances = []
        angle = msg.angle_min
        for measured_range in msg.ranges:
            if (not math.isnan(measured_range) and
                    not math.isinf(measured_range) and
                    measured_range >= msg.range_min and
                    measured_range <= msg.range_max):
                clearance_distances.append(measured_range)
                if abs(angle) <= FRONT_SCAN_HALF_ANGLE:
                    forward_distance = measured_range * math.cos(angle)
                    if forward_distance > 0.0:
                        forward_distances.append(forward_distance)
            angle += msg.angle_increment

        if not forward_distances or not clearance_distances:
            return

        forward_distances.sort()
        middle = len(forward_distances) // 2
        if len(forward_distances) % 2:
            median_distance = forward_distances[middle]
        else:
            median_distance = (
                forward_distances[middle - 1] +
                forward_distances[middle]) * 0.5

        self.front_scan_distance = median_distance
        clearance_distances.sort()
        clearance_index = min(
            ROTATION_CLEARANCE_REQUIRED_POINTS - 1,
            len(clearance_distances) - 1)
        self.rotation_clearance_distance = clearance_distances[clearance_index]
        self.last_front_scan_wall_time = time.time()
        self.front_scan_sequence += 1

    def shutdown(self):
        if self.shutdown_started:
            return

        self.shutdown_started = True
        self.cancel_mission = True
        self.paused = False

        try:
            self.client.cancel_all_goals()
        except Exception as exc:
            rospy.logwarn("Failed to cancel move_base goals during shutdown: %s", exc)

        # Keep publishing zero briefly so a final move_base command cannot win
        # the race with goal cancellation and leave the STM32 moving.
        zero_twist = Twist()
        for _ in range(20):
            try:
                cmd_vel_pub.publish(zero_twist)
            except Exception:
                break
            time.sleep(0.05)

    def build_goal(self, target_pose):
        x, y, z_ori, w_ori = target_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z_ori
        goal.target_pose.pose.orientation.w = w_ori
        return goal

    def pose_yaw(self, pose):
        return normalize_angle(2.0 * math.atan2(pose[2], pose[3]))

    def center_pose_facing_corridor(self, center_pose, room_pose):
        center_x, center_y = center_pose[0], center_pose[1]
        room_yaw = self.pose_yaw(room_pose)
        corridor_yaws = (
            normalize_angle(room_yaw + math.pi * 0.5),
            normalize_angle(room_yaw - math.pi * 0.5),
        )

        center_dx = None
        center_dy = None
        if self.amcl_position is not None:
            center_dx = center_x - self.amcl_position[0]
            center_dy = center_y - self.amcl_position[1]

        if (center_dx is not None and
                math.hypot(center_dx, center_dy) >= 0.05):
            reference_yaw = math.atan2(
                center_dy, center_dx)
        elif self.amcl_yaw is not None:
            reference_yaw = self.amcl_yaw
        else:
            reference_yaw = corridor_yaws[0]

        corridor_yaw = min(
            corridor_yaws,
            key=lambda yaw: abs(normalize_angle(yaw - reference_yaw)))
        return (
            center_x,
            center_y,
            math.sin(corridor_yaw * 0.5),
            math.cos(corridor_yaw * 0.5),
        )

    def wait_while_paused(self):
        while self.paused and not rospy.is_shutdown():
            self.stop_robot()
            rospy.sleep(0.2)

    def set_xy_goal_tolerance(self, tolerance):
        try:
            self.dwa_client.update_configuration({
                'xy_goal_tolerance': float(tolerance),
            })
            rospy.loginfo("DWA xy_goal_tolerance set to %.2f m", tolerance)
            return True
        except Exception as exc:
            rospy.logerr("Failed to set DWA xy_goal_tolerance: %s", exc)
            self.stop_robot()
            return False

    def distance_to_target(self, target_pose):
        if (self.amcl_position is None or
                self.last_amcl_wall_time is None or
                time.time() - self.last_amcl_wall_time > AMCL_STALE_TIMEOUT):
            return None
        return math.hypot(
            target_pose[0] - self.amcl_position[0],
            target_pose[1] - self.amcl_position[1])

    def staging_line_errors(self, center_pose, room_pose):
        if self.amcl_position is None:
            return None

        room_dx = room_pose[0] - center_pose[0]
        room_dy = room_pose[1] - center_pose[1]
        room_distance = math.hypot(room_dx, room_dy)
        if room_distance < 0.05:
            return None

        room_normal_x = room_dx / room_distance
        room_normal_y = room_dy / room_distance
        corridor_x = -room_normal_y
        corridor_y = room_normal_x
        offset_x = self.amcl_position[0] - center_pose[0]
        offset_y = self.amcl_position[1] - center_pose[1]

        along_error = abs(offset_x * corridor_x + offset_y * corridor_y)
        cross_error = abs(
            offset_x * room_normal_x + offset_y * room_normal_y)
        return along_error, cross_error

    def evaluate_position_goal(self, location_name, target_pose, tolerance,
                               staging_room_pose=None):
        if tolerance is None:
            return 0
        distance = self.distance_to_target(target_pose)
        if distance is None:
            return 0
        if distance <= tolerance:
            self.client.cancel_goal()
            self.stop_robot()
            print("[navi] arrived at {} (position {:.3f} m)".format(
                console_text(location_name), distance))
            return 1

        if staging_room_pose is None:
            return 0
        line_errors = self.staging_line_errors(target_pose, staging_room_pose)
        if line_errors is None:
            return 0
        along_error, cross_error = line_errors
        if (along_error <= STAGING_LINE_ALONG_TOLERANCE and
                cross_error <= STAGING_LINE_CROSS_TOLERANCE):
            self.client.cancel_goal()
            self.stop_robot()
            print(
                "[navi] arrived at {} (staging line: along {:.3f} m, "
                "cross {:.3f} m)".format(
                    console_text(location_name), along_error, cross_error))
            return 1

        if along_error <= STAGING_LINE_MISS_STOP_TOLERANCE:
            self.client.cancel_goal()
            self.stop_robot()
            print(
                "[navi] crossed {} too far from corridor center "
                "(cross {:.3f} m at x {:.3f}, y {:.3f}); stopping for "
                "waypoint calibration".format(
                    console_text(location_name), cross_error,
                    self.amcl_position[0], self.amcl_position[1]))
            return -1
        return 0

    def log_goal_progress(self, location_name, target_pose,
                          staging_room_pose=None):
        distance = self.distance_to_target(target_pose)
        if distance is None:
            print("[navi] {} waiting for fresh /amcl_pose".format(
                console_text(location_name)))
            return
        if staging_room_pose is None:
            print("[navi] {} distance {:.3f} m".format(
                console_text(location_name), distance))
            return
        line_errors = self.staging_line_errors(target_pose, staging_room_pose)
        if line_errors is None:
            return
        along_error, cross_error = line_errors
        print(
            "[navi] {} distance {:.3f} m, staging along {:.3f} m, "
            "cross {:.3f} m (x {:.3f}, y {:.3f})".format(
                console_text(location_name), distance,
                along_error, cross_error,
                self.amcl_position[0], self.amcl_position[1]))

    def move_to_goal(self, location_name, target_pose=None,
                     position_tolerance=None, staging_room_pose=None):
        if target_pose is None and location_name not in locations:
            print("[navi] unknown location: {}".format(console_text(location_name)))
            return False

        if target_pose is None:
            target_pose = locations[location_name]

        print("\n[navi] moving to {}".format(console_text(location_name)))
        last_progress_log = 0.0
        while not rospy.is_shutdown():
            if self.cancel_mission:
                self.client.cancel_goal()
                self.stop_robot()
                return False

            self.wait_while_paused()
            if self.cancel_mission or rospy.is_shutdown():
                return False

            position_state = self.evaluate_position_goal(
                location_name, target_pose, position_tolerance,
                staging_room_pose)
            if position_state > 0:
                return True
            if position_state < 0:
                return False

            self.client.send_goal(self.build_goal(target_pose))
            while not rospy.is_shutdown():
                if self.cancel_mission:
                    self.client.cancel_goal()
                    self.stop_robot()
                    return False
                if self.paused:
                    self.client.cancel_goal()
                    self.stop_robot()
                    self.wait_while_paused()
                    break
                now = time.time()
                if (position_tolerance is not None and
                        now - last_progress_log >= GOAL_PROGRESS_LOG_INTERVAL):
                    self.log_goal_progress(
                        location_name, target_pose, staging_room_pose)
                    last_progress_log = now
                position_state = self.evaluate_position_goal(
                    location_name, target_pose, position_tolerance,
                    staging_room_pose)
                if position_state > 0:
                    return True
                if position_state < 0:
                    return False
                if self.client.wait_for_result(rospy.Duration(0.2)):
                    state = self.client.get_state()
                    if state in (GoalStatus.PREEMPTED,
                                 GoalStatus.RECALLED):
                        self.stop_robot()
                        rospy.loginfo(
                            "Navigation goal paused; waiting to resend the "
                            "same destination")
                        self.wait_while_paused()
                        if self.cancel_mission or rospy.is_shutdown():
                            return False
                        rospy.loginfo(
                            "Resuming navigation to %s",
                            console_text(location_name))
                        break
                    if state == GoalStatus.SUCCEEDED:
                        self.stop_robot()
                        print("[navi] arrived at {}".format(console_text(location_name)))
                        return True
                    if (position_tolerance is not None and
                            state in (GoalStatus.ABORTED,
                                      GoalStatus.REJECTED,
                                      GoalStatus.LOST)):
                        distance = self.distance_to_target(target_pose)
                        fallback_tolerance = (
                            position_tolerance + COARSE_GOAL_FALLBACK_MARGIN)
                        if (distance is not None and
                                distance <= fallback_tolerance):
                            self.stop_robot()
                            print(
                                "[navi] accepting {} near staging point after "
                                "planner failure (position {:.3f} m, limit "
                                "{:.3f} m)".format(
                                    console_text(location_name), distance,
                                    fallback_tolerance))
                            return True
                    self.stop_robot()
                    print("[navi] failed to reach {}. state={}".format(console_text(location_name), state))
                    return False

    def wait_for_fresh_odom(self, timeout):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_mission:
                return False
            if (self.odom_position is not None and
                    self.last_odom_wall_time is not None and
                    time.time() - self.last_odom_wall_time <= ODOM_STALE_TIMEOUT):
                return True
            self.stop_robot()
            rospy.sleep(0.05)
        return False

    def wait_for_fresh_localization(self, timeout):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_mission:
                return False
            if (self.amcl_position is not None and
                    self.amcl_yaw is not None and
                    self.last_amcl_wall_time is not None and
                    time.time() - self.last_amcl_wall_time <= AMCL_STALE_TIMEOUT):
                return True
            self.stop_robot()
            rospy.sleep(0.05)
        return False

    def prepare_direct_alignment(self, context):
        self.client.cancel_all_goals()
        self.stop_robot()
        rospy.sleep(0.2)

        if not self.wait_for_fresh_odom(ODOM_WAIT_TIMEOUT):
            rospy.logerr("Fresh /odom data is required for %s", context)
            return False
        if not self.wait_for_fresh_localization(AMCL_WAIT_TIMEOUT):
            rospy.logerr("Fresh /amcl_pose data is required for %s", context)
            return False
        return True

    def rotate_to_map_yaw(self, target_name, target_yaw, action_text,
                          min_angular_speed=ALIGN_MIN_ANGULAR_SPEED,
                          max_angular_speed=ALIGN_MAX_ANGULAR_SPEED,
                          timeout=ALIGN_TIMEOUT):
        if not self.wait_for_fresh_front_scan(FRONT_SCAN_WAIT_TIMEOUT):
            rospy.logerr(
                "Fresh /scan data is required before direct rotation")
            self.stop_robot()
            return False
        if (self.rotation_clearance_distance is None or
                self.rotation_clearance_distance <
                ROTATION_CLEARANCE_RADIUS):
            rospy.logerr(
                "Direct rotation refused: nearest surface %.3f m, "
                "required %.3f m",
                self.rotation_clearance_distance
                if self.rotation_clearance_distance is not None else -1.0,
                ROTATION_CLEARANCE_RADIUS)
            self.stop_robot()
            return False

        required_rotation = normalize_angle(target_yaw - self.amcl_yaw)
        start_odom_yaw = self.odom_yaw
        start_time = time.time()
        rate = rospy.Rate(20)

        print("[navi] {} {} ({:.1f} deg remaining)".format(
            action_text, console_text(target_name),
            math.degrees(required_rotation)))

        while not rospy.is_shutdown():
            if self.cancel_mission:
                self.stop_robot()
                return False

            if self.paused:
                pause_started = time.time()
                self.stop_robot()
                self.wait_while_paused()
                start_time += time.time() - pause_started
                continue

            if (self.odom_yaw is None or self.last_odom_wall_time is None or
                    time.time() - self.last_odom_wall_time > ODOM_STALE_TIMEOUT):
                rospy.logerr("/odom stopped during direct rotation")
                self.stop_robot()
                return False
            if (self.last_front_scan_wall_time is None or
                    time.time() - self.last_front_scan_wall_time >
                    FRONT_SCAN_STALE_TIMEOUT):
                rospy.logerr("/scan stopped during direct rotation")
                self.stop_robot()
                return False
            if (self.rotation_clearance_distance is None or
                    self.rotation_clearance_distance <
                    ROTATION_CLEARANCE_RADIUS):
                rospy.logerr(
                    "Direct rotation stopped: nearest surface %.3f m, "
                    "required %.3f m",
                    self.rotation_clearance_distance
                    if self.rotation_clearance_distance is not None else -1.0,
                    ROTATION_CLEARANCE_RADIUS)
                self.stop_robot()
                return False

            rotated = normalize_angle(self.odom_yaw - start_odom_yaw)
            error = normalize_angle(required_rotation - rotated)
            if abs(error) <= ALIGN_YAW_TOLERANCE:
                self.stop_robot()
                print("[navi] rotation complete (error {:.1f} deg)".format(
                    math.degrees(error)))
                return True

            if time.time() - start_time > timeout:
                rospy.logerr(
                    "Direct rotation timed out with %.1f deg remaining",
                    math.degrees(error))
                self.stop_robot()
                return False

            angular_speed = ALIGN_ANGULAR_KP * abs(error)
            angular_speed = min(max_angular_speed, angular_speed)
            angular_speed = max(min_angular_speed, angular_speed)
            command = Twist()
            command.angular.z = math.copysign(angular_speed, error)
            cmd_vel_pub.publish(command)
            rate.sleep()

        self.stop_robot()
        return False

    def align_to_room(self, room_name, room_pose):
        if not self.prepare_direct_alignment("room alignment"):
            return False

        target_yaw = self.pose_yaw(room_pose)
        return self.rotate_to_map_yaw(
            room_name, target_yaw, "fine-aligning toward")

    def navigation_target_for(self, room_name):
        center_target = room_name + u"_중앙"
        if center_target in locations:
            return center_target
        return room_name

    def align_toward_destination(self, room_name):
        if not self.prepare_direct_alignment("next-destination alignment"):
            return False

        target_name = self.navigation_target_for(room_name)
        target_pose = locations[target_name]
        current_x, current_y = self.amcl_position
        delta_x = target_pose[0] - current_x
        delta_y = target_pose[1] - current_y
        if math.hypot(delta_x, delta_y) < 0.05:
            self.stop_robot()
            return True

        target_yaw = math.atan2(delta_y, delta_x)
        return self.rotate_to_map_yaw(
            room_name,
            target_yaw,
            "turning toward next destination",
            NEXT_GOAL_MIN_ANGULAR_SPEED,
            NEXT_GOAL_MAX_ANGULAR_SPEED,
            NEXT_GOAL_ALIGN_TIMEOUT)

    def align_first_destination_if_behind(self, room_name):
        if not self.prepare_direct_alignment("initial-destination alignment"):
            return False

        target_name = self.navigation_target_for(room_name)
        target_pose = locations[target_name]
        current_x, current_y = self.amcl_position
        delta_x = target_pose[0] - current_x
        delta_y = target_pose[1] - current_y
        if math.hypot(delta_x, delta_y) < 0.05:
            self.stop_robot()
            return True

        target_yaw = math.atan2(delta_y, delta_x)
        heading_error = normalize_angle(target_yaw - self.amcl_yaw)
        if abs(heading_error) <= INITIAL_GOAL_BEHIND_ANGLE:
            print(
                "[navi] first destination is ahead ({:.1f} deg); "
                "keeping current heading".format(math.degrees(heading_error)))
            self.stop_robot()
            return True

        return self.rotate_to_map_yaw(
            room_name,
            target_yaw,
            "turning in place toward first destination",
            NEXT_GOAL_MIN_ANGULAR_SPEED,
            NEXT_GOAL_MAX_ANGULAR_SPEED,
            NEXT_GOAL_ALIGN_TIMEOUT)

    def drive_straight_distance(self, description, distance, speed, timeout):
        self.client.cancel_all_goals()
        self.stop_robot()
        rospy.sleep(0.2)

        if not self.wait_for_fresh_odom(ODOM_WAIT_TIMEOUT):
            rospy.logerr("Fresh /odom data is required for %s", description)
            self.stop_robot()
            return False

        start_x, start_y = self.odom_position
        start_time = time.time()
        rate = rospy.Rate(20)
        command = Twist()
        command.linear.x = speed

        print("[navi] {}: {:.2f} m".format(description, distance))

        while not rospy.is_shutdown():
            if self.cancel_mission:
                self.stop_robot()
                return False

            if self.paused:
                pause_started = time.time()
                self.stop_robot()
                self.wait_while_paused()
                start_time += time.time() - pause_started
                continue

            if (self.last_odom_wall_time is None or
                    time.time() - self.last_odom_wall_time > ODOM_STALE_TIMEOUT):
                rospy.logerr("/odom stopped during %s", description)
                self.stop_robot()
                return False

            current_x, current_y = self.odom_position
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if traveled >= distance:
                self.stop_robot()
                print("[navi] {} complete at {:.3f} m".format(
                    description, traveled))
                return True

            if time.time() - start_time > timeout:
                rospy.logerr(
                    "%s timed out after %.3f m (target %.3f m)",
                    description, traveled, distance)
                self.stop_robot()
                return False

            cmd_vel_pub.publish(command)
            rate.sleep()

        self.stop_robot()
        return False

    def wait_for_fresh_front_scan(self, timeout):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.cancel_mission:
                return False
            if (self.front_scan_distance is not None and
                    self.last_front_scan_wall_time is not None and
                    time.time() - self.last_front_scan_wall_time <=
                    FRONT_SCAN_STALE_TIMEOUT):
                return True
            self.stop_robot()
            rospy.sleep(0.05)
        return False

    def drive_forward_to_door(self, room_name, maximum_distance):
        description = "approaching {}".format(console_text(room_name))
        self.client.cancel_all_goals()
        self.stop_robot()
        rospy.sleep(0.2)

        if not self.wait_for_fresh_odom(ODOM_WAIT_TIMEOUT):
            rospy.logerr("Fresh /odom data is required for %s", description)
            return False
        if not self.wait_for_fresh_front_scan(FRONT_SCAN_WAIT_TIMEOUT):
            rospy.logerr("Fresh front /scan data is required for %s", description)
            return False

        initial_front_distance = self.front_scan_distance
        if initial_front_distance > LIDAR_APPROACH_MAX_START_RANGE:
            rospy.logerr(
                "%s refused: front surface is %.3f m away (limit %.3f m)",
                description,
                initial_front_distance,
                LIDAR_APPROACH_MAX_START_RANGE)
            self.stop_robot()
            return False

        if initial_front_distance <= LIDAR_DOOR_STOP_DISTANCE:
            self.stop_robot()
            print(
                "[navi] {} already at door clearance "
                "(front gap {:.3f} m)".format(
                    description,
                    max(0.0, initial_front_distance -
                        LIDAR_TO_FRONT_EDGE)))
            return True

        expected_travel = (
            initial_front_distance - LIDAR_DOOR_STOP_DISTANCE)
        travel_limit = min(
            maximum_distance,
            expected_travel + LIDAR_APPROACH_LIMIT_MARGIN)
        start_x, start_y = self.odom_position
        start_time = time.time()
        rate = rospy.Rate(20)
        last_scan_sequence = self.front_scan_sequence
        stopped_scan_count = 0

        print(
            "[navi] {} using front lidar: {:.3f} m -> {:.3f} m "
            "(robot gap {:.3f} m, max travel {:.3f} m)".format(
                description,
                initial_front_distance,
                LIDAR_DOOR_STOP_DISTANCE,
                DOOR_FRONT_CLEARANCE,
                travel_limit))

        while not rospy.is_shutdown():
            if self.cancel_mission:
                self.stop_robot()
                return False

            if self.paused:
                pause_started = time.time()
                self.stop_robot()
                self.wait_while_paused()
                start_time += time.time() - pause_started
                stopped_scan_count = 0
                continue

            now = time.time()
            if (self.last_odom_wall_time is None or
                    now - self.last_odom_wall_time > ODOM_STALE_TIMEOUT):
                rospy.logerr("/odom stopped during %s", description)
                self.stop_robot()
                return False
            if (self.last_front_scan_wall_time is None or
                    now - self.last_front_scan_wall_time >
                    FRONT_SCAN_STALE_TIMEOUT):
                rospy.logerr("Front /scan stopped during %s", description)
                self.stop_robot()
                return False

            front_distance = self.front_scan_distance
            if self.front_scan_sequence != last_scan_sequence:
                last_scan_sequence = self.front_scan_sequence
                if front_distance <= LIDAR_DOOR_STOP_DISTANCE:
                    stopped_scan_count += 1
                else:
                    stopped_scan_count = 0

            if front_distance <= LIDAR_DOOR_STOP_DISTANCE:
                self.stop_robot()
                if stopped_scan_count >= FRONT_SCAN_REQUIRED_STOPS:
                    current_x, current_y = self.odom_position
                    traveled = math.hypot(
                        current_x - start_x, current_y - start_y)
                    print(
                        "[navi] {} complete at {:.3f} m "
                        "(front gap {:.3f} m)".format(
                            description,
                            traveled,
                            max(0.0, front_distance -
                                LIDAR_TO_FRONT_EDGE)))
                    return True
                rate.sleep()
                continue

            current_x, current_y = self.odom_position
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if traveled >= travel_limit:
                rospy.logerr(
                    "%s stopped at safety travel limit %.3f m; "
                    "front surface remains %.3f m away",
                    description, traveled, front_distance)
                self.stop_robot()
                return False

            if now - start_time > LIDAR_APPROACH_TIMEOUT:
                rospy.logerr(
                    "%s timed out after %.3f m; front surface %.3f m away",
                    description, traveled, front_distance)
                self.stop_robot()
                return False

            command = Twist()
            if (front_distance - LIDAR_DOOR_STOP_DISTANCE <=
                    LIDAR_APPROACH_SLOW_MARGIN):
                command.linear.x = LIDAR_APPROACH_SLOW_SPEED
            else:
                command.linear.x = LIDAR_APPROACH_SPEED
            cmd_vel_pub.publish(command)
            rate.sleep()

        self.stop_robot()
        return False

    def backup_for_next_destination(self):
        return self.drive_straight_distance(
            "backing up before next destination",
            NEXT_GOAL_BACKUP_DISTANCE,
            -NEXT_GOAL_BACKUP_SPEED,
            NEXT_GOAL_BACKUP_TIMEOUT)

    def prepare_for_next_destination(self, room_name):
        if not self.backup_for_next_destination():
            return False
        rospy.sleep(0.5)
        if not self.align_toward_destination(room_name):
            return False
        self.stop_robot()
        rospy.sleep(0.5)
        return True

    def move_to_room(self, room_name):
        center_target = room_name + u"_중앙"
        has_center_target = center_target in locations
        if has_center_target:
            center_tolerance = CENTER_POSITION_TOLERANCES.get(
                room_name, CENTER_XY_GOAL_TOLERANCE)
            if not self.set_xy_goal_tolerance(center_tolerance):
                return False
            center_pose = locations[center_target]
            if room_name in LIDAR_APPROACH_MAX_DISTANCES:
                center_pose = self.center_pose_facing_corridor(
                    center_pose, locations[room_name])
            position_tolerance = CENTER_POSITION_TOLERANCES.get(room_name)
            if not self.move_to_goal(
                    center_target, center_pose, position_tolerance,
                    locations[room_name]):
                print("[navi] center waypoint failed for {}; skipping final approach".format(console_text(room_name)))
                return False
            self.stop_robot()
            rospy.sleep(1.0)

        if room_name in LIDAR_APPROACH_MAX_DISTANCES:
            if not self.align_to_room(
                    room_name, locations[room_name]):
                return False
            rospy.sleep(0.5)
            return self.drive_forward_to_door(
                room_name, LIDAR_APPROACH_MAX_DISTANCES[room_name])

        final_pose = locations[room_name]
        if not has_center_target:
            return self.move_to_goal(room_name, final_pose)

        if not self.set_xy_goal_tolerance(FINAL_XY_GOAL_TOLERANCE):
            return False

        try:
            return self.move_to_goal(room_name, final_pose)
        finally:
            self.set_xy_goal_tolerance(CENTER_XY_GOAL_TOLERANCE)

    def wait_for_item(self, room_name, has_next):
        self.item_received = False
        self.waiting_for_item = True
        self.publish_status("ARRIVED:{}".format(room_for_status(room_name)))
        self.status_pub.publish("SCENARIO_5")
        rospy.loginfo(
            "[WAITING] 물품 수령 확인을 위해 %.0f초 대기합니다.",
            ITEM_RECEIPT_TIMEOUT)

        remaining_time = ITEM_RECEIPT_TIMEOUT
        try:
            while remaining_time > 0.0 and not rospy.is_shutdown():
                if self.cancel_mission:
                    return False
                if self.paused:
                    self.wait_while_paused()
                    continue
                if self.item_received:
                    rospy.loginfo(
                        "[RECEIVED] %s호 물품 수령 음성을 확인했습니다.",
                        room_for_status(room_name))
                    self.status_pub.publish("SCENARIO_8")
                    rospy.sleep(2.0)
                    return True
                sleep_time = min(0.1, remaining_time)
                rospy.sleep(sleep_time)
                remaining_time -= sleep_time

            self.status_pub.publish("SCENARIO_13")
            rospy.loginfo(
                "[TIMEOUT] 물품 수령 확인 없음. 배송 실패 처리 후 "
                "다음 단계로 넘어갑니다.")
            rospy.sleep(3.0)
            return True
        finally:
            self.waiting_for_item = False

    def return_to_initial_position(self):
        self.status_pub.publish("SCENARIO_9")
        rospy.sleep(3.0)
        self.publish_status("RETURNING")

        if not self.backup_for_next_destination():
            rospy.logerr(
                "Failed to back up before returning to the initial position")
            return False
        rospy.sleep(0.5)
        if not self.align_toward_destination(HOME_LOCATION_NAME):
            rospy.logerr("Failed to align toward the initial position")
            return False
        self.stop_robot()
        rospy.sleep(0.5)

        if not self.set_xy_goal_tolerance(HOME_XY_GOAL_TOLERANCE):
            return False
        try:
            if not self.move_to_goal(
                    HOME_LOCATION_NAME,
                    self.home_pose,
                    HOME_XY_GOAL_TOLERANCE):
                return False
        finally:
            self.set_xy_goal_tolerance(CENTER_XY_GOAL_TOLERANCE)

        self.stop_robot()
        rospy.sleep(0.5)
        if not self.rotate_to_map_yaw(
                HOME_LOCATION_NAME,
                self.home_yaw,
                "aligning at initial position",
                NEXT_GOAL_MIN_ANGULAR_SPEED,
                NEXT_GOAL_MAX_ANGULAR_SPEED,
                NEXT_GOAL_ALIGN_TIMEOUT):
            return False

        self.stop_robot()
        rospy.loginfo("[RETURNED] 초기 위치 복귀가 완료되었습니다.")
        return True

    def run_delivery_journey(self, rooms):
        normalized_rooms = []
        for room in rooms:
            normalized = normalize_room_name(room)
            if normalized in locations:
                normalized_rooms.append(normalized)
            else:
                rospy.logwarn("Unknown delivery room ignored: %s", room)

        if not normalized_rooms:
            rospy.logwarn("No valid delivery rooms in command: %s", rooms)
            self.publish_status("IDLE")
            return

        with self.mission_lock:
            self.cancel_mission = False
            self.paused = False

        for index, room in enumerate(normalized_rooms):
            if rospy.is_shutdown() or self.cancel_mission:
                return

            self.current_target = room_for_status(room)
            self.publish_status("MOVING:{}".format(self.current_target))
            if index == 0 and not self.align_first_destination_if_behind(room):
                self.stop_robot()
                if self.cancel_mission:
                    return
                self.publish_status("NAV_FAILED")
                return
            success = self.move_to_room(room)
            if not success:
                self.stop_robot()
                if self.cancel_mission:
                    return
                self.publish_status("NAV_FAILED")
                return

            has_next = index < len(normalized_rooms) - 1
            if not self.wait_for_item(room, has_next):
                return
            if has_next:
                next_room = normalized_rooms[index + 1]
                if not self.prepare_for_next_destination(next_room):
                    self.stop_robot()
                    if self.cancel_mission:
                        return
                    self.publish_status("NAV_FAILED")
                    return

        if not self.return_to_initial_position():
            self.stop_robot()
            if self.cancel_mission:
                return
            rospy.logerr("Initial-position return failed")
            self.current_target = ""
            self.publish_status("RETURN_FAILED")
            return
        self.stop_robot()
        self.current_target = ""
        self.publish_status("IDLE")

    def replace_active_mission(self, payload):
        rospy.logwarn(
            "Replacing the paused active mission with new destinations: %s",
            payload)
        self.cancel_mission = True
        self.paused = False
        self.last_resume_command_wall_time = None
        self.item_received = False
        try:
            self.client.cancel_all_goals()
        except Exception as exc:
            rospy.logwarn(
                "Failed to cancel move_base goal during mission replacement: %s",
                exc)
        self.stop_robot()

        old_thread = self.active_thread
        if old_thread and old_thread.is_alive():
            old_thread.join(MISSION_REPLACE_JOIN_TIMEOUT)
        if old_thread and old_thread.is_alive():
            rospy.logerr(
                "Old mission did not stop within %.1f seconds; "
                "new destination rejected for safety",
                MISSION_REPLACE_JOIN_TIMEOUT)
            self.publish_status("NAV_FAILED")
            return False

        self.active_thread = None
        self.cancel_mission = False
        self.waiting_for_item = False
        rospy.loginfo(
            "Previous mission stopped; starting replacement destinations: %s",
            payload)
        return True

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cmd = data.get("command")
            payload = data.get("payload", [])
            if not isinstance(payload, list):
                payload = [payload]
        except Exception as exc:
            rospy.logwarn("Invalid /llm_command message: %s", exc)
            return

        rospy.loginfo("[LLM command] %s %s", cmd, payload)

        if cmd == "SCENARIO_21":
            if not self.paused:
                self.resume_status = self.current_state
            self.paused = True
            self.last_resume_command_wall_time = None
            self.client.cancel_goal()
            self.stop_robot()
            self.publish_status("PAUSED")
            rospy.loginfo(
                "Mission paused without clearing the active destination")
            return

        if cmd == "SCENARIO_22":
            if self.paused:
                self.paused = False
                self.last_resume_command_wall_time = time.time()
                resume_status = self.resume_status
                if not resume_status or resume_status == "PAUSED":
                    resume_status = (
                        "MOVING:{}".format(self.current_target)
                        if self.current_target else "MOVING")
                self.publish_status(resume_status)
                rospy.loginfo(
                    "Mission resumed with status %s", resume_status)
            else:
                rospy.logwarn(
                    "Resume command ignored because the mission is not paused")
            return

        if cmd == "SCENARIO_8":
            if self.waiting_for_item:
                self.item_received = True
                rospy.loginfo(
                    "[LLM command] item-received signal accepted")
            else:
                rospy.logwarn(
                    "Item-received signal ignored because the robot is not "
                    "waiting at a destination")
            return

        if cmd not in ["SCENARIO_1", "SCENARIO_2", "SCENARIO_3", "SCENARIO_4", "SCENARIO_6"]:
            return

        valid_payload = [
            room for room in payload
            if normalize_room_name(room) in locations
        ]
        if not valid_payload:
            rospy.logwarn(
                "New mission ignored because it has no known destinations: %s",
                payload)
            return

        if self.active_thread and self.active_thread.is_alive():
            resumed_recently = (
                self.last_resume_command_wall_time is not None and
                time.time() - self.last_resume_command_wall_time <=
                MISSION_REPLACE_AFTER_RESUME_WINDOW)
            if not self.paused and not resumed_recently:
                rospy.logwarn(
                    "Mission already running. New command ignored: %s",
                    payload)
                return
            if not self.replace_active_mission(payload):
                return

        self.active_thread = threading.Thread(target=self.run_delivery_journey, args=(payload,))
        self.active_thread.daemon = True
        self.active_thread.start()

    def run_cli_goals(self, goals):
        normalized_rooms = []
        for target in goals:
            room = normalize_room_name(target)
            if room not in locations:
                print("[navi] unknown location: {}".format(console_text(target)))
                continue
            normalized_rooms.append(room)

        if not normalized_rooms:
            print("[navi] no valid destinations")
            return

        for index, room in enumerate(normalized_rooms):
            if rospy.is_shutdown() or self.cancel_mission:
                return

            self.current_target = room_for_status(room)
            self.publish_status("MOVING:{}".format(self.current_target))
            if index == 0 and not self.align_first_destination_if_behind(room):
                self.stop_robot()
                self.publish_status("IDLE")
                return
            success = self.move_to_room(room)
            if not success:
                self.stop_robot()
                self.publish_status("IDLE")
                print("[navi] route stopped after a failed destination")
                return

            self.publish_status("ARRIVED:{}".format(self.current_target))
            self.stop_robot()
            rospy.sleep(3.0)

            has_next = index < len(normalized_rooms) - 1
            if has_next:
                next_room = normalized_rooms[index + 1]
                if not self.prepare_for_next_destination(next_room):
                    self.stop_robot()
                    self.publish_status("IDLE")
                    print("[navi] failed to prepare for next destination")
                    return

        self.publish_status("IDLE")
        print("\n[navi] delivery sequence complete")

    def run(self):
        input_goals = sys.argv[1:]
        if input_goals:
            self.run_cli_goals(input_goals)
        else:
            print("[navi] waiting for /llm_command")
            rospy.spin()


if __name__ == '__main__':
    try:
        DeliveryNavigator().run()
    except rospy.ROSInterruptException:
        pass
