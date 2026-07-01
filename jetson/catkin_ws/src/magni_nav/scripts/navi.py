#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import sys
import json
import threading
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# =========================================================
# 1. [완벽 복구된 좌표 데이터베이스]
# =========================================================
locations = {
    # --- 문 앞 최종 목적지 좌표 ---
    "544호":   (-12.944165, 7.656571, 0.454100, 0.890950),
    "542호":   (-5.589854, 2.422153, 0.458113, 0.888893),
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
    "543호_중앙": (-0.987668, -1.818605, -0.886152, 0.463393),
    "540호_중앙": (1.298922, -3.393200, 0.450952, 0.892547),
    "541호_중앙": (6.338689, -7.008862, 0.004209, 0.999991), 
    "539호_중앙": (13.693975, -12.199325, -0.000112, 1.000000) 
}

cmd_vel_pub = None
tf_listener = None

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


locations = dict((as_text(key), value) for key, value in locations.items())


def get_current_yaw():
    global tf_listener
    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        return tf.transformations.euler_from_quaternion(rot)[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None


def normalize_room_name(value):
    if value is None:
        return None
    room = as_text(value).strip().lower().replace(" ", "")
    if not room:
        return None
    if room in [u"엘베", u"엘리베이터", "elevator"]:
        return u"엘베"
    if room.endswith(u"호"):
        return room
    return room + u"호"


def room_for_status(location_name):
    room = location_name.replace(u"_중앙", "")
    if room.endswith(u"호"):
        return room[:-1]
    return room


class DeliveryNavigator(object):
    def __init__(self):
        global cmd_vel_pub, tf_listener
        rospy.init_node('navi_cmd_node')
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
        self.command_sub = rospy.Subscriber('/llm_command', String, self.command_callback)
        tf_listener = tf.TransformListener()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mission_lock = threading.Lock()
        self.active_thread = None
        self.paused = False
        self.cancel_mission = False
        self.item_received = False
        self.current_state = "IDLE"
        self.current_target = ""

        rospy.sleep(1.0)
        print("⏳ 자율주행 서버 대기 중...")
        self.client.wait_for_server()
        self.publish_status("IDLE")

    def publish_status(self, status):
        self.current_state = status
        self.status_pub.publish(status)
        rospy.loginfo("[robot_status] %s", status)

    def stop_robot(self):
        twist = Twist()
        cmd_vel_pub.publish(twist)

    def build_goal(self, location_name):
        x, y, z_ori, w_ori = locations[location_name]
        current_yaw = get_current_yaw()
        if current_yaw is not None:
            q = tf.transformations.quaternion_from_euler(0, 0, current_yaw)
            z_ori = q[2]
            w_ori = q[3]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z_ori
        goal.target_pose.pose.orientation.w = w_ori
        return goal

    def wait_while_paused(self):
        while self.paused and not rospy.is_shutdown():
            self.stop_robot()
            rospy.sleep(0.2)

    def move_to_goal(self, location_name):
        if location_name not in locations:
            print("❌ '{}' 좌표가 데이터베이스에 없습니다.".format(location_name))
            return False

        print("\n>>> 🚀 [{}] 이동 시작...".format(location_name))
        while not rospy.is_shutdown():
            if self.cancel_mission:
                self.client.cancel_goal()
                self.stop_robot()
                return False

            self.wait_while_paused()
            if self.cancel_mission or rospy.is_shutdown():
                return False

            self.client.send_goal(self.build_goal(location_name))
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
                if self.client.wait_for_result(rospy.Duration(0.2)):
                    state = self.client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        print("✅ [{}] 도착 완료!".format(location_name))
                        return True
                    print("❌ [{}] 도착 실패. (상태 코드: {})".format(location_name, state))
                    return False

    def move_to_room(self, room_name):
        center_target = room_name + u"_중앙"
        if center_target in locations:
            if not self.move_to_goal(center_target):
                print("⚠️ [{}] 중앙 경유 실패. 최종 목적지 진입을 생략합니다.".format(room_name))
                return False
            rospy.sleep(1.0)
        return self.move_to_goal(room_name)

    def backup_50cm(self):
        print("\n🚗 [안전 확보] 문 앞 공간을 빠져나오기 위해 50cm 후진합니다...")
        twist = Twist()
        twist.linear.x = -0.15
        rate = rospy.Rate(10)
        for _ in range(35):
            if rospy.is_shutdown() or self.cancel_mission:
                break
            self.wait_while_paused()
            cmd_vel_pub.publish(twist)
            rate.sleep()
        self.stop_robot()
        rospy.sleep(1.0)
        print("✅ 후진 완료. 다음 목적지로 주행을 준비합니다.")

    def wait_for_item(self, room_name, has_next):
        self.item_received = False
        self.publish_status("ARRIVED:{}".format(room_for_status(room_name)))
        self.status_pub.publish("SCENARIO_5")
        rospy.loginfo("[WAITING] 물품 수령 확인을 위해 20초 대기합니다.")

        remaining_ticks = 40
        while remaining_ticks > 0 and not rospy.is_shutdown():
            if self.cancel_mission:
                return False
            if self.paused:
                self.wait_while_paused()
                continue
            if self.item_received:
                if has_next:
                    self.status_pub.publish("SCENARIO_8")
                    rospy.sleep(2.0)
                return True
            rospy.sleep(0.5)
            remaining_ticks -= 1

        self.status_pub.publish("SCENARIO_13")
        rospy.loginfo("[TIMEOUT] 물품 수령 확인 없음. 배송 실패 처리 후 다음 목적지로 넘어갑니다.")
        rospy.sleep(3.0)
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
            success = self.move_to_room(room)
            if not success:
                self.stop_robot()
                self.publish_status("IDLE")
                return

            has_next = index < len(normalized_rooms) - 1
            if not self.wait_for_item(room, has_next):
                return
            if has_next:
                self.backup_50cm()

        self.status_pub.publish("SCENARIO_9")
        rospy.sleep(3.0)
        if u"엘베" in locations:
            self.publish_status("RETURNING")
            self.move_to_room(u"엘베")
        self.publish_status("IDLE")

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
            self.paused = True
            self.client.cancel_goal()
            self.stop_robot()
            self.publish_status("PAUSED")
            return

        if cmd == "SCENARIO_22":
            if self.paused:
                self.paused = False
                if self.current_target:
                    self.publish_status("MOVING:{}".format(self.current_target))
            return

        if cmd == "SCENARIO_8":
            self.item_received = True
            return

        if cmd not in ["SCENARIO_1", "SCENARIO_2", "SCENARIO_3", "SCENARIO_4", "SCENARIO_6"]:
            return

        if self.active_thread and self.active_thread.is_alive():
            rospy.logwarn("Mission already running. New command ignored: %s", payload)
            return

        self.active_thread = threading.Thread(target=self.run_delivery_journey, args=(payload,))
        self.active_thread.daemon = True
        self.active_thread.start()

    def run_cli_goals(self, goals):
        for index, target in enumerate(goals):
            if rospy.is_shutdown():
                break
            room = normalize_room_name(target)
            if room not in locations:
                print("❌ '{}' 좌표가 데이터베이스에 없습니다.".format(target))
                continue
            success = self.move_to_room(room)
            if success and index < len(goals) - 1:
                rospy.sleep(2.0)
                self.backup_50cm()
        print("\n=== 🎉 모든 배달 주행 시퀀스 완전 종료 ===")

    def run(self):
        input_goals = sys.argv[1:]
        if input_goals:
            self.run_cli_goals(input_goals)
        else:
            print("🎙️ 음성 명령 대기 중: /llm_command 토픽을 기다립니다.")
            rospy.spin()


if __name__ == '__main__':
    try:
        DeliveryNavigator().run()
    except rospy.ROSInterruptException:
        pass
