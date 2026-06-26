#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import sys
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

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

def get_current_yaw():
    global tf_listener
    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        return tf.transformations.euler_from_quaternion(rot)[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None

# =========================================================
# 2. [안전 탈출 및 후진 로직]
# =========================================================
def backup_50cm():
    """배달 완료 후 좁은 문 앞 공간을 빠져나오기 위해 50cm 후진"""
    global cmd_vel_pub
    print("\n🚗 [안전 확보] 문 앞 공간을 빠져나오기 위해 50cm 후진합니다...")
    twist = Twist()
    twist.linear.x = -0.15  # 마그니 로봇이 미끄러지지 않는 안전한 후진 속도
    rate = rospy.Rate(10)
    
    # 0.15m/s * 35번(3.5초) = 약 52cm 후진
    for _ in range(35):
        if rospy.is_shutdown(): break
        cmd_vel_pub.publish(twist)
        rate.sleep()
        
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    rospy.sleep(1.0)
    print("✅ 후진 완료. 다음 목적지로 주행을 준비합니다.")

# =========================================================
# 3. [자율 주행 핵심 구동 로직]
# =========================================================
def move_to_goal(client, location_name):
    global tf_listener
    if location_name not in locations:
        print("❌ '{}' 좌표가 데이터베이스에 없습니다.".format(location_name))
        return False
        
    x, y, z_ori, w_ori = locations[location_name]

    # 목적지 좌표까지의 이동이 목표이므로 goal yaw는 현재 로봇 yaw를 유지한다.
    # 이렇게 하면 문 앞 최종 자세를 맞추려고 시작부터 제자리 회전하는 동작을 줄일 수 있다.
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

    print("\n>>> 🚀 [{}] 이동 시작...".format(location_name))
    client.send_goal(goal)
    client.wait_for_result()
    
    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        print("✅ [{}] 도착 완료!".format(location_name))
        return True
    else:
        print("❌ [{}] 도착 실패. (상태 코드: {})".format(location_name, state))
        return False

# =========================================================
# 4. [메인 루프]
# =========================================================
def main():
    global cmd_vel_pub, tf_listener
    rospy.init_node('navi_cmd_node')
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 실시간 좌표 변환용 리스너 세팅
    tf_listener = tf.TransformListener()
    rospy.sleep(1.0)

    input_goals = sys.argv[1:]
    if not input_goals: 
        print("사용법: rosrun magni_nav navi.py 544호 542호")
        return

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("⏳ 자율주행 서버 대기 중...")
    client.wait_for_server()

    try:
        for i, target in enumerate(input_goals):
            if rospy.is_shutdown(): break
            
            # 단계 1: 목적지 호수의 '중앙' 경유지가 데이터베이스에 있다면 먼저 직진 주행
            center_target = target + "_중앙"
            if center_target in locations:
                if not move_to_goal(client, center_target):
                    print("⚠️ [{}] 중앙 경유 실패. 최종 목적지 진입을 생략합니다.".format(target))
                    continue
                rospy.sleep(1.0)
            
            # 단계 2: 중앙 도착 후, 실제 최종 문 앞 목적지로 진입
            success = move_to_goal(client, target)
            
            # 단계 3: 성공적으로 문 앞에 도착했고, 뒤에 목적지가 더 남아있다면 안전하게 50cm 후진 탈출
            if success and i < len(input_goals) - 1:
                rospy.sleep(2.0)
                backup_50cm()
                
    except KeyboardInterrupt:
        print("\n🛑 사용자 강제 종료 수신")
        
    print("\n=== 🎉 모든 배달 주행 시퀀스 완전 종료 ===")

if __name__ == '__main__':
    main()
