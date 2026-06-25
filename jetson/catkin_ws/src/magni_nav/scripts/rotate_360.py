#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import math

def rotate_360():
    rospy.init_node('rotate_360_node', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 20Hz로 아주 촘촘하고 부드럽게 명령을 쏩니다
    rate = rospy.Rate(20) 
    vel_msg = Twist()
    
    # 초당 0.5 라디안의 안정적인 속도로 회전
    angular_speed = 0.5 
    # 360도는 2파이(2 * pi) 라디안입니다
    target_angle = 2 * math.pi 

    # 목표 각도에 도달하기 위한 시간 계산 (거리 = 속력 x 시간)
    duration = target_angle / angular_speed
    
    rospy.loginfo("🚀 360도 정밀 회전을 시작합니다! (약 %.1f초 소요)" % duration)
    rospy.loginfo("실제 로봇이 회전하는 동안 RViz 화면을 잘 지켜보세요.")
    
    start_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - start_time
        
        if elapsed_time < duration:
            vel_msg.angular.z = angular_speed
            pub.publish(vel_msg)
        else:
            break # 계산된 시간이 지나면 루프 탈출
        
        rate.sleep()

    # 정확히 시간이 되면 속도를 0으로 만들어 브레이크를 밟음
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)
    rospy.loginfo("🛑 회전 완료! 실제 로봇의 위치와 RViz의 라이다 선을 비교해 보세요.")

if __name__ == '__main__':
    try:
        rotate_360()
    except rospy.ROSInterruptException:
        pass
