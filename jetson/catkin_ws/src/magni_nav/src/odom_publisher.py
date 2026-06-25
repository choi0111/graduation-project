#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Point32

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher')
        
        # STM32와 맞춘 틱 수치
        self.left_ticks_per_rev = 3788.0
        self.right_ticks_per_rev = 3691.0
        
        self.wheel_radius = 0.125 / 2.0
        self.wheel_base = 0.695
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.last_time = rospy.Time.now()
        self.first_run = True
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # [핵심] STM32가 쏘는 'wheel_ticks (Point32)'를 받도록 세팅!
        rospy.Subscriber("wheel_ticks", Point32, self.ticks_callback)
        
        self.rate = rospy.Rate(20) 

    def ticks_callback(self, msg):
        # x에 담긴 왼쪽 틱, y에 담긴 오른쪽 틱을 꺼내옵니다.
        self.left_ticks = int(msg.x)
        self.right_ticks = int(msg.y)

    def update(self):
        current_time = rospy.Time.now()
        if self.first_run:
            self.last_left_ticks = self.left_ticks
            self.last_right_ticks = self.right_ticks
            self.last_time = current_time
            self.first_run = False
            return

        dt = (current_time - self.last_time).to_sec()
        if dt == 0:
            return

        delta_left_ticks = self.left_ticks - self.last_left_ticks
        delta_right_ticks = self.right_ticks - self.last_right_ticks

        # 16-bit 타이머 오버플로우 방지
        if delta_left_ticks > 32768: delta_left_ticks -= 65536
        elif delta_left_ticks < -32768: delta_left_ticks += 65536
        if delta_right_ticks > 32768: delta_right_ticks -= 65536
        elif delta_right_ticks < -32768: delta_right_ticks += 65536

        # 이동 거리 계산
        distance_left = (delta_left_ticks / self.left_ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        distance_right = (delta_right_ticks / self.right_ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        # 차동 구동 역학 공식
        distance = (distance_right + distance_left) / 2.0
        d_th = (distance_right - distance_left) / self.wheel_base

        delta_x = distance * math.cos(self.th)
        delta_y = distance * math.sin(self.th)
        delta_th = d_th

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        vx = distance / dt
        vth = d_th / dt

        # TF 발행
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # Odom 발행
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

        self.odom_pub.publish(odom)

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time

    def spin(self):
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        odom = OdomPublisher()
        odom.spin()
    except rospy.ROSInterruptException:
        pass
