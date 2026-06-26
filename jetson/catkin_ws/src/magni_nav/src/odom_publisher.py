#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import tf
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomPublisher:
    def __init__(self):
        rospy.init_node("odom_publisher")

        self.left_ticks_per_rev = rospy.get_param("~left_ticks_per_rev", 3788.0)
        self.right_ticks_per_rev = rospy.get_param("~right_ticks_per_rev", 3691.0)
        self.wheel_diameter = rospy.get_param("~wheel_diameter", 0.125)
        self.wheel_base = rospy.get_param("~wheel_base", 0.695)
        self.left_tick_sign = rospy.get_param("~left_tick_sign", 1.0)
        self.right_tick_sign = rospy.get_param("~right_tick_sign", 1.0)

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.rate_hz = rospy.get_param("~rate", 30.0)

        self.wheel_radius = self.wheel_diameter / 2.0

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.have_ticks = False
        self.first_update = True

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("wheel_ticks", Point32, self.ticks_callback, queue_size=10)

        rospy.loginfo(
            "odom_publisher: left_ticks_per_rev=%.3f right_ticks_per_rev=%.3f "
            "wheel_base=%.3f wheel_diameter=%.3f",
            self.left_ticks_per_rev,
            self.right_ticks_per_rev,
            self.wheel_base,
            self.wheel_diameter,
        )

    def ticks_callback(self, msg):
        self.left_ticks = int(msg.x * self.left_tick_sign)
        self.right_ticks = int(msg.y * self.right_tick_sign)
        self.have_ticks = True

    def update(self):
        current_time = rospy.Time.now()

        if not self.have_ticks:
            self.last_time = current_time
            return

        if self.first_update:
            self.last_left_ticks = self.left_ticks
            self.last_right_ticks = self.right_ticks
            self.last_time = current_time
            self.first_update = False
            return

        dt = (current_time - self.last_time).to_sec()
        if dt <= 0.0:
            return

        delta_left_ticks = self.left_ticks - self.last_left_ticks
        delta_right_ticks = self.right_ticks - self.last_right_ticks

        distance_left = (
            float(delta_left_ticks)
            / self.left_ticks_per_rev
            * (2.0 * math.pi * self.wheel_radius)
        )
        distance_right = (
            float(delta_right_ticks)
            / self.right_ticks_per_rev
            * (2.0 * math.pi * self.wheel_radius)
        )

        distance = (distance_right + distance_left) / 2.0
        d_th = (distance_right - distance_left) / self.wheel_base

        heading_mid = self.th + d_th / 2.0
        self.x += distance * math.cos(heading_mid)
        self.y += distance * math.sin(heading_mid)
        self.th = normalize_angle(self.th + d_th)

        vx = distance / dt
        vth = d_th / dt

        odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)

        if self.publish_tf:
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                odom_quat,
                current_time,
                self.base_frame,
                self.odom_frame,
            )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = Pose(Point(self.x, self.y, 0.0), Quaternion(*odom_quat))
        odom.twist.twist = Twist(Vector3(vx, 0.0, 0.0), Vector3(0.0, 0.0, vth))

        # Nonzero covariance helps AMCL and RViz understand this is wheel odom,
        # not a perfect pose measurement.
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks
        self.last_time = current_time

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()


if __name__ == "__main__":
    try:
        OdomPublisher().spin()
    except rospy.ROSInterruptException:
        pass
