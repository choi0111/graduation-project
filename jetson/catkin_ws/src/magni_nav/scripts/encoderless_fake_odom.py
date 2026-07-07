#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


class EncoderlessFakeOdom(object):
    def __init__(self):
        rospy.init_node("encoderless_fake_odom")

        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.4))

        # Tune these if the encoderless DC motors move slower/faster than cmd_vel.
        self.linear_scale = float(rospy.get_param("~linear_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))

        # Keep this conservative. Encoderless odom is only a test aid.
        self.max_linear = float(rospy.get_param("~max_linear", 0.08))
        self.max_angular = float(rospy.get_param("~max_angular", 0.20))

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0

        now = rospy.Time.now()
        self.last_time = now
        self.last_cmd_time = now

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=20)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=10)

        rospy.logwarn(
            "encoderless_fake_odom is running. This is for temporary encoderless "
            "DC motor tests only; use odom_publisher.py again for final driving."
        )

    def clamp(self, value, limit):
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def cmd_callback(self, msg):
        self.vx = self.clamp(msg.linear.x * self.linear_scale, self.max_linear)
        self.vth = self.clamp(msg.angular.z * self.angular_scale, self.max_angular)
        self.last_cmd_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
                vx = 0.0
                vth = 0.0
            else:
                vx = self.vx
                vth = self.vth

            delta_x = vx * math.cos(self.th) * dt
            delta_y = vx * math.sin(self.th) * dt
            delta_th = vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))

            odom_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.th)

            self.tf_broadcaster.sendTransform(
                (self.x, self.y, 0.0),
                odom_quat,
                now,
                self.base_frame,
                self.odom_frame,
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = vx
            odom.twist.twist.angular.z = vth

            # Large covariance tells localization not to over-trust fake odom.
            odom.pose.covariance[0] = 0.25
            odom.pose.covariance[7] = 0.25
            odom.pose.covariance[35] = 0.50
            odom.twist.covariance[0] = 0.25
            odom.twist.covariance[35] = 0.50

            self.odom_pub.publish(odom)
            rate.sleep()


if __name__ == "__main__":
    try:
        EncoderlessFakeOdom().spin()
    except rospy.ROSInterruptException:
        pass
