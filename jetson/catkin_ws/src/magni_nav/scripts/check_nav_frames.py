#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


last_scan = None
last_odom = None


def yaw_from_quat(q):
    return tf.transformations.euler_from_quaternion(q)[2]


def scan_cb(msg):
    global last_scan
    last_scan = msg


def odom_cb(msg):
    global last_odom
    last_odom = msg


def main():
    rospy.init_node("check_nav_frames")
    listener = tf.TransformListener()
    rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
    rospy.Subscriber("/odom", Odometry, odom_cb, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        print("\n--- navigation frame check ---")

        if last_scan is None:
            print("/scan: no message yet")
        else:
            print(
                "/scan: frame_id={} stamp={:.3f} ranges={}".format(
                    last_scan.header.frame_id,
                    last_scan.header.stamp.to_sec(),
                    len(last_scan.ranges),
                )
            )

        if last_odom is None:
            print("/odom: no message yet")
        else:
            q = last_odom.pose.pose.orientation
            yaw = yaw_from_quat([q.x, q.y, q.z, q.w])
            print(
                "/odom: x={:.3f} y={:.3f} yaw={:.1f}deg vx={:.3f} wz={:.3f}".format(
                    last_odom.pose.pose.position.x,
                    last_odom.pose.pose.position.y,
                    math.degrees(yaw),
                    last_odom.twist.twist.linear.x,
                    last_odom.twist.twist.angular.z,
                )
            )

        for parent, child in [
            ("map", "base_footprint"),
            ("odom", "base_footprint"),
            ("base_footprint", "laser"),
        ]:
            try:
                trans, rot = listener.lookupTransform(parent, child, rospy.Time(0))
                yaw = yaw_from_quat(rot)
                print(
                    "tf {} -> {}: xyz=({:.3f}, {:.3f}, {:.3f}) yaw={:.1f}deg".format(
                        parent,
                        child,
                        trans[0],
                        trans[1],
                        trans[2],
                        math.degrees(yaw),
                    )
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                print("tf {} -> {}: {}".format(parent, child, exc))

        rate.sleep()


if __name__ == "__main__":
    main()
