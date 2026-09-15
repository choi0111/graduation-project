#!/usr/bin/env python
"""Odom-frame wall reference for straight corridor returns (ROS independent)."""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def fit_wall(points):
    if len(points) < 8:
        return None
    xs, ys = zip(*points)
    if max(xs) - min(xs) < 0.25:
        return None
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    variance = sum((x - mx) ** 2 for x in xs)
    if variance < 1e-6:
        return None
    slope = sum((x - mx) * (y - my) for x, y in points) / variance
    intercept = my - slope * mx
    scale = math.sqrt(1.0 + slope * slope)
    residual = math.sqrt(sum((y - slope*x - intercept) ** 2
                             for x, y in points) / len(points)) / scale
    if residual > 0.025:
        return None
    return math.atan(slope), intercept


class ReturnCorridor(object):
    def __init__(self, width=2.36):
        self.width = width
        self.axis = None
        self.center = None
        self.confirmations = 0
        self.last_observed_pose = None

    def observe(self, left_points, right_points, pose):
        left, right = fit_wall(left_points), fit_wall(right_points)
        x, y, yaw = pose
        pair = (left is not None and right is not None and
                abs(wrap(left[0] - right[0])) < math.radians(5))
        if pair:
            heading = (left[0] + right[0]) * 0.5
            width = (left[1] - right[1]) * math.cos(heading)
            pair = abs(width - self.width) <= 0.08
        if pair:
            axis = wrap(yaw + heading)
            center = (x - math.sin(axis) * (left[1]+right[1]) *
                      0.5 * math.cos(heading),
                      y + math.cos(axis) * (left[1]+right[1]) *
                      0.5 * math.cos(heading))
            if self.axis is not None:
                if math.cos(axis - self.axis) < 0:
                    axis = wrap(axis + math.pi)
                if abs(wrap(axis - self.axis)) > math.radians(12):
                    self.confirmations = 0
                    return
            self.confirmations += 1
            if self.axis is None and self.confirmations < 8:
                return
            self.axis = axis if self.axis is None else wrap(
                self.axis + 0.15 * wrap(axis - self.axis))
            self.center = center
            self.last_observed_pose = (x, y)
            return
        self.confirmations = 0
        if self.axis is None:
            return
        # Match a visible line to the previously observed physical wall, not
        # to whichever surface happens to be closest to half the corridor width.
        nx, ny = -math.sin(self.axis), math.cos(self.axis)
        matches = []
        for wall in (left, right):
            if wall is None:
                continue
            axis = wrap(yaw + wall[0])
            if math.cos(axis - self.axis) < 0:
                axis = wrap(axis + math.pi)
            if abs(wrap(axis - self.axis)) > math.radians(8):
                continue
            wx = x - math.sin(yaw) * wall[1]
            wy = y + math.cos(yaw) * wall[1]
            offset = (wx-self.center[0])*nx + (wy-self.center[1])*ny
            error = abs(abs(offset) - self.width*0.5)
            if error <= 0.10:
                matches.append((error, axis, offset))
        if matches:
            _, axis, offset = min(matches)
            shift = offset - math.copysign(self.width*0.5, offset)
            self.center = (self.center[0]+0.15*shift*nx,
                           self.center[1]+0.15*shift*ny)
            self.axis = wrap(self.axis+0.15*wrap(axis-self.axis))
            self.last_observed_pose = (x, y)

    def command(self, pose, speed):
        if self.axis is None:
            return 0.0, 0.0, 'waiting for measured corridor'
        x, y, yaw = pose
        if math.hypot(x-self.last_observed_pose[0],
                      y-self.last_observed_pose[1]) > 0.50:
            return 0.0, 0.0, 'wall reference lost beyond 0.50 m'
        axis = self.axis
        if math.cos(axis-yaw) < 0:
            axis = wrap(axis+math.pi)
        heading = wrap(axis-yaw)
        if abs(heading) > math.radians(35):
            return 0.0, 0.0, 'not aligned with measured corridor'
        lateral = ((self.center[0]-x)*-math.sin(axis) +
                   (self.center[1]-y)*math.cos(axis))
        # Steer gently toward the center, then parallel to the measured wall.
        trim = max(-math.radians(8), min(math.radians(8), 0.35*lateral))
        angular = max(-0.06, min(0.06, 0.8*(heading+trim)))
        if abs(lateral) > 0.12 or abs(heading) > math.radians(4):
            speed = min(speed, 0.04)
        return speed, angular, 'tracking measured corridor'
