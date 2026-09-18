"""Exercise production scan and command callbacks with geometric scans."""
import ast
import math
import pathlib
import threading
import time
import types
import unittest

from test_return_corridor import SCRIPTS, ReturnCorridor, fit_wall


def twist():
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0., y=0., z=0.),
        angular=types.SimpleNamespace(x=0., y=0., z=0.))


def node():
    source = ast.parse((SCRIPTS / 'corridor_centering.py').read_text(encoding='utf-8'))
    body = [n for n in source.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
    output = []
    noop = lambda *a, **kw: None
    ros = types.SimpleNamespace(
        init_node=noop, get_param=lambda key, default: default,
        Publisher=lambda *a, **kw: types.SimpleNamespace(publish=output.append),
        Subscriber=noop, Timer=noop, Duration=lambda v: v,
        loginfo=noop, logwarn=noop, loginfo_throttle=noop, logwarn_throttle=noop)
    env = dict(math=math, threading=threading, time=time, rospy=ros,
               Twist=twist, LaserScan=object, Odometry=object, Empty=object,
               ReturnCorridor=ReturnCorridor, fit_wall=fit_wall)
    exec(compile(ast.Module(body=body, type_ignores=[]), '<centering>', 'exec'), env)
    result = env['CorridorCentering']()
    result.door_recess_width_tolerance = .10
    result.width_learning_alpha = 0.
    result.maximum_output_angular_speed = .06
    result.output = output
    return result


def scan(y=.2, yaw=0., upper=1.22, lower=-1.22, zero_start=False):
    start = 0. if zero_start else -math.pi
    step = math.pi / 360
    ranges = []
    for index in range(720):
        dy = math.sin(start + index * step + yaw)
        r = ((upper if dy > 0 else lower) - y) / dy if abs(dy) > 1e-8 else float('inf')
        ranges.append(r if r > 0 else float('inf'))
    return types.SimpleNamespace(angle_min=start, angle_increment=step,
                                 ranges=ranges, range_min=.05, range_max=20.)


class ScanTests(unittest.TestCase):
    def test_closed_loop_converges_from_either_side_in_both_directions(self):
        for direction in (0., math.pi):
            for initial_y in (-.35, .35):
                n = node()
                y, yaw = initial_y, direction
                cmd = twist()
                cmd.linear.x = .1
                for _ in range(10):
                    n.scan_callback(scan(y=y, yaw=yaw, upper=1.18, lower=-1.18))
                self.assertTrue(n.normal_corridor_seen)
                for _ in range(1000):
                    n.scan_callback(scan(y=y, yaw=yaw, upper=1.18, lower=-1.18))
                    n.cmd_callback(cmd)
                    result = n.output[-1]
                    yaw += result.angular.z * .1
                    y += result.linear.x * math.sin(yaw) * .1
                    self.assertLess(abs(y), .40)
                self.assertLess(abs(y), .06)

    def test_unsafe_room_rotation_does_not_restart_driving(self):
        tree = ast.parse((SCRIPTS / 'navi.py').read_text(encoding='utf-8'))
        method = next(n for n in ast.walk(tree)
                      if isinstance(n, ast.FunctionDef) and
                      n.name == 'ensure_room_rotation_clearance')
        env = dict(rospy=types.SimpleNamespace(logerr=lambda *a: None),
                   FRONT_SCAN_WAIT_TIMEOUT=1., ROTATION_CLEARANCE_RADIUS=.677,
                   console_text=str)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<clearance>', 'exec'), env)
        stopped = []
        robot = types.SimpleNamespace(
            wait_for_fresh_front_scan=lambda timeout: True,
            rotation_clearance_is_safe=lambda: False,
            rotation_clearance_distance=.660,
            stop_robot=lambda: stopped.append(True))
        self.assertFalse(env['ensure_room_rotation_clearance'](
            robot, '545ho', '545ho_center', None))
        self.assertEqual(stopped, [True])
        robot.rotation_clearance_is_safe = lambda: True
        self.assertTrue(env['ensure_room_rotation_clearance'](
            robot, '545ho', '545ho_center', None))

    def test_acquired_normal_corridor_stays_active_and_steers_away_from_left(self):
        for zero_start in (False, True):
            n = node()
            for _ in range(30):
                n.scan_callback(scan(zero_start=zero_start))
            self.assertTrue(n.normal_corridor_seen)
            self.assertEqual(n.wall_mode, 'both')
            cmd = twist()
            cmd.linear.x = .1
            n.cmd_callback(cmd)
            self.assertLess(n.output[-1].angular.z, 0.)

    def test_slanted_scan_uses_perpendicular_width(self):
        n = node()
        for _ in range(30):
            n.scan_callback(scan(yaw=math.radians(18)))
        self.assertEqual(n.wall_mode, 'both')
        self.assertAlmostEqual(n.current_corridor_width, 2.44, places=5)

    def test_door_uses_opposite_intact_wall(self):
        for upper, lower, mode in ((1.355, -1.18, 'right'),
                                   (1.18, -1.355, 'left')):
            n = node()
            for _ in range(10):
                n.scan_callback(scan(y=0., upper=1.18, lower=-1.18))
            for _ in range(30):
                n.scan_callback(scan(y=0., upper=upper, lower=lower))
                self.assertEqual(n.wall_mode, mode)

    def test_initial_open_area_not_acquired(self):
        n = node()
        for _ in range(30):
            n.scan_callback(scan(upper=3., lower=-1.18))
        self.assertFalse(n.normal_corridor_seen)


if __name__ == '__main__':
    unittest.main()
