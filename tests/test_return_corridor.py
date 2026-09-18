import math
import ast
import pathlib
import sys
import threading
import types
import unittest

SCRIPTS = pathlib.Path(__file__).resolve().parents[1] / 'jetson/catkin_ws/src/magni_nav/scripts'
sys.path.insert(0, str(SCRIPTS))
from return_corridor import ReturnCorridor, fit_wall


def twist():
    return types.SimpleNamespace(linear=types.SimpleNamespace(x=0.),
                                 angular=types.SimpleNamespace(z=0.))


def ros_stub():
    return types.SimpleNamespace(loginfo_throttle=lambda *args: None,
                                 loginfo=lambda *args: None,
                                 logerr=lambda *args: None,
                                 logwarn=lambda *args: None,
                                 sleep=lambda _: None,
                                 is_shutdown=lambda: False)


def walls(pose, upper=1.18, lower=-1.18):
    x, y, yaw = pose
    result = []
    for sign in (1, -1):
        points = []
        for deg in range(70, 111):
            a = math.radians(sign*deg)
            dy = math.sin(yaw+a)
            level = upper if dy > 0 else lower
            r = (level-y)/dy
            if r > 0:
                points.append((r*math.cos(a), r*math.sin(a)))
        result.append(points)
    return result


class ReturnTests(unittest.TestCase):
    def seed(self, pose=(0., 0., 0.)):
        c = ReturnCorridor()
        for _ in range(8):
            c.observe(*walls(pose), pose)
        self.assertIsNotNone(c.axis)
        return c

    def test_needs_confirmed_normal_walls(self):
        c = ReturnCorridor()
        for _ in range(20):
            c.observe(*walls((0., 0., 0.), upper=1.355), (0., 0., 0.))
        self.assertIsNone(c.axis)
        self.assertEqual(c.command((0., 0., 0.), .1)[:2], (0., 0.))

    def test_slanted_wall_is_fitted_not_rejected_as_open(self):
        points, _ = walls((0., 0., math.radians(18)))
        line = fit_wall(points)
        self.assertIsNotNone(line)
        self.assertAlmostEqual(line[0], -math.radians(18))

    def test_reverse_swaps_steering_sign(self):
        c = self.seed()
        a = c.command((0., .2, 0.), .1)
        b = c.command((0., .2, math.pi), .1)
        self.assertLess(a[1], 0)
        self.assertGreater(b[1], 0)

    def test_door_recess_does_not_move_center(self):
        c = self.seed()
        pose = (1., 0., math.pi)
        for _ in range(20):
            c.observe(*walls(pose, upper=1.355), pose)
        self.assertAlmostEqual(c.center[1], 0.)
        self.assertGreater(c.command(pose, .1)[0], 0)

    def test_both_recesses_are_not_trusted(self):
        c = self.seed()
        pose = (1., 0., math.pi)
        c.observe(*walls(pose, upper=1.355, lower=-1.355), pose)
        self.assertEqual(c.command(pose, .1)[:2], (0., 0.))

    def test_opening_cannot_allow_unbounded_blind_travel(self):
        c = self.seed()
        self.assertGreater(c.command((.4, 0., math.pi), .1)[0], 0)
        self.assertEqual(c.command((.51, 0., math.pi), .1)[:2], (0., 0.))

    def test_large_misalignment_holds_without_rotating(self):
        c = self.seed()
        self.assertEqual(c.command((0., 0., math.radians(50)), .1)[:2], (0., 0.))

    def test_closed_loop_both_directions_remain_inside_corridor(self):
        for initial_yaw in (0., math.pi):
            c = self.seed()
            x, y, yaw = 0., .25, initial_yaw + math.radians(10)
            for _ in range(1800):
                pose = (x, y, yaw)
                c.observe(*walls(pose), pose)
                v, w, _ = c.command(pose, .1)
                self.assertGreater(v, 0)
                self.assertLess(abs(y), .40)
                x += v*math.cos(yaw)*.1
                y += v*math.sin(yaw)*.1
                yaw += w*.1
            self.assertLess(abs(y), .04)

    def test_navigation_heading_is_not_cancelled_by_lateral_error(self):
        tree = ast.parse(
            (SCRIPTS/'corridor_centering.py').read_text(encoding='utf-8'))
        function = next(
            n for n in tree.body
            if isinstance(n, ast.FunctionDef) and
            n.name == 'navigation_steering_correction')
        env = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     '<navigation-steering>', 'exec'), env)
        correction, applied_lateral = env[
            'navigation_steering_correction'](
                -.060, .057, math.radians(8.7), math.radians(4.0),
                .015, .060)
        self.assertAlmostEqual(applied_lateral, -.015)
        self.assertGreater(correction, .03)

    def test_navigation_uses_full_lateral_trim_when_heading_is_straight(self):
        tree = ast.parse(
            (SCRIPTS/'corridor_centering.py').read_text(encoding='utf-8'))
        function = next(
            n for n in tree.body
            if isinstance(n, ast.FunctionDef) and
            n.name == 'navigation_steering_correction')
        env = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     '<navigation-steering>', 'exec'), env)
        correction, applied_lateral = env[
            'navigation_steering_correction'](
                -.050, 0., math.radians(1.0), math.radians(4.0),
                .015, .060)
        self.assertAlmostEqual(applied_lateral, -.050)
        self.assertAlmostEqual(correction, -.050)

    def test_normal_corridor_acquisition_prefers_two_flat_walls(self):
        tree = ast.parse(
            (SCRIPTS/'corridor_centering.py').read_text(encoding='utf-8'))
        function = next(
            n for n in tree.body
            if isinstance(n, ast.FunctionDef) and
            n.name == 'normal_corridor_acquisition_candidate')
        env = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]),
                     '<corridor-acquisition>', 'exec'), env)
        candidate = env['normal_corridor_acquisition_candidate']

        # 2.445 m overlaps the configured doorway-width band, but two flat
        # continuous walls still identify the normal corridor reliably.
        self.assertTrue(candidate(True, True, True))
        self.assertFalse(candidate(True, True, False))
        self.assertFalse(candidate(False, True, True))

    def return_node(self):
        tree = ast.parse((SCRIPTS/'corridor_centering.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        env = dict(math=math, time=types.SimpleNamespace(time=lambda: 10.),
                   rospy=ros_stub(), Twist=twist, ReturnCorridor=ReturnCorridor)
        exec(compile(ast.Module(body=[cls], type_ignores=[]), '<node>', 'exec'), env)
        node = env['CorridorCentering'].__new__(env['CorridorCentering'])
        node.lock = threading.Lock()
        node.return_pose = (0., 0., math.pi)
        node.return_odom_time = node.last_scan_wall_time = 10.
        node.scan_timeout = .5
        node.robot_half_width, node.minimum_edge_clearance = .385, .25
        node.safety_left_distance = node.safety_right_distance = 1.18
        node.return_front_clear = True
        node.return_controller = self.seed()
        node.output = []
        node.cmd_pub = types.SimpleNamespace(publish=node.output.append)
        return node

    def test_return_topic_fails_closed_on_stale_scan(self):
        node = self.return_node()
        node.last_scan_wall_time = 8.
        cmd = twist()
        cmd.linear.x = .1
        node.return_cmd_callback(cmd)
        self.assertEqual(node.output[-1].linear.x, 0.)

    def test_return_topic_stops_near_wall_without_forced_turn(self):
        node = self.return_node()
        node.safety_right_distance = .476
        cmd = twist()
        cmd.linear.x = .1
        node.return_cmd_callback(cmd)
        self.assertEqual(node.output[-1].linear.x, 0.)
        self.assertEqual(node.output[-1].angular.z, 0.)

    def test_stop_clears_watchdog_and_forward_command(self):
        node = self.return_node()
        cmd = twist()
        cmd.linear.x = .1
        node.return_cmd_callback(cmd)
        self.assertGreater(node.output[-1].linear.x, 0.)
        node.return_cmd_callback(twist())
        self.assertEqual(node.output[-1].linear.x, 0.)
        self.assertIsNone(node.return_command_time)

    def test_return_navigation_does_not_force_map_heading_realign(self):
        tree = ast.parse((SCRIPTS/'navi.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if getattr(n, 'name', '') == 'drive_corridor_to_home')
        env = dict(math=math, time=types.SimpleNamespace(time=lambda: 10.),
                   rospy=ros_stub(), Twist=twist, console_text=str,
                   normalize_angle=lambda a: math.atan2(math.sin(a), math.cos(a)))
        for n in tree.body:
            if isinstance(n, ast.Assign) and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name):
                name = n.targets[0].id
                if name.startswith(('HOME_', 'ODOM_', 'AMCL_', 'GOAL_PROGRESS_')):
                    env[name] = eval(compile(ast.Expression(n.value), '<constant>', 'eval'), env)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<return>', 'exec'), env)
        poses = iter([(3., 0.), (1.8, 0.)])
        robot = types.SimpleNamespace(home_pose=(0., 0.), amcl_position=(5., 0.),
            amcl_yaw=math.pi-.6, cancel_mission=False, paused=False,
            last_odom_wall_time=10., last_amcl_wall_time=10.,
            cancel_goal_if_active=lambda: None, stop_corridor_drive=lambda: None,
            wait_for_fresh_odom=lambda _: True, refresh_localization_from_tf=lambda: True)
        env['rospy'].Rate = lambda _: types.SimpleNamespace(
            sleep=lambda: setattr(robot, 'amcl_position', next(poses)))
        output = []
        env['cmd_vel_nav_pub'] = types.SimpleNamespace(publish=output.append)
        self.assertTrue(env['drive_corridor_to_home'](robot, position_tolerance=1.9,
                                                     corridor_yaw=math.pi))
        self.assertEqual(len(output), 2)
        self.assertTrue(all(cmd.angular.z == 0. for cmd in output))


if __name__ == '__main__':
    unittest.main()
