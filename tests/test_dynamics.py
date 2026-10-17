from dataclasses import replace
import math
from pathlib import Path
import struct
import tempfile
import unittest

from openasc.config import ContentError
from openasc.dynamics import Controls, Dynamics, DT
from openasc.ground import Ground
from openasc.kn5 import IDENTITY, Mesh
from openasc.placement import spawn_transform
from openasc.surfaces import parse_surfaces
from openasc.vehicle import load_vehicle
from vehicle_fixtures import install_car


def plane(size=1000, friction=.9, slope=0):
    surface = parse_surfaces(f'[SURFACE_0]\nKEY=ROAD\nFRICTION={friction}\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0')[0]['ROAD']
    points = [(-size, -size * slope, -size), (size, -size * slope, -size),
              (-size, size * slope, size), (size, size * slope, size)]
    vertices = b''.join(struct.pack('<11f', *p, 0, 1, 0, 0, 0, 1, 0, 0) for p in points)
    mesh = Mesh('1ROAD', 0, IDENTITY, vertices, struct.pack('<6H', 0, 2, 1, 1, 2, 3), True, True, True, (0, 0), None)
    ground = Ground()
    ground.add_mesh(mesh, IDENTITY, surface, 'original.kn5')
    ground.build()
    return ground


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        root = Path(temp.name)
        install_car(root)
        self.vehicle = load_vehicle(root, 'fixture')
        world = list(IDENTITY)
        world[13] = 2
        self.spawn = spawn_transform('AC_PIT_0', 'original.kn5', world)

    def model(self, **kwargs):
        return Dynamics(self.vehicle, plane(**kwargs), self.spawn)

    def run_steps(self, model, seconds, controls):
        model.set_controls(controls)
        model.resume()
        for _ in range(round(seconds / DT)):
            model.step()

    def test_input_schema_rejects_nonfinite_partial_boolean_and_unsupported_gears(self):
        valid = dict(throttle=1, brake=0, steer=-1, handbrake=0, gear=3)
        self.assertEqual(Controls.from_dict(valid, 3).gear, 3)
        for update in ({'throttle': math.nan}, {'throttle': 10 ** 1000}, {'brake': True}, {'steer': 2}, {'gear': 4}, {'gear': 1.5}):
            with self.assertRaises(ContentError):
                Controls.from_dict(valid | update, 3)
        with self.assertRaises(ContentError):
            Controls.from_dict({'throttle': 1}, 3)
        for invalid in (None, [], [{}]):
            with self.assertRaises(ContentError):
                Controls.from_dict(invalid, 3)

    def test_stationary_neutral_pause_and_reset(self):
        model = self.model()
        self.assertTrue(model.paused)
        model.step()
        self.assertEqual(model.tick, 0)
        self.run_steps(model, 2, Controls(throttle=1, gear=0))
        self.assertEqual((model.x, model.z, model.vx, model.vz), (0, 0, 0, 0))
        self.assertAlmostEqual(model.y, .536)
        model.pause()
        tick = model.tick
        model.step()
        self.assertEqual(model.tick, tick)
        model.reset()
        self.assertEqual(model.tick, 0)
        self.assertTrue(model.paused)

    def test_forward_acceleration_and_braking_dissipate_motion(self):
        model = self.model()
        self.run_steps(model, 3, Controls(throttle=1))
        self.assertGreater(model.z, 5)
        self.assertGreater(model.longitudinal_speed(), 4)
        self.assertAlmostEqual(model.x, 0)
        self.assertAlmostEqual(model.yaw, 0)
        model.set_controls(Controls(brake=1))
        previous = math.hypot(model.vx, model.vz)
        for _ in range(600):
            model.step()
            current = math.hypot(model.vx, model.vz)
            self.assertLessEqual(current, previous + 1e-9)
            previous = current
        self.assertEqual(previous, 0)

    def test_left_right_symmetry_and_reverse_steering_direction(self):
        left, right, reverse = self.model(), self.model(), self.model()
        self.run_steps(left, 3, Controls(throttle=.7, steer=.35))
        self.run_steps(right, 3, Controls(throttle=.7, steer=-.35))
        self.assertGreater(left.x, 0)
        self.assertGreater(left.yaw, 0)
        self.assertAlmostEqual(left.x, -right.x, places=10)
        self.assertAlmostEqual(left.z, right.z, places=10)
        self.assertAlmostEqual(left.yaw, -right.yaw, places=10)
        self.run_steps(reverse, 3, Controls(throttle=.7, steer=.35, gear=-1))
        self.assertLess(reverse.longitudinal_speed(), 0)
        self.assertLess(reverse.yaw, 0)
        with self.assertRaises(ContentError):
            left.set_controls(Controls(gear=-1))

    def test_contact_force_circle_and_low_grip_acceleration(self):
        low, normal = self.model(friction=.1), self.model()
        self.run_steps(normal, 2, Controls(throttle=1))
        self.run_steps(low, 2, Controls(throttle=1))
        self.assertLess(low.longitudinal_speed(), normal.longitudinal_speed())
        low.set_controls(Controls(throttle=1, steer=1, brake=.5))
        for _ in range(300):
            low.step()
            for force in low.forces:
                self.assertLessEqual(math.hypot(force['longitudinal_n'], force['lateral_n']), force['limit_n'] + 1e-9)

    def test_repeatable_controls_do_not_depend_on_render_frames(self):
        first, second = self.model(), self.model()
        for controls, duration in [(Controls(throttle=.8), 2), (Controls(throttle=.3, steer=.2), 1), (Controls(brake=.8), 2)]:
            self.run_steps(first, duration, controls)
            self.run_steps(second, duration, controls)
        self.assertEqual(first.snapshot(), second.snapshot())
        self.assertEqual(first.tick, 600)

    def test_lost_support_stops_before_leaving_the_ground_and_slope_changes_pose(self):
        model = self.model(size=5)
        self.run_steps(model, 5, Controls(throttle=1))
        self.assertTrue(model.paused)
        self.assertIn('drive_no_support', model.reason)
        self.assertEqual((model.vx, model.vz), (0, 0))
        model.support(model.x, model.z, model.yaw, model.y + 2)
        sloped = self.model(slope=.05)
        self.run_steps(sloped, .5, Controls(gear=0))
        self.assertLess(sloped.z, 0)
        world = sloped.world()
        self.assertLess(world[6], 0)
        self.assertAlmostEqual(math.hypot(*world[4:7]), 1)

    def test_unsupported_vehicle_features_fail_before_starting(self):
        vehicle = replace(self.vehicle, drivetrain=self.vehicle.drivetrain | {'traction': 'AWD'})
        with self.assertRaises(ContentError):
            Dynamics(vehicle, plane(), self.spawn)

    def test_brakes_can_hold_a_grippy_slope_but_cannot_stop_frictionless_motion(self):
        hill = self.model(slope=.2)
        self.run_steps(hill, 1, Controls(brake=1))
        self.assertAlmostEqual(hill.z, 0, places=8)
        self.assertAlmostEqual(hill.longitudinal_speed(), 0, places=8)
        ice = self.model(friction=0)
        ice.vz = .03
        self.run_steps(ice, .1, Controls(brake=1))
        self.assertGreater(ice.vz, .029)
