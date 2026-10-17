from pathlib import Path
import math
import random
import struct
import tempfile
import unittest
from unittest.mock import patch

from openasc.config import ContentError
from openasc.kn5 import IDENTITY, Mesh
from openasc.walls import Body, Pose, Walls, SKIN, separation, swept_minimum, load_body, resolve_contacts
from openasc.vehicle import load_vehicle
from vehicle_fixtures import install_car
from openasc.dynamics import Dynamics, Controls, DT
from openasc.placement import spawn_transform
from test_dynamics import plane
from kn5_fixtures import file_bytes, mesh


def wall_mesh(points, indices=(0, 1, 2, 0, 2, 3), name='01WALL'):
    vertices = b''.join(struct.pack('<11f', *p, 0, 0, 1, 0, 0, 1, 0, 0) for p in points)
    return Mesh(name, 0, IDENTITY, vertices, struct.pack(f'<{len(indices)}H', *indices), True, False, False, (0, 0), None)


def walls_at(z=5, low=-1, high=3, x0=-10, x1=10):
    walls = Walls()
    walls.add_mesh(wall_mesh(((x0, low, z), (x1, low, z), (x1, high, z), (x0, high, z))), IDENTITY, 'wall.kn5')
    walls.build()
    return walls


class WallGeometryTests(unittest.TestCase):
    def setUp(self):
        self.body = Body((0, 0, 0), (1, .5, 2))

    def test_fast_translation_detects_thin_wall_from_both_sides(self):
        walls = walls_at()
        for start, end, expected_normal in [(0, 100, -1), (10, -100, 1)]:
            hits = walls.sweep(self.body, Pose((0, 1, start), 0), Pose((0, 1, end), 0))
            self.assertTrue(hits)
            self.assertAlmostEqual(hits[0]['normal'][2], expected_normal)
            expected = ((3 - SKIN) / 100) if start == 0 else ((start - 7 - SKIN) / (start - end))
            self.assertAlmostEqual(hits[0]['fraction'], expected, places=7)
            pose = Pose((0, 1, start), 0).between(Pose((0, 1, end), 0), hits[0]['fraction'])
            self.assertFalse(walls.overlaps(self.body, pose))
            self.assertGreaterEqual(len(hits), 2)

    def test_height_filter_and_finite_triangle_ends_do_not_create_infinite_walls(self):
        walls = walls_at(low=5, high=7)
        self.assertFalse(walls.sweep(self.body, Pose((0, 0, 0), 0), Pose((0, 0, 10), 0)))
        walls = walls_at(x0=-1, x1=1)
        self.assertFalse(walls.sweep(self.body, Pose((4, 1, 0), 0), Pose((4, 1, 10), 0)))
        self.assertTrue(walls.sweep(self.body, Pose((1.9, 1, 0), 0), Pose((1.9, 1, 10), 0)))

    def test_rotating_corner_hits_wall_even_when_both_endpoints_are_clear(self):
        walls = walls_at(z=2.15)
        start, end = Pose((0, 1, 0), 0.), Pose((0, 1, 0), 1.1)
        self.assertFalse(walls.overlaps(self.body, start))
        self.assertFalse(walls.overlaps(self.body, end))
        hits = walls.sweep(self.body, start, end)
        self.assertTrue(hits)
        self.assertGreater(hits[0]['fraction'], 0)
        self.assertLess(hits[0]['fraction'], .5)

    def test_tangent_and_separating_motion_are_allowed_at_existing_skin_contact(self):
        walls = walls_at()
        pose = Pose((0, 1, 3 - SKIN), 0)
        self.assertFalse(walls.sweep(self.body, pose, Pose((2, 1, 3 - SKIN), 0)))
        self.assertFalse(walls.sweep(self.body, pose, Pose((0, 1, 2), 0)))
        self.assertTrue(walls.sweep(self.body, pose, Pose((0, 1, 4), 0)))
        self.assertTrue(walls.sweep(self.body, pose, Pose((0, 1, 3 - SKIN), .3)))

    def test_projection_extrema_bound_dense_rotation_samples(self):
        body = Body((.2, .1, -.3), (1, .5, 2))
        start, end = Pose((3, 2, 4), 2.9), Pose((2, 2.1, 3), -2.8)
        axis = (.6, 0, .8)
        exact = swept_minimum(body, start, end, axis)
        minimum = math.inf
        for i in range(1001):
            center, axes = body.frame(start.between(end, i / 1000))
            minimum = min(minimum, sum(a * b for a, b in zip(center, axis)) - sum(body.half[j] * abs(sum(a * b for a, b in zip(axes[j], axis))) for j in range(3)))
        self.assertLessEqual(exact, minimum + 1e-9)
        self.assertLess(minimum - exact, 1e-5)

    def test_grazing_rotational_skin_contact_has_an_analytic_interior_time(self):
        peak = math.atan(.5)
        wall_z = math.sqrt(5) + SKIN - .00002
        wall_z = struct.unpack('<f', struct.pack('<f', wall_z))[0]
        walls = walls_at(z=wall_z)
        # Tiny closing velocities around a rotational extremum defeated the old
        # speed-bound iteration. The two endpoint projections both stay clear.
        start, end = Pose((0, 1, 0), peak - .01), Pose((0, 1, 0), peak + .01)
        hits = walls.sweep(self.body, start, end)
        expected = (.01 - math.acos((wall_z - SKIN) / math.sqrt(5))) / .02
        self.assertTrue(hits)
        self.assertAlmostEqual(hits[0]['fraction'], expected, places=7)

    def test_transform_winding_degenerate_and_input_limits(self):
        walls = walls_at()
        matrix = list(IDENTITY)
        matrix[12] = 20
        walls.add_mesh(wall_mesh(((0, 0, 0), (0, 0, 0), (0, 1, 0)), (0, 1, 2)), matrix, 'degenerate.kn5')
        walls.build()
        self.assertEqual(walls.degenerate, 1)
        triangle = walls.triangle(0)
        pose = Pose((0, 1, 3), 0)
        self.assertEqual(separation(self.body, pose, triangle)[0], separation(self.body, pose, tuple(reversed(triangle)))[0])
        with patch('openasc.walls.MAX_TRIANGLES', 0):
            with self.assertRaises(ContentError):
                walls.add_mesh(wall_mesh(((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0))), IDENTITY, 'budget.kn5')


class WallResponseTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.car = install_car(self.root)
        self.vehicle = load_vehicle(self.root, 'fixture')
        self.body = load_body(self.root, self.vehicle)
        pose = list(IDENTITY)
        pose[13] = 2
        self.spawn = spawn_transform('AC_PIT_0', 'original.kn5', pose)

    def model(self, walls=None):
        return Dynamics(self.vehicle, plane(), self.spawn, walls or walls_at(), self.body)

    def test_transformed_collider_dimensions_and_missing_or_degenerate_collider(self):
        self.assertAlmostEqual(self.body.half[0], .7, places=6)
        self.assertGreater(self.body.half[2], 1.6)
        self.assertGreater(self.body.half[1], .49)
        (self.car / 'collider.kn5').unlink()
        with self.assertRaises(OSError):
            load_body(self.root, self.vehicle)
        (self.car / 'collider.kn5').write_bytes(file_bytes(mesh()))
        with self.assertRaises(ContentError) as error:
            load_body(self.root, self.vehicle)
        self.assertEqual(error.exception.diagnostic.code, 'body_shape')

    def test_head_on_contact_stops_body_and_reverse_can_pull_away(self):
        model = self.model()
        model.set_controls(Controls(throttle=1))
        model.resume()
        for _ in range(600):
            model.step()
            self.assertFalse(model.walls.overlaps(model.body, Pose((model.x, model.y, model.z), model.yaw)))
        self.assertFalse(model.paused, model.reason)
        self.assertGreater(model.wall_contacts, 0)
        self.assertLess(abs(model.longitudinal_speed()), .1)
        self.assertLess(model.z + self.body.center[2] + self.body.half[2], 5)
        before = model.z
        model.set_controls(Controls(throttle=.5, gear=-1))
        for _ in range(120):
            model.step()
        self.assertLess(model.z, before - .5)
        self.assertLess(model.longitudinal_speed(), -1)

    def test_fast_dynamic_contact_and_glancing_motion_do_not_cross(self):
        model = self.model()
        model.vz = 80
        model.resume()
        for _ in range(30):
            model.step()
        self.assertFalse(model.paused, model.reason)
        self.assertGreater(model.wall_contacts, 0)
        self.assertLess(model.z + self.body.center[2] + self.body.half[2], 5)
        model = self.model()
        model.vx, model.vz = 5, 8
        model.resume()
        for _ in range(180):
            model.step()
            self.assertFalse(model.walls.overlaps(model.body, Pose((model.x, model.y, model.z), model.yaw)))
        self.assertFalse(model.paused, model.reason)
        self.assertGreater(model.wall_contacts, 0)
        self.assertGreater(model.x, 1)

    def test_inelastic_impulses_never_add_kinetic_energy(self):
        rng = random.Random(412)
        contacts = [{'normal': (0., 0., -1.), 'point': (x, 1., 2.)} for x in (-1., 1.)]
        for _ in range(200):
            velocity = (rng.uniform(-20, 20), rng.uniform(-10, 30))
            rate = rng.uniform(-5, 5)
            before = .5 * 900 * sum(v * v for v in velocity) + .5 * 1200 * rate * rate
            final, spin, impulse = resolve_contacts(contacts, Pose((0., 1., 0.), 0.), 900, 1200, velocity, rate)
            after = .5 * 900 * sum(v * v for v in final) + .5 * 1200 * spin * spin
            self.assertLessEqual(after, before + 1e-7)
            self.assertGreaterEqual(impulse, 0)

    def test_overlapping_spawn_and_vertical_response_are_diagnosed(self):
        with self.assertRaises(ContentError) as error:
            self.model(walls_at(z=0))
        self.assertEqual(error.exception.diagnostic.code, 'drive_spawn_wall_overlap')
        with self.assertRaises(ContentError) as error:
            resolve_contacts([{'normal': (0, -1, 0), 'point': (0, 2, 0)}], Pose((0, 1, 0), 0), 900, 1200, (1, 0), 0)
        self.assertEqual(error.exception.diagnostic.code, 'wall_vertical_contact')


if __name__ == '__main__':
    unittest.main()
