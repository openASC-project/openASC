"""Original geometry and configuration fixtures for independent ground queries."""
import contextlib
import io
import json
import math
from pathlib import Path
import random
import struct
import tempfile
import unittest
from unittest.mock import patch

from kn5_fixtures import base, file_bytes, mesh
from openasc.__main__ import main
from openasc.config import ContentError
from openasc.ground import Ground, load_ground
from openasc.kn5 import IDENTITY, Mesh
from openasc.surfaces import mesh_surface, parse_surfaces, load_surfaces


def definition(key='ROAD', friction='.83', number=0):
    return f'[SURFACE_{number}]\nKEY={key}\nFRICTION={friction}\nDAMPING=.02\nIS_VALID_TRACK=1\nIS_PITLANE=0\n'


SURFACE = parse_surfaces(definition())[0]['ROAD']


def triangle(points, name='01ROAD', world=IDENTITY):
    vertices = b''.join(struct.pack('<11f', *p, 0, 1, 0, 0, 0, 1, 0, 0) for p in points)
    return Mesh(name, 0, world, vertices, struct.pack('<3H', 0, 1, 2), True, False, False, (0, 0), None)


class SurfaceTests(unittest.TestCase):
    def test_typed_parameters_and_source_locations(self):
        text = definition() + 'SIN_HEIGHT=.01\nWAV=custom.wav\nFUTURE_FIELD=uninterpreted\n'
        surfaces, diagnostics = parse_surfaces(text, 'original.ini')
        surface = surfaces['ROAD']
        self.assertEqual((surface.friction, surface.damping, surface.valid_track, surface.pitlane), (.83, .02, True, False))
        self.assertEqual(surface.parameters['SIN_HEIGHT'], .01)
        self.assertEqual(surface.parameters['WAV'], 'custom.wav')
        self.assertEqual((diagnostics[0]['source'], diagnostics[0]['line']), ('original.ini', 9))

    def test_invalid_coefficients_flags_required_fields_and_duplicates(self):
        cases = [(definition(friction='NaN'), 'nonfinite_number'),
                 (definition(friction='-1'), 'surface_range'),
                 (definition().replace('IS_VALID_TRACK=1', 'IS_VALID_TRACK=2'), 'surface_flag'),
                 (definition().replace('DAMPING=.02\n', ''), 'surface_field'),
                 (definition() + definition(number=1), 'surface_duplicate'),
                 (definition(key='1ROAD'), 'surface_key')]
        for text, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(ContentError) as error:
                    parse_surfaces(text, 'surface.ini')
                self.assertEqual(error.exception.diagnostic.code, code)
                self.assertGreater(error.exception.diagnostic.line, 0)

    def test_positive_numeric_prefix_and_ambiguous_keys(self):
        surfaces = parse_surfaces(definition())[0]
        for name in ('ROAD', '0ROAD', '000ROAD', 'VISUAL_ROAD', 'AC_PIT_0'):
            self.assertIsNone(mesh_surface(name, surfaces))
        self.assertEqual(mesh_surface('001road_detail', surfaces).key, 'ROAD')
        with self.assertRaises(ContentError) as error:
            mesh_surface('1GRAVEL', surfaces)
        self.assertEqual(error.exception.diagnostic.code, 'surface_unmatched')
        surfaces.update(parse_surfaces(definition(key='ROAD_EXTRA'))[0])
        with self.assertRaises(ContentError) as error:
            mesh_surface('1ROAD_EXTRA_1', surfaces)
        self.assertEqual(error.exception.diagnostic.code, 'surface_ambiguous')


class GroundMathTests(unittest.TestCase):
    def ground(self, triangles):
        ground = Ground()
        for i, points in enumerate(triangles):
            ground.add_mesh(triangle(points, f'1ROAD_{i}'), IDENTITY, SURFACE, 'original.kn5')
        ground.build()
        return ground

    def test_ramp_height_normal_winding_and_edge_contact(self):
        points = ((0, 0, 0), (2, 2, 0), (0, 0, 2))
        for shape in (points, tuple(reversed(points))):
            ground = self.ground([shape])
            hit = ground.probe((.5, 3, .5))
            self.assertAlmostEqual(hit['position'][1], .5)
            self.assertAlmostEqual(hit['distance'], 2.5)
            for actual, expected in zip(hit['normal'], (-math.sqrt(.5), math.sqrt(.5), 0)):
                self.assertAlmostEqual(actual, expected)
            self.assertIsNotNone(ground.probe((1, 1, 1), 0))
            self.assertIsNone(ground.probe((1.1, 4, 1.1)))
            self.assertIsNone(ground.probe((.5, .4, .5)))
            self.assertIsNone(ground.probe((.5, 3, .5), 2.4))

    def test_bridge_selects_nearest_surface_below_origin(self):
        ground = self.ground([((0, y, 0), (3, y, 0), (0, y, 3)) for y in (0, 4)])
        self.assertEqual(ground.probe((1, 5, 1))['position'][1], 4)
        self.assertEqual(ground.probe((1, 3, 1))['position'][1], 0)
        self.assertEqual(ground.probe((1, 4, 1), 0)['distance'], 0)
        self.assertIsNone(ground.probe((1, -1, 1)))

    def test_world_transform_and_degenerate_vertical_faces(self):
        ground = Ground()
        world = (0., 0., -1., 0., 0., 2., 0., 0., 1., 0., 0., 0., 10., 3., 20., 1.)
        ground.add_mesh(triangle(((0, 0, 0), (2, 2, 0), (0, 0, 2))), world, SURFACE, 'transformed.kn5')
        ground.add_mesh(triangle(((0, 0, 0), (0, 2, 0), (0, 0, 2))), IDENTITY, SURFACE, 'vertical.kn5')
        ground.add_mesh(triangle(((0, 0, 0), (0, 0, 0), (1, 0, 0))), IDENTITY, SURFACE, 'degenerate.kn5')
        ground.build()
        self.assertAlmostEqual(ground.probe((10.5, 20, 19.5))['position'][1], 4)
        self.assertEqual(ground.skipped_degenerate, 2)

    def test_bvh_matches_analytic_grid_and_is_deterministic_on_shared_edges(self):
        shapes = []
        for x in range(12):
            for z in range(12):
                shapes.append(((x, x * .1 + z * .2, z), (x + 1, (x + 1) * .1 + z * .2, z), (x, x * .1 + (z + 1) * .2, z + 1)))
                shapes.append(((x + 1, (x + 1) * .1 + z * .2, z), (x + 1, (x + 1) * .1 + (z + 1) * .2, z + 1), (x, x * .1 + (z + 1) * .2, z + 1)))
        ground = self.ground(shapes)
        self.assertIsNotNone(ground.root.left)
        rng = random.Random(482)
        for _ in range(200):
            x, z = rng.uniform(.01, 11.99), rng.uniform(.01, 11.99)
            hit = ground.probe((x, 10, z))
            self.assertIsNotNone(hit)
            self.assertAlmostEqual(hit['position'][1], .1 * x + .2 * z, places=6)
        self.assertEqual(ground.probe((.5, 10, .5))['triangle'], 0)
        self.assertIsNone(ground.probe((-1, 10, 2)))

    def test_empty_query_limits_and_geometry_budget(self):
        ground = self.ground([])
        self.assertIsNone(ground.probe((0, 0, 0)))
        for origin, distance in [((math.nan, 0, 0), 1), ((0, 0, 0), math.inf), ((0, 0, 0), -1), ((1e8, 0, 0), 1), ((0, 0), 1)]:
            with self.subTest(origin=origin, distance=distance):
                with self.assertRaises(ContentError):
                    ground.probe(origin, distance)
        with patch('openasc.ground.MAX_TRIANGLES', 0):
            with self.assertRaises(ContentError) as error:
                ground.add_mesh(triangle(((0, 0, 0), (1, 0, 0), (0, 0, 1))), IDENTITY, SURFACE, 'budget.kn5')
            self.assertEqual(error.exception.diagnostic.code, 'ground_limit')
        with self.assertRaises(ContentError) as error:
            ground.add_mesh(triangle(((1e8, 0, 0), (1e8, 0, 1), (1e8, 1, 0))), IDENTITY, SURFACE, 'range.kn5')
        self.assertEqual(error.exception.diagnostic.code, 'ground_range')


class GroundIntegrationTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.track = self.root / 'content/tracks/fixture'
        (self.track / 'data').mkdir(parents=True)
        (self.root / 'system/data').mkdir(parents=True)
        (self.root / 'system/data/surfaces.ini').write_text(definition())
        (self.track / 'data/surfaces.ini').write_text(definition(friction='.71'))
        (self.track / 'models.ini').write_text('[MODEL_0]\nFILE=fixture.kn5\nPOSITION=10,3,20\nROTATION=0,0,0\n')
        (self.track / 'fixture.kn5').write_bytes(file_bytes(base([
            mesh(name='01ROAD', renderable=False), mesh(name='visual'),
            mesh(name='01WALL'), mesh(name='01UNKNOWN')])))

    def test_hidden_physics_meshes_and_surface_override_without_asset_export(self):
        before = set(self.root.rglob('*'))
        ground = load_ground(self.root, 'fixture')
        self.assertEqual(len(ground.mesh_ids), 1)
        self.assertEqual(ground.probe((10.25, 4, 20.25))['surface']['friction'], .71)
        self.assertEqual(ground.probe((10.25, 4, 20.25))['position'], [10.25, 3., 20.25])
        codes = {d['code'] for d in ground.diagnostics}
        self.assertTrue({'ground_wall_omitted', 'surface_unmatched'} <= codes)
        self.assertNotIn(str(self.root), json.dumps(ground.summary()))
        self.assertEqual(set(self.root.rglob('*')), before)

    def test_layout_uses_only_selected_surface_file_and_shared_system_defaults(self):
        (self.track / 'alternate/data').mkdir(parents=True)
        (self.track / 'alternate/data/surfaces.ini').write_text(definition(friction='.42'))
        surfaces, _ = load_surfaces(self.root, 'fixture', 'alternate')
        self.assertEqual(surfaces['ROAD'].friction, .42)
        surfaces, warnings = load_surfaces(self.root, 'fixture', 'absent')
        self.assertEqual(surfaces['ROAD'].friction, .83)
        self.assertEqual(warnings[0]['code'], 'surface_file_missing')
        with self.assertRaises(ContentError):
            load_surfaces(self.root, '../outside')

    def test_invalid_override_is_not_silently_replaced_with_default(self):
        (self.track / 'data/surfaces.ini').write_text(definition(friction='NaN'))
        with self.assertRaises(ContentError):
            load_ground(self.root, 'fixture')

    def test_inactive_physical_mesh_and_unvalidated_model_rotation_are_omitted(self):
        (self.track / 'fixture.kn5').write_bytes(file_bytes(base([mesh(name='01ROAD')], active=False)))
        ground = load_ground(self.root, 'fixture')
        self.assertFalse(ground.mesh_ids)
        self.assertTrue(ground.summary()['has_errors'])
        config = self.track / 'models.ini'
        config.write_text(config.read_text().replace('ROTATION=0,0,0', 'ROTATION=0,90,0'))
        ground = load_ground(self.root, 'fixture')
        self.assertIn('model_rotation_unsupported', [d['code'] for d in ground.diagnostics])

    def test_cli_rejects_nonfinite_probe_as_json_error(self):
        output = io.StringIO()
        with contextlib.redirect_stderr(output):
            code = main(['ground', str(self.root), '--track', 'fixture', '--at', 'nan', '0', '0'])
        self.assertEqual(code, 2)
        self.assertEqual(json.loads(output.getvalue())['diagnostics'][0]['code'], 'nonfinite_number')

    def test_cli_hit_miss_and_no_ground_exit_status(self):
        def run(extra):
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = main(['ground', str(self.root), '--track', 'fixture', *extra])
            return code, json.loads(output.getvalue())
        code, result = run(['--at', '10.25', '4', '20.25'])
        self.assertEqual(code, 0)
        self.assertEqual(result['probe']['hit']['mesh'], '01ROAD')
        self.assertIsNone(run(['--at', '0', '4', '0'])[1]['probe']['hit'])
        (self.track / 'fixture.kn5').write_bytes(file_bytes(base([mesh(name='visual')])))
        code, result = run([])
        self.assertEqual(code, 2)
        self.assertTrue(result['has_errors'])
        self.assertIn('no_ground', [d['code'] for d in result['diagnostics']])


if __name__ == '__main__':
    unittest.main()
