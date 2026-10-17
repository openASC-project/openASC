import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from kn5_fixtures import base, file_bytes, mesh
from vehicle_fixtures import install_car
from openasc.__main__ import main
from openasc.config import ContentError
from openasc.kn5 import IDENTITY, load_kn5
from openasc.placement import inspect_placement, inspect_spawns, spawn_transform


class PlacementTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        install_car(self.root)
        self.track = self.root / 'content/tracks/fixture'
        (self.track / 'data').mkdir(parents=True)
        (self.track / 'data/surfaces.ini').write_text('[SURFACE_0]\nKEY=ROAD\nFRICTION=.9\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n')
        (self.track / 'models.ini').write_text('[MODEL_0]\nFILE=original.kn5\nPOSITION=5,3,0\nROTATION=0,0,0\n')
        self.path = self.track / 'original.kn5'
        self.pose = (0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 10., 2., 0., 1.)
        self.write()

    def write(self, markers=None):
        if markers is None:
            markers = [base([mesh(name='AC_PIT_0', renderable=False)], matrix=self.pose, name='AC_PIT_0')]
        road = mesh(name='01ROAD', renderable=False, points=((-50, 0, -50), (50, 0, -50), (-50, 0, 50), (50, 0, 50)), indices=(0, 2, 1, 1, 2, 3))
        self.path.write_bytes(file_bytes(base([road, *markers])))

    def test_source_basis_parent_links_and_same_named_attached_cube(self):
        parsed = load_kn5(self.path)
        named = [node for node in parsed.node_transforms if node.name == 'AC_PIT_0']
        self.assertEqual(len(named), 2)
        self.assertEqual(parsed.node_transforms[named[1].parent].name, 'AC_PIT_0')
        result = inspect_spawns(self.root, 'fixture')
        self.assertFalse(result['has_errors'])
        self.assertEqual(len(result['spawns']), 1)
        selected = result['spawns'][0]
        self.assertEqual(selected['position'], (15, 5, 0))
        self.assertEqual(selected['forward'], (1, 0, 0))
        self.assertEqual(selected['left'], (0, 0, -1))

    def test_vehicle_footprint_support_preserves_rotated_forward_left_and_right(self):
        before = set(self.root.rglob('*'))
        result = inspect_placement(self.root, 'fixture', 'fixture')
        self.assertTrue(result['all_wheels_supported'])
        self.assertFalse(result['has_errors'])
        self.assertFalse(result['simulation_ready'])
        left = result['contacts']['front_left']
        self.assertAlmostEqual(left['origin'][0], 16.8)
        self.assertAlmostEqual(left['origin'][2], -.7)
        self.assertEqual(left['hit']['position'][1], 3)
        self.assertEqual(left['hit']['distance'], 2)
        self.assertEqual(result['contacts']['rear_right']['origin'], [13.8, 5, .75])
        self.assertNotIn(str(self.root), json.dumps(result))
        self.assertEqual(set(self.root.rglob('*')), before)

    def test_genuine_duplicate_is_not_repaired_by_matching_position(self):
        self.write([base([], matrix=self.pose, name='AC_PIT_0'), base([], matrix=self.pose, name='AC_PIT_0')])
        result = inspect_spawns(self.root, 'fixture')
        self.assertTrue(result['has_errors'])
        self.assertFalse(result['spawns'])
        self.assertIn('spawn_duplicate', [d['code'] for d in result['diagnostics']])
        with self.assertRaises(ContentError) as error:
            inspect_placement(self.root, 'fixture', 'fixture')
        self.assertIn('spawn_duplicate', [d['code'] for d in error.exception.diagnostics])

    def test_noncanonical_inactive_and_skewed_spawns_are_explicit(self):
        skewed = list(self.pose)
        skewed[0:3] = (0, .2, -1)
        self.write([base([], name='AC_PIT_00'), base([], name='AC_PIT_1', active=False),
                    base([], name='AC_PIT_2', matrix=skewed)])
        result = inspect_spawns(self.root, 'fixture')
        self.assertFalse(result['spawns'])
        codes = {d['code'] for d in result['diagnostics']}
        self.assertTrue({'spawn_name', 'spawn_basis', 'no_spawns'} <= codes)
        for matrix in ((-1., 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1),
                       (1., 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1),
                       (1., 0, 0, .1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)):
            with self.assertRaises(ContentError):
                spawn_transform('AC_PIT_0', 'original.kn5', matrix)

    def test_unsupported_model_rotation_and_missing_ground_are_not_success(self):
        config = self.track / 'models.ini'
        original = config.read_text()
        config.write_text(original.replace('ROTATION=0,0,0', 'ROTATION=0,90,0'))
        result = inspect_spawns(self.root, 'fixture')
        self.assertTrue(result['has_errors'])
        self.assertIn('model_rotation_unsupported', [d['code'] for d in result['diagnostics']])
        config.write_text(original)
        outside = list(self.pose)
        outside[12] = 100
        self.write([base([], name='AC_PIT_0', matrix=outside)])
        result = inspect_placement(self.root, 'fixture', 'fixture')
        self.assertFalse(result['all_wheels_supported'])
        self.assertTrue(result['has_errors'])
        self.assertEqual(sum(d['code'] == 'placement_no_support' for d in result['diagnostics']), 4)

    def test_spawn_and_placement_cli_exit_statuses(self):
        for command, extra in [('spawns', []), ('place', ['--car', 'fixture'])]:
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = main([command, str(self.root), '--track', 'fixture', *extra])
            self.assertEqual(code, 0)
            self.assertFalse(json.loads(output.getvalue())['has_errors'])
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(['place', str(self.root), '--car', 'fixture', '--track', 'fixture', '--spawn', 'AC_PIT_99']), 2)
