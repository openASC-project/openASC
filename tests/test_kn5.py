from pathlib import Path
import struct
import tempfile
import unittest

from kn5_fixtures import base, file_bytes, mesh, string
from openasc.kn5 import IDENTITY, Kn5Error, load_kn5, multiply, transform


class Kn5Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'original.kn5'

    def load(self, data):
        self.path.write_bytes(data)
        return load_kn5(self.path, 'original.kn5')

    def test_versions_geometry_and_material_definitions(self):
        for version in (5, 6):
            with self.subTest(version=version):
                scene = self.load(file_bytes(version=version))
                self.assertEqual(scene.version, version)
                self.assertEqual(scene.summary()['triangles'], 1)
                self.assertEqual(scene.node_count, 2)
                self.assertEqual(scene.materials[0]['textures'][0]['texture'], 'fixture.dds')
                self.assertAlmostEqual(scene.materials[0]['properties']['ksDiffuse'][0], .7, places=6)
                self.assertEqual(scene.meshes[0].bounds, ([0., 0., 0.], [1., 0., 1.]))
                self.assertEqual(len(scene.meshes[0].vertices), 132)
                self.assertIn('textures_not_rendered', [d['code'] for d in scene.diagnostics])

    def test_hierarchy_composition_and_visibility_inheritance(self):
        translated = list(IDENTITY)
        translated[12] = 10
        rotated = (0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1.)
        scene = self.load(file_bytes(base([base([mesh()], rotated, False)], translated)))
        result = scene.meshes[0]
        self.assertEqual(result.bounds, ([10., 0., -1.], [11., 0., 0.]))
        self.assertFalse(result.active)
        self.assertEqual(transform(multiply(translated, rotated), (0., 0., 1.)), (11., 0., 0.))

    def test_skinned_mesh_is_diagnosed_and_following_mesh_stays_aligned(self):
        scene = self.load(file_bytes(base([mesh(skinned=True), mesh(name='after_skin')])))
        self.assertEqual([m.name for m in scene.meshes], ['after_skin'])
        self.assertIn('skinned_mesh_skipped', [d['code'] for d in scene.diagnostics])

    def test_invalid_indices_materials_and_numbers(self):
        cases = [(mesh(indices=(0, 2, 3)), 'kn5_indices'),
                 (mesh(indices=(0, 1)), 'kn5_indices'),
                 (mesh(material=2), 'kn5_material'),
                 (mesh(points=((float('nan'), 0, 0),)), 'kn5_nonfinite')]
        for node, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(Kn5Error) as error:
                    self.load(file_bytes(base([node])))
                self.assertEqual(error.exception.diagnostic.code, code)
                self.assertIsInstance(error.exception.diagnostic.byte_offset, int)

    def test_truncations_never_return_a_partial_scene(self):
        data = file_bytes()
        for end in (0, 5, 9, 13, 30, len(data) - 1, len(data) - 20):
            with self.subTest(end=end):
                with self.assertRaises(Kn5Error) as error:
                    self.load(data[:end])
                self.assertEqual(error.exception.diagnostic.source, 'original.kn5')
                self.assertIn('byte_offset', error.exception.diagnostic.as_dict())

    def test_unknown_nodes_trailing_data_and_allocation_limits(self):
        unknown = struct.pack('<I', 99) + string('unknown') + struct.pack('<IB', 0, 1)
        for data, code in [(file_bytes(unknown), 'kn5_node_type'),
                           (file_bytes() + b'extra', 'kn5_trailing_data'),
                           (b'sc6969' + struct.pack('<III', 6, 0, 0xffffffff), 'kn5_limit')]:
            with self.subTest(code=code):
                with self.assertRaises(Kn5Error) as error:
                    self.load(data)
                self.assertEqual(error.exception.diagnostic.code, code)

    def test_nonaffine_transform_and_excessive_depth(self):
        invalid = list(IDENTITY)
        invalid[3] = 1
        node = mesh()
        for _ in range(130):
            node = base([node])
        for data, code in [(file_bytes(base([mesh()], invalid)), 'kn5_transform'),
                           (file_bytes(node), 'kn5_limit')]:
            with self.subTest(code=code):
                with self.assertRaises(Kn5Error) as error:
                    self.load(data)
                self.assertEqual(error.exception.diagnostic.code, code)
