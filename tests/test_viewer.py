import http.client
import json
from pathlib import Path
import tempfile
from threading import Thread
import unittest

from kn5_fixtures import base, file_bytes, mesh
from dds_fixtures import dds
from openasc.viewer import PreviewError, load_preview, make_server


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.track = self.root / 'content/tracks/fixture'
        self.track.mkdir(parents=True)
        self.config = self.track / 'models.ini'
        self.config.write_text('[MODEL_0]\nFILE=fixture.kn5\nPOSITION=10,0,0\nROTATION=0,0,0\n')
        (self.track / 'fixture.kn5').write_bytes(file_bytes(base([mesh(name='visual'), mesh(name='01ROAD', renderable=False)])))

    def test_transforms_hidden_road_focus_and_binary_offsets(self):
        preview = load_preview(self.root, 'fixture')
        metadata = preview.metadata
        self.assertTrue(metadata['geometry_loaded'])
        self.assertFalse(metadata['texture_preview_available'])
        self.assertEqual(metadata['bounds'], [[10., 0., 0.], [11., 0., 1.]])
        self.assertEqual(metadata['road_focus_bounds'], metadata['bounds'])
        self.assertEqual(len(metadata['meshes']), 1)
        item = metadata['meshes'][0]
        self.assertEqual(item['vertex_offset'] % 4, 0)
        self.assertEqual(item['world'][12], 10)
        self.assertEqual(preview.geometry[item['index_offset']:item['index_offset'] + 6], b'\0\0\2\0\1\0')
        self.assertNotIn(str(self.root), json.dumps(metadata))

    def test_no_geometry_preserves_underlying_errors(self):
        self.config.write_text('[MODEL_0]\nFILE=fixture.kn5\nPOSITION=0,0,0\nROTATION=0,90,0\n')
        with self.assertRaises(PreviewError) as caught:
            load_preview(self.root, 'fixture')
        self.assertIn('model_rotation_unsupported', [d['code'] for d in caught.exception.diagnostics])
        self.assertIn('no_preview_geometry', [d['code'] for d in caught.exception.diagnostics])

    def test_embedded_diffuse_texture_is_resolved_without_exporting(self):
        (self.track / 'fixture.kn5').write_bytes(file_bytes(texture=dds()))
        preview = load_preview(self.root, 'fixture')
        self.assertTrue(preview.metadata['texture_preview_available'])
        self.assertEqual(preview.metadata['materials'][0]['preview_texture'], 0)
        self.assertEqual(preview.metadata['textures'][0]['format'], 'BC1')
        self.assertEqual(preview.textures[0].data[:4], b'DDS ')
        self.assertFalse(list(self.track.glob('*.dds')))

    def test_unsupported_texture_remains_visible_as_diagnostic_colour(self):
        preview = load_preview(self.root, 'fixture')
        self.assertFalse(preview.metadata['texture_preview_available'])
        self.assertIn('texture_unsupported', [d['code'] for d in preview.metadata['diagnostics']])
        self.assertFalse(preview.metadata['has_errors'])

    def test_shared_texture_cache_and_rectangular_mip_budget(self):
        texture = dds(width=1, height=8, fourcc=b'\0' * 4, bits=8,
                      pf_flags=0x20000, masks=(255, 0, 0, 0), payload=bytes(8))
        (self.track / 'fixture.kn5').write_bytes(file_bytes(texture=texture))
        original = self.config.read_text()
        self.config.write_text(original + original.replace('MODEL_0', 'MODEL_1'))
        preview = load_preview(self.root, 'fixture')
        self.assertEqual(len(preview.textures), 1)
        self.assertEqual(preview.textures[0].info['decoded_budget'], (8 + 4 + 2 + 1) * 4)
        self.assertEqual([item['preview_texture'] for item in preview.metadata['materials']], [0, 0])

    def test_material_states_survive_texture_preparation(self):
        for blend, alpha, depth in [(0, False, 0), (1, False, 1), (2, False, 2), (0, True, 0)]:
            with self.subTest(blend=blend, alpha=alpha, depth=depth):
                (self.track / 'fixture.kn5').write_bytes(file_bytes(texture=dds(), blend=blend, alpha_tested=alpha, depth=depth))
                material = load_preview(self.root, 'fixture').metadata['materials'][0]
                self.assertEqual(material['preview_blend'], 1 if blend == 1 else 0)
                self.assertEqual(material['preview_alpha_test'], alpha or blend == 2)
                self.assertEqual(material['preview_depth'], depth)

    def test_multiple_models_have_aligned_binary_data_and_material_offsets(self):
        (self.track / 'fixture.kn5').write_bytes(file_bytes(base([mesh(name='a'), mesh(name='b')])))
        original = self.config.read_text()
        self.config.write_text(original + original.replace('MODEL_0', 'MODEL_1').replace('10,0,0', '20,0,0'))
        preview = load_preview(self.root, 'fixture')
        items = preview.metadata['meshes']
        self.assertEqual([item['material'] for item in items], [0, 0, 1, 1])
        self.assertEqual(preview.metadata['bounds'], [[10., 0., 0.], [21., 0., 1.]])
        for item in items:
            self.assertEqual(item['vertex_offset'] % 4, 0)
            self.assertEqual(item['index_offset'] % 2, 0)
            self.assertEqual(len(preview.geometry[item['vertex_offset']:item['vertex_offset'] + item['vertex_bytes']]), 132)

    def test_local_server_assets_metadata_and_request_boundaries(self):
        (self.track / 'fixture.kn5').write_bytes(file_bytes(texture=dds()))
        preview = load_preview(self.root, 'fixture')
        server = make_server(preview, 0)
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            def request(path, headers=None):
                client = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=5)
                try:
                    client.request('GET', path, headers=headers or {})
                    response = client.getresponse()
                    return response.status, dict(response.getheaders()), response.read()
                finally:
                    client.close()

            status, headers, body = request('/api/scene')
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(body)['triangles'], 1)
            self.assertEqual(headers['Cache-Control'], 'no-store')
            self.assertNotIn('Access-Control-Allow-Origin', headers)
            self.assertIn("frame-ancestors 'none'", headers['Content-Security-Policy'])
            self.assertEqual(request('/geometry.bin')[2], preview.geometry)
            self.assertEqual(request('/textures/0.bin')[2], preview.textures[0].data)
            self.assertEqual(request('/textures/1.bin')[0], 404)
            self.assertEqual(request('/textures/../fixture.kn5')[0], 404)
            self.assertEqual(request('/textures/0.bin', {'Origin': 'https://example.com'})[0], 403)
            self.assertIn(b'Local scene viewer', request('/')[2])
            self.assertEqual(request('/../README.md')[0], 404)
            self.assertEqual(request('/api/scene', {'Host': 'example.com'})[0], 403)
            self.assertEqual(request('/api/scene', {'Origin': 'https://example.com'})[0], 403)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=5)
