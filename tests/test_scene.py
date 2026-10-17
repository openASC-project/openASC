import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from contextlib import redirect_stdout

from openasc.__main__ import main
from openasc.config import ContentError
from openasc.scene import confined, inspect_scene


class SceneTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.track = self.root / 'content/tracks/test_track'
        self.track.mkdir(parents=True)

    def model(self, name='test.kn5', version=6):
        # Original header-only fixture. It intentionally contains no geometry.
        data = b'sc6969' + struct.pack('<I', version)
        if version == 6:
            data += struct.pack('<I', 0)
        (self.track / name).write_bytes(data)

    def config(self, text, layout=None):
        name = 'models.ini' if layout is None else f'models_{layout}.ini'
        (self.track / name).write_text(text, encoding='utf-8')

    def declaration(self, file='test.kn5'):
        return f'[MODEL_4]\nFILE={file}\nPOSITION=1,2,3\nROTATION=0,90,0\n'

    def test_manifest_is_deterministic_and_distinguishes_header_inspection(self):
        self.model()
        self.config(self.declaration() + '\n[DYNAMIC_OBJECT_0]\nFILE=unavailable.kn5\n')
        report = inspect_scene(self.root, 'test_track')
        self.assertEqual(report, inspect_scene(self.root, 'test_track'))
        self.assertEqual(report['scope'], 'configuration_and_header_only')
        self.assertFalse(report['geometry_loaded'])
        self.assertFalse(report['has_errors'])
        self.assertEqual(report['models'][0]['position'], [1, 2, 3])
        self.assertEqual(report['models'][0]['rotation'], [0, 90, 0])
        self.assertEqual(report['models'][0]['kn5']['scope'], 'header_only')
        self.assertEqual(report['diagnostics'][0]['code'], 'unsupported_section')
        self.assertEqual(report['diagnostics'][0]['severity'], 'warning')
        self.assertNotIn(str(self.root), json.dumps(report))

    def test_explicit_layout_and_no_automatic_choice(self):
        self.model(version=5)
        self.config(self.declaration(), 'short')
        with self.assertRaises(ContentError) as caught:
            inspect_scene(self.root, 'test_track')
        self.assertIn('--layout', caught.exception.diagnostic.message)
        report = inspect_scene(self.root, 'test_track', 'short')
        self.assertEqual(report['config'], 'models_short.ini')
        self.assertEqual(report['models'][0]['kn5']['version'], 5)

    def test_paths_cannot_escape_the_track(self):
        for path in ('../outside.kn5', '..\\outside.kn5', 'C:\\outside.kn5', 'C:outside.kn5', '/outside.kn5', '\\outside.kn5'):
            with self.subTest(path=path):
                with self.assertRaises(ContentError):
                    confined(self.track, path, 'models.ini', 2)
        for track in ('../test_track', 'a/b', 'a\\b', '..'):
            with self.assertRaises(ContentError):
                inspect_scene(self.root, track)

    def test_resource_and_transform_errors_have_exact_lines(self):
        self.model()
        cases = [
            (self.declaration('missing.kn5'), 'missing_resource', 2),
            (self.declaration().replace('POSITION=1,2,3', 'POSITION=1,2'), 'invalid_vector', 3),
            (self.declaration().replace('ROTATION=0,90,0', 'ROTATION=0,nan,0'), 'nonfinite_number', 4),
            (self.declaration().replace('POSITION=1,2,3\n', ''), 'missing_transform', 1),
            (self.declaration('../outside.kn5'), 'resource_escape', 2),
        ]
        for config, code, line in cases:
            with self.subTest(code=code):
                self.config(config)
                report = inspect_scene(self.root, 'test_track')
                matching = [d for d in report['diagnostics'] if d['code'] == code]
                self.assertEqual(matching[0]['line'], line)
                self.assertTrue(report['has_errors'])

    def test_resolved_link_cannot_escape_the_content_root(self):
        outside = self.root / 'outside.kn5'
        outside.write_bytes(b'sc6969' + struct.pack('<I', 5))
        link = self.track / 'linked.kn5'
        try:
            link.symlink_to(outside)
        except (OSError, NotImplementedError):
            self.skipTest('Creating symbolic links is unavailable on this host.')
        self.config(self.declaration('linked.kn5'))
        report = inspect_scene(self.root, 'test_track')
        self.assertIn('resource_escape', [d['code'] for d in report['diagnostics']])
        self.assertEqual(report['models'], [])

    def test_unknown_fields_are_diagnosed(self):
        self.model()
        self.config(self.declaration() + 'CUSTOM_FIELD=7\n')
        report = inspect_scene(self.root, 'test_track')
        self.assertEqual(report['diagnostics'][0]['code'], 'unsupported_key')
        self.assertEqual(report['diagnostics'][0]['line'], 5)

    def test_bad_signature_version_and_truncated_header(self):
        self.config(self.declaration())
        for data, expected in [(b'not-a-kn5-file', 'kn5_header'),
                               (b'sc6969' + struct.pack('<I', 99), 'kn5_version'),
                               (b'sc6969' + struct.pack('<I', 6), 'kn5_header')]:
            with self.subTest(expected=expected):
                (self.track / 'test.kn5').write_bytes(data)
                report = inspect_scene(self.root, 'test_track')
                self.assertIn(expected, [d['code'] for d in report['diagnostics']])
                self.assertEqual(report['models'], [])

    def test_invalid_scene_exits_nonzero_but_reports_all_errors(self):
        self.config(self.declaration('missing.kn5') + '[MODEL_8]\nFILE=also_missing.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n')
        output = io.StringIO()
        with redirect_stdout(output):
            code = main(['scene', str(self.root), '--track', 'test_track'])
        report = json.loads(output.getvalue())
        self.assertEqual(code, 2)
        self.assertEqual(sum(d['code'] == 'missing_resource' for d in report['diagnostics']), 2)
