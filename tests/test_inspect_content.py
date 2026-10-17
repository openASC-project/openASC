import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from tools.inspect_content import inventory, main


class InventoryTests(unittest.TestCase):
    def test_invalid_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                inventory(Path(directory))

    def test_empty_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for kind in ('cars', 'tracks'):
                (root / 'content' / kind).mkdir(parents=True)
            self.assertEqual(inventory(root)['counts'], {'cars': 0, 'tracks': 0})

    def test_presence_determinism_and_no_parsing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            car = root / 'content/cars/test_car'
            (car / 'ui').mkdir(parents=True)
            (car / 'ui/ui_car.json').write_text('deliberately not JSON')
            (car / 'data.acd').write_bytes(b'own fixture')
            (root / 'content/cars/another_car/data').mkdir(parents=True)
            track = root / 'content/tracks/test_track'
            track.mkdir(parents=True)
            (track / 'models_layout.ini').write_text('[MODEL_0]')
            (track / 'test.kn5').write_bytes(b'own fixture')
            before = (car / 'data.acd').read_bytes()
            result = inventory(root)
            self.assertEqual(result, inventory(root))
            self.assertEqual(result['counts'], {'cars': 2, 'tracks': 1})
            self.assertEqual([c['id'] for c in result['cars']], ['another_car', 'test_car'])
            self.assertTrue(result['cars'][1]['packed_data_present'])
            self.assertTrue(result['cars'][1]['ui_metadata_present'])
            self.assertFalse(result['cars'][1]['unpacked_data_present'])
            self.assertTrue(result['tracks'][0]['models_config_present'])
            self.assertTrue(result['tracks'][0]['kn5_present'])
            self.assertNotIn(str(root), str(result))
            self.assertEqual((car / 'data.acd').read_bytes(), before)

    def test_output_never_overwrites_existing_file_or_game_installation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'game'
            for kind in ('cars', 'tracks'):
                (root / 'content' / kind).mkdir(parents=True)
            existing = Path(directory) / 'existing.json'
            existing.write_text('keep this')
            for output in (existing, root / 'report.json'):
                with self.subTest(output=output.name):
                    with patch('sys.argv', ['inspect_content', str(root), '--output', str(output)]):
                        with self.assertRaises(SystemExit) as error:
                            main()
                        self.assertEqual(error.exception.code, 2)
            self.assertEqual(existing.read_text(), 'keep this')
            self.assertFalse((root / 'report.json').exists())


if __name__ == '__main__':
    unittest.main()
