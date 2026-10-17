import contextlib
import io
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from openasc.acd import Archive, folder_key
from openasc.config import ContentError
from openasc.__main__ import main
from openasc.vehicle import CarData, load_vehicle
from vehicle_fixtures import FILES, archive_bytes, install_car


class ArchiveTests(unittest.TestCase):
    def test_independent_short_name_checksums_and_fixed_encoded_bytes(self):
        self.assertEqual(folder_key('a'), b'97-0-0-131-66-101-171-171')
        self.assertEqual(folder_key('AB'), b'195-158-0-33-66-101-171-99')
        self.assertEqual(folder_key('abcdef'), b'85-104-132-143-246-161-74-103')
        self.assertEqual(folder_key('abcdefghij'), b'247-134-32-237-238-213-74-107')
        data = struct.pack('<i', 7) + b'car.ini' + struct.pack('<i', 2) + bytes((140, 0, 0, 0, 145, 0, 0, 0))
        self.assertEqual(Archive(data, 'ab').read('CAR.INI'), b'[X')
        for identifier in ('', '../a', 'car name', 'машина', 'a' * 129):
            with self.assertRaises(ContentError):
                folder_key(identifier)

    def test_record_index_optional_header_case_and_per_record_key_reset(self):
        files = [('first.ini', bytes(range(256))), ('second.lut', b'0|1\n1|2'), ('empty.txt', b'')]
        for header in (None, 42):
            archive = Archive(archive_bytes(files, header=header), 'fixture', 'original.acd')
            self.assertEqual(archive.header, header)
            for name, expected in files:
                self.assertEqual(archive.read(name.upper()), expected)
            with self.assertRaises(ContentError):
                archive.read('missing.ini')
            with self.assertRaises(ContentError):
                archive.read('../first.ini')

    def test_truncated_fields_payload_and_invalid_names_are_diagnosed(self):
        data = archive_bytes([('one.ini', b'original')])
        for end in range(len(data)):
            with self.subTest(end=end):
                with self.assertRaises(ContentError) as error:
                    Archive(data[:end], 'fixture', 'truncated.acd')
                self.assertIn('byte_offset', error.exception.diagnostic.as_dict())
                self.assertEqual(error.exception.diagnostic.source, 'truncated.acd')
        for name in ('../bad.ini', '/bad.ini', 'folder/file.ini', 'C:bad.ini'):
            with self.assertRaises(ContentError) as error:
                Archive(archive_bytes([(name, b'')]), 'fixture')
            self.assertEqual(error.exception.diagnostic.code, 'acd_name')
        with self.assertRaises(ContentError) as error:
            Archive(archive_bytes([('same.ini', b'a'), ('SAME.INI', b'b')]), 'fixture')
        self.assertEqual(error.exception.diagnostic.code, 'acd_duplicate')

    def test_negative_lengths_declared_budgets_and_trailing_bytes(self):
        for value in (-1, 256, 0):
            with self.assertRaises(ContentError):
                Archive(struct.pack('<i', value), 'fixture')
        prefix = struct.pack('<i', 5) + b'a.ini'
        for length in (-1, 8 * 1024 ** 2 + 1):
            with self.assertRaises(ContentError) as error:
                Archive(prefix + struct.pack('<i', length), 'fixture')
            self.assertEqual(error.exception.diagnostic.code, 'acd_limit')
        data = archive_bytes([('a.ini', b'a')])
        for limit, value in [('MAX_ARCHIVE', 1), ('MAX_DECODED', 0), ('MAX_ENTRIES', 0)]:
            with patch('openasc.acd.' + limit, value):
                with self.assertRaises(ContentError) as error:
                    Archive(data, 'fixture')
                self.assertEqual(error.exception.diagnostic.code, 'acd_limit')
        with self.assertRaises(ContentError):
            Archive(data + b'junk', 'fixture')


class VehicleTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.car = install_car(self.root)

    def test_packed_and_unpacked_fields_wheel_positions_and_no_extraction(self):
        before = set(self.root.rglob('*'))
        vehicle = load_vehicle(self.root, 'fixture')
        self.assertEqual((vehicle.mass_kg, vehicle.wheelbase_m), (900, 3))
        self.assertEqual(vehicle.drivetrain['forward_ratios'], (3, 2, 1.2))
        self.assertAlmostEqual(vehicle.wheel_offsets()['front_left'][0], .7)
        self.assertAlmostEqual(vehicle.wheel_offsets()['front_left'][2], 1.8)
        self.assertAlmostEqual(vehicle.wheel_offsets()['rear_right'][2], -1.2)
        self.assertEqual(vehicle.inertia_dimensions_m, (1.5, 1.2, 3.5))
        self.assertEqual(vehicle.field_sources['car.ini']['BASIC.TOTALMASS']['line'], 4)
        self.assertFalse(vehicle.summary()['simulation_ready'])
        self.assertNotIn(str(self.root), json.dumps(vehicle.summary()))
        self.assertEqual(set(self.root.rglob('*')), before)
        packed = install_car(self.root, 'packed_fixture', packed=True)
        packed_before = set(packed.rglob('*'))
        other = load_vehicle(self.root, 'packed_fixture')
        self.assertEqual(other.mass_kg, vehicle.mass_kg)
        self.assertEqual(other.engine['torque_lut']['points'], vehicle.engine['torque_lut']['points'])
        self.assertEqual(other.data_source, 'packed')
        self.assertEqual(set(packed.rglob('*')), packed_before)

    def test_compound_choice_and_unsupported_version(self):
        self.assertEqual(load_vehicle(self.root, 'fixture', compound=1).axles['front']['tyre_radius_m'], .32)
        with self.assertRaises(ContentError):
            load_vehicle(self.root, 'fixture', compound=2)
        path = self.car / 'data/tyres.ini'
        path.write_text(path.read_text().replace('VERSION=10', 'VERSION=999'))
        with self.assertRaises(ContentError) as error:
            load_vehicle(self.root, 'fixture')
        self.assertEqual(error.exception.diagnostic.code, 'vehicle_version')

    def test_auto_selects_unpacked_as_a_whole_with_no_archive_fallback(self):
        (self.car / 'data.acd').write_bytes(archive_bytes(FILES.items()))
        data = CarData(self.root, 'fixture')
        self.assertEqual(data.mode, 'unpacked')
        self.assertEqual(data.diagnostics[0]['code'], 'car_data_precedence')
        (self.car / 'data/engine.ini').unlink()
        with self.assertRaises(ContentError):
            load_vehicle(self.root, 'fixture')
        self.assertEqual(load_vehicle(self.root, 'fixture', mode='packed').mass_kg, 900)

    def test_invalid_ranges_vectors_gears_and_lut_domain(self):
        cases = [('car.ini', 'TOTALMASS=900', 'TOTALMASS=nan', 'nonfinite_number'),
                 ('car.ini', 'INERTIA=1.5,1.2,3.5', 'INERTIA=1,2', 'vehicle_vector'),
                 ('suspensions.ini', 'CG_LOCATION=.4', 'CG_LOCATION=1', 'vehicle_range'),
                 ('drivetrain.ini', 'COUNT=3', 'COUNT=3.5', 'vehicle_range'),
                 ('drivetrain.ini', 'GEAR_R=-3', 'GEAR_R=3', 'vehicle_range'),
                 ('brakes.ini', 'FRONT_SHARE=.6', 'FRONT_SHARE=1.1', 'vehicle_range'),
                 ('power.lut', '6500|100', '5000|100', 'vehicle_power_domain')]
        for file, old, new, code in cases:
            with self.subTest(file=file, code=code):
                path = self.car / 'data' / file
                original = path.read_text()
                path.write_text(original.replace(old, new))
                try:
                    with self.assertRaises(ContentError) as error:
                        load_vehicle(self.root, 'fixture')
                    self.assertEqual(error.exception.diagnostic.code, code)
                finally:
                    path.write_text(original)

    def test_references_cannot_escape_and_text_encoding_is_explicit(self):
        path = self.car / 'data/engine.ini'
        path.write_text(path.read_text().replace('power.lut', '../power.lut'))
        with self.assertRaises(ContentError) as error:
            load_vehicle(self.root, 'fixture')
        self.assertEqual(error.exception.diagnostic.code, 'car_data_name')
        path.write_text(FILES['engine.ini'])
        lod = self.car / 'data/lods.ini'
        lod.write_text('[LOD_0]\nFILE=../../outside.kn5')
        with self.assertRaises(ContentError) as error:
            load_vehicle(self.root, 'fixture')
        self.assertEqual(error.exception.diagnostic.code, 'resource_escape')
        (self.car / 'data/label.txt').write_bytes(b'caf\xe9')
        with self.assertRaises(ContentError):
            CarData(self.root, 'fixture').text('label.txt')
        self.assertEqual(CarData(self.root, 'fixture', encoding='cp1252').text('label.txt'), 'café')
        with patch('openasc.vehicle.MAX_ENTRY', 2):
            with self.assertRaises(ContentError):
                CarData(self.root, 'fixture').text('car.ini')

    def test_cli_reports_configuration_and_rejects_missing_input(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            code = main(['car', str(self.root), '--car', 'fixture'])
        self.assertEqual(code, 0)
        self.assertEqual(json.loads(output.getvalue())['scope'], 'vehicle_configuration_subset')
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(['car', str(self.root), '--car', 'missing']), 2)
