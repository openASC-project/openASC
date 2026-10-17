import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

from openasc.__main__ import main
from openasc.config import ContentError, parse_ini, parse_lut, read_ini


class IniTests(unittest.TestCase):
    def test_values_comments_case_and_source_locations(self):
        document = parse_ini('\ufeff; own fixture\n[model_7] ; comment\nfile="folder/a;b.kn5" ; tail\nposition = 1, 2, 3\nempty=\nurl=https://example.org/a#part\nname=driver\'s car\nvalue=a=b\n', 'fixture.ini')
        section = document.sections['MODEL_7']
        self.assertEqual(section.line, 2)
        self.assertEqual(section.lines['FILE'], 3)
        self.assertEqual(section.values['FILE'], 'folder/a;b.kn5')
        self.assertEqual(section.values['POSITION'], '1, 2, 3')
        self.assertEqual(section.values['EMPTY'], '')
        self.assertEqual(section.values['URL'], 'https://example.org/a#part')
        self.assertEqual(section.values['NAME'], "driver's car")
        self.assertEqual(section.values['VALUE'], 'a=b')

    def test_windows_path_is_not_escaped(self):
        document = parse_ini('[A]\nFILE="models\\new.kn5"')
        self.assertEqual(document.sections['A'].values['FILE'], 'models\\new.kn5')

    def test_ambiguous_or_invalid_input_is_rejected_with_location(self):
        cases = [
            ('x=1', 'missing_section', 1),
            ('[]', 'empty_section', 1),
            ('[A]junk', 'invalid_section', 1),
            ('[A]\n[a]', 'duplicate_section', 2),
            ('[A]\nx=1\nX=2', 'duplicate_key', 3),
            ('[A]\ninvalid', 'invalid_assignment', 2),
            ('[A]\n=1', 'invalid_key', 2),
            ('[A]\nbad key=1', 'invalid_key', 2),
            ('[A]\nx="unfinished', 'unclosed_quote', 2),
            ('[A]\nx="a" "b"', 'invalid_quote', 2),
            ('[A]\nx=a"b"', 'invalid_quote', 2),
        ]
        for text, code, line in cases:
            with self.subTest(code=code, text=text):
                with self.assertRaises(ContentError) as caught:
                    parse_ini(text, 'broken.ini')
                diagnostic = caught.exception.diagnostic
                self.assertEqual((diagnostic.code, diagnostic.source, diagnostic.line), (code, 'broken.ini', line))

    def test_explicit_legacy_encoding(self):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'legacy.ini'
            file.write_bytes('[A]\nNAME=caf\xe9'.encode('latin1'))
            with self.assertRaises(ContentError) as caught:
                read_ini(file)
            self.assertEqual(caught.exception.diagnostic.code, 'text_encoding')
            self.assertEqual(read_ini(file, 'cp1252').sections['A'].values['NAME'], 'caf\xe9')


class LutTests(unittest.TestCase):
    def test_linear_interpolation_and_clamping(self):
        table = parse_lut('# own fixture\n0|10\n10|30 ; comment\n30|-10\n', 'curve.lut')
        for x, expected in [(-5, 10), (0, 10), (5, 20), (10, 30), (20, 10), (30, -10), (99, -10)]:
            with self.subTest(x=x):
                self.assertEqual(table.sample(x), expected)

    def test_single_point_and_scientific_notation(self):
        table = parse_lut('1e2|-2.5e-1')
        self.assertEqual(table.sample(-1000), -.25)
        self.assertEqual(table.sample(1000), -.25)

    def test_malformed_and_ambiguous_tables(self):
        cases = [('', 'empty_lut', 0), ('0|1|2', 'invalid_lut_row', 1),
                 ('0|1\n0|2', 'unordered_lut', 2), ('1|1\n0|2', 'unordered_lut', 2),
                 ('a|1', 'invalid_number', 1), ('1|nan', 'nonfinite_number', 1),
                 ('inf|1', 'nonfinite_number', 1), ('1|', 'invalid_number', 1)]
        for text, code, line in cases:
            with self.subTest(text=text):
                with self.assertRaises(ContentError) as caught:
                    parse_lut(text, 'curve.lut')
                self.assertEqual(caught.exception.diagnostic.code, code)
                self.assertEqual(caught.exception.diagnostic.line, line)

    def test_nonfinite_samples_and_numeric_overflow(self):
        table = parse_lut('0|0\n1|1')
        for x in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ContentError):
                table.sample(x)
        wide = parse_lut('-1e308|0\n1e308|1')
        with self.assertRaises(ContentError) as caught:
            wide.sample(0)
        self.assertEqual(caught.exception.diagnostic.code, 'numeric_range')


class ConfigCliTests(unittest.TestCase):
    def test_cli_success_and_structured_error(self):
        with tempfile.TemporaryDirectory() as directory:
            file = Path(directory) / 'fixture.lut'
            file.write_text('0|2\n10|4', encoding='utf-8')
            output, errors = io.StringIO(), io.StringIO()
            with redirect_stdout(output), redirect_stderr(errors):
                code = main(['lut', str(file), '--sample', '5'])
            self.assertEqual(code, 0)
            self.assertEqual(json.loads(output.getvalue())['sample']['y'], 3)
            self.assertEqual(errors.getvalue(), '')
            output, errors = io.StringIO(), io.StringIO()
            with redirect_stdout(output), redirect_stderr(errors):
                code = main(['lut', str(file), '--sample', 'nan'])
            self.assertEqual(code, 2)
            self.assertEqual(output.getvalue(), '')
            self.assertEqual(json.loads(errors.getvalue())['diagnostics'][0]['code'], 'nonfinite_number')
