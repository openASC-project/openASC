import unittest

from dds_fixtures import dds
from openasc.config import ContentError
from openasc.dds import parse_dds


class DdsTests(unittest.TestCase):
    def test_bc_variants_and_mip_offsets(self):
        for fourcc, size, expected in [(b'DXT1', 8, 'BC1'), (b'DXT3', 16, 'BC2'), (b'DXT5', 16, 'BC3')]:
            with self.subTest(format=expected):
                texture = parse_dds(dds(fourcc=fourcc, payload=bytes(size * 3), mips=3))
                self.assertEqual(texture.info['format'], expected)
                self.assertEqual([level['offset'] for level in texture.info['levels']], [128, 128 + size, 128 + size * 2])
                self.assertEqual([level['width'] for level in texture.info['levels']], [4, 2, 1])
                self.assertEqual(len(texture.data), 128 + size * 3)

    def test_non_multiple_of_four_and_partial_block_payload(self):
        texture = parse_dds(dds(width=5, height=3, payload=bytes(16)))
        self.assertEqual(texture.info['levels'][0]['length'], 16)
        with self.assertRaises(ContentError):
            parse_dds(dds(width=5, height=3, payload=bytes(15)))

    def test_raw_pitch_channel_masks_and_luminance(self):
        texture = parse_dds(dds(width=1, height=2, fourcc=b'\0' * 4, bits=24,
            pf_flags=0x40, masks=(0xff0000, 0xff00, 0xff, 0), flags=0x100f, pitch=4, payload=bytes(8)))
        self.assertEqual(texture.info['levels'][0]['row_pitch'], 4)
        self.assertTrue(texture.info['generate_mips'])
        luminance = parse_dds(dds(width=1, height=1, fourcc=b'\0' * 4, bits=16,
            pf_flags=0x20001, masks=(0xff, 0, 0, 0xff00), payload=b'\x40\x80'))
        self.assertEqual(luminance.info['format'], 'LUMINANCE')

    def test_linear_size_is_not_mistaken_for_raw_row_pitch(self):
        texture = parse_dds(dds(width=2, height=2, fourcc=b'\0' * 4, bits=32, pf_flags=0x41,
            masks=(0xff0000, 0xff00, 0xff, 0xff000000), flags=0x81007, pitch=16, payload=bytes(16)))
        self.assertEqual(texture.info['levels'][0]['row_pitch'], 8)

    def test_unflagged_alpha_mask_is_ignored_for_opaque_rgb(self):
        texture = parse_dds(dds(width=1, height=1, fourcc=b'\0' * 4, bits=32, pf_flags=0x40,
            masks=(0xff0000, 0xff00, 0xff, 0xff000000), payload=bytes(4)))
        self.assertEqual(texture.info['masks'][3], 0)

    def test_unknown_formats_dimensions_cubes_and_corrupt_headers(self):
        cases = [(b'bad', 'dds_header'), (dds(fourcc=b'DX10'), 'dds_format'),
                 (dds(width=0), 'dds_limit'), (dds(width=8193), 'dds_limit'),
                 (dds(caps2=0x200), 'dds_surface'), (dds(mips=4), 'dds_mips'),
                 (dds(payload=b''), 'dds_truncated')]
        for data, code in cases:
            with self.subTest(code=code):
                with self.assertRaises(ContentError) as caught:
                    parse_dds(data, 'own.dds')
                self.assertEqual(caught.exception.diagnostic.code, code)
                self.assertEqual(caught.exception.diagnostic.source, 'own.dds')

    def test_invalid_and_overlapping_masks_are_rejected(self):
        for masks in [(0xf800, 0x7e0, 0x1f, 0x8000), (0xf500, 0x7e0, 0x1f, 0), (0xff0000, 0xff00, 0xff, 0)]:
            with self.subTest(masks=masks):
                with self.assertRaises(ContentError) as caught:
                    parse_dds(dds(width=1, height=1, fourcc=b'\0' * 4, bits=16, pf_flags=0x41 if masks[3] else 0x40, masks=masks, payload=bytes(2)))
                self.assertEqual(caught.exception.diagnostic.code, 'dds_masks')
