"""Validate a bounded legacy DDS subset for the local texture preview."""
from dataclasses import dataclass
import struct

from .config import ContentError


@dataclass
class Texture:
    info: dict
    data: bytes


def parse_dds(data, source='<dds>'):
    def fail(code, message):
        raise ContentError(code, message, source)

    if len(data) < 128 or data[:4] != b'DDS ':
        fail('dds_header', 'Missing or truncated legacy DDS header.')
    header = struct.unpack_from('<31I', data, 4)
    if header[0] != 124 or header[18] != 32:
        fail('dds_header', 'Unsupported DDS header or pixel-format structure size.')
    flags, height, width, pitch, depth, mip_count = header[1:7]
    if not width or not height or max(width, height) > 8192 or width * height > 16_777_216:
        fail('dds_limit', 'Texture dimensions exceed 8192 per axis or 16777216 pixels, or are empty.')
    if depth > 1 or header[27] & (0xfe00 | 0x200000):
        fail('dds_surface', 'Cube and volume textures are not supported by this 2D preview.')
    mip_count = mip_count or 1
    if mip_count > max(width, height).bit_length():
        fail('dds_mips', 'Declared mip count exceeds the 2D image chain.')
    pf_flags, fourcc, bits = header[19:22]
    masks = list(header[22:26])
    levels, offset = [], 128
    code = struct.pack('<I', fourcc)
    if pf_flags & 4:
        formats = {b'DXT1': ('BC1', 8), b'DXT3': ('BC2', 16), b'DXT5': ('BC3', 16)}
        if code not in formats:
            fail('dds_format', f'Unsupported FourCC {code!r}; DX10, BC4-7, and premultiplied variants are not decoded.')
        format_name, block_size = formats[code]
        w, h = width, height
        for _ in range(mip_count):
            length = ((w + 3) // 4) * ((h + 3) // 4) * block_size
            if offset + length > len(data):
                fail('dds_truncated', 'Block-compressed mip data ends before the declared chain.')
            levels.append({'width': w, 'height': h, 'offset': offset, 'length': length})
            offset += length
            w, h = max(1, w // 2), max(1, h // 2)
    elif pf_flags & (0x40 | 0x20000):
        format_name = 'LUMINANCE' if pf_flags & 0x20000 else 'RGB'
        if bits not in (8, 16, 24, 32) or (format_name == 'RGB' and bits == 8):
            fail('dds_format', 'Unsupported uncompressed pixel bit count.')
        if not pf_flags & 1:
            masks[3] = 0  # XRGB storage does not imply valid alpha.
        used = 0
        for mask in masks:
            if mask:
                shift = (mask & -mask).bit_length() - 1
                normalized = mask >> shift
                if normalized & (normalized + 1) or mask >= 1 << bits or used & mask:
                    fail('dds_masks', 'Pixel channel masks must be contiguous, in range, and nonoverlapping.')
                used |= mask
        if not masks[0] or (format_name == 'RGB' and not all(masks[:3])):
            fail('dds_masks', 'Required colour or luminance channels are missing.')
        if format_name == 'LUMINANCE' and (masks[1] or masks[2]):
            fail('dds_masks', 'Luminance data must not define separate green or blue channels.')
        if pf_flags & 1 and not masks[3]:
            fail('dds_masks', 'Alpha is flagged but its channel mask is missing.')
        tight = width * (bits // 8)
        row_pitch = pitch if flags & 8 else tight
        if row_pitch < tight or row_pitch > tight + 65536:
            fail('dds_pitch', 'Unsupported or invalid uncompressed row pitch.')
        length = row_pitch * height
        if offset + length > len(data):
            fail('dds_truncated', 'Uncompressed base-level data is incomplete.')
        levels.append({'width': width, 'height': height, 'offset': offset,
                       'length': length, 'row_pitch': row_pitch})
        offset += length
    else:
        fail('dds_format', 'Unsupported DDS pixel format.')
    return Texture({'width': width, 'height': height, 'format': format_name, 'bits': bits,
                    'masks': masks, 'levels': levels, 'source_mips': mip_count,
                    'generate_mips': format_name in ('RGB', 'LUMINANCE'),
                    'size_bytes': offset}, bytes(data[:offset]))


def read_embedded(path, descriptor):
    if descriptor['size_bytes'] > 128 * 1024 ** 2:
        raise ContentError('dds_limit', 'Embedded texture exceeds 128 MiB.', descriptor['name'])
    with path.open('rb') as stream:
        stream.seek(descriptor['offset'])
        data = stream.read(descriptor['size_bytes'])
    if len(data) != descriptor['size_bytes']:
        raise ContentError('dds_truncated', 'Embedded texture changed or ended while reading.', descriptor['name'])
    return parse_dds(data, descriptor['name'])
