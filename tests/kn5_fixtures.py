"""Original minimal binary fixtures. No game or SDK content is included."""
import struct
from openasc.kn5 import IDENTITY


def string(value):
    encoded = value.encode('utf-8')
    return struct.pack('<I', len(encoded)) + encoded


def base(children, matrix=IDENTITY, active=True, name='root'):
    return (struct.pack('<I', 1) + string(name) + struct.pack('<IB', len(children), active)
            + struct.pack('<16f', *matrix) + b''.join(children))


def mesh(name='fixture_mesh', material=0, indices=(0, 2, 1), renderable=True, skinned=False,
         points=((0., 0., 0.), (1., 0., 0.), (0., 0., 1.))):
    result = struct.pack('<I', 3 if skinned else 2) + string(name) + struct.pack('<IBBBB', 0, 1, 1, 1, 0)
    if skinned:
        result += struct.pack('<I', 1) + string('fixture_bone') + struct.pack('<16f', *IDENTITY)
    result += struct.pack('<I', len(points))
    for point in points:
        result += struct.pack('<11f', *point, 0., 1., 0., 0., 0., 1., 0., 0.)
        if skinned:
            result += struct.pack('<8f', 1., 0., 0., 0., 0., -1., -1., -1.)
    result += struct.pack('<I', len(indices)) + struct.pack(f'<{len(indices)}H', *indices)
    result += struct.pack('<IIff', material, 0, 0., 0.)
    if not skinned:
        result += struct.pack('<ffffB', 0., 0., 0., 1., renderable)
    return result


def file_bytes(node=None, version=6, texture=b'original fixture bytes', blend=0, alpha_tested=False, depth=0):
    header = b'sc6969' + struct.pack('<I', version)
    if version == 6:
        header += struct.pack('<I', 0)
    textures = struct.pack('<II', 1, 1) + string('fixture.dds') + struct.pack('<I', len(texture)) + texture
    materials = (struct.pack('<I', 1) + string('fixture_material') + string('fixture_shader')
                 + struct.pack('<BBII', blend, alpha_tested, depth, 1) + string('ksDiffuse')
                 + struct.pack('<10f', .7, *([0.] * 9)) + struct.pack('<I', 1)
                 + string('txDiffuse') + struct.pack('<I', 0) + string('fixture.dds'))
    return header + textures + materials + (node if node is not None else base([mesh()]))
