"""Bounded KN5 v5/v6 static geometry reader. No assets are extracted to disk."""
from array import array
from dataclasses import dataclass, replace
import math
from pathlib import Path
import struct
import sys

from .config import ContentError, Diagnostic

IDENTITY = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)


def multiply(a, b):
    """Column-major composition; KN5's row-vector storage has matching flat order."""
    return tuple(sum(a[k * 4 + row] * b[column * 4 + k] for k in range(4))
                 for column in range(4) for row in range(4))


def transform(matrix, point):
    return tuple(sum(matrix[column * 4 + row] * point[column] for column in range(3))
                 + matrix[12 + row] for row in range(3))


class Kn5Error(ContentError):
    def __init__(self, code, message, source, offset):
        super().__init__(code, message, source)
        self.diagnostic = replace(self.diagnostic, byte_offset=offset)


class Reader:
    def __init__(self, stream, source):
        self.stream, self.source = stream, source
        stream.seek(0, 2)
        self.size = stream.tell()
        stream.seek(0)
        self.string_bytes = 0
        if self.size > 2 * 1024 ** 3:
            self.fail('kn5_limit', 'Files larger than 2 GiB are unsupported.')

    @property
    def offset(self):
        return self.stream.tell()

    def fail(self, code, message):
        raise Kn5Error(code, message, self.source, self.offset)

    def require(self, count):
        if count < 0 or count > self.size - self.offset:
            self.fail('kn5_truncated', f'Expected {count} bytes within the remaining file.')

    def read(self, count):
        self.require(count)
        result = self.stream.read(count)
        if len(result) != count:
            self.fail('kn5_truncated', 'File changed or ended during reading.')
        return result

    def skip(self, count):
        self.require(count)
        self.stream.seek(count, 1)

    def u32(self):
        return struct.unpack('<I', self.read(4))[0]

    def count(self, label, limit=100_000):
        value = self.u32()
        if value > limit:
            self.fail('kn5_limit', f'{label} count {value} exceeds the supported limit {limit}.')
        return value

    def flag(self):
        value = self.read(1)[0]
        if value not in (0, 1):
            self.fail('kn5_flag', f'Expected a boolean flag, got {value}.')
        return bool(value)

    def string(self):
        length = self.count('String byte', 1_048_576)
        self.string_bytes += length
        if self.string_bytes > 32 * 1024 ** 2:
            self.fail('kn5_limit', 'Combined string data exceeds 32 MiB.')
        try:
            return self.read(length).decode('utf-8')
        except UnicodeDecodeError as error:
            self.fail('kn5_string', f'Invalid UTF-8 string: {error.reason}.')

    def floats(self, count):
        result = struct.unpack(f'<{count}f', self.read(count * 4))
        if not all(math.isfinite(value) for value in result):
            self.fail('kn5_nonfinite', 'A numeric field contains NaN or infinity.')
        return result


@dataclass
class Mesh:
    name: str
    material: int
    world: tuple
    vertices: bytes
    indices: bytes
    active: bool
    visible: bool
    renderable: bool
    lod: tuple
    bounds: tuple | None


@dataclass
class Kn5Scene:
    source: str
    version: int
    textures: list
    materials: list
    meshes: list
    node_count: int
    diagnostics: list

    def summary(self):
        return {'source': self.source, 'version': self.version, 'scope': 'static_geometry_preview',
                'textures': len(self.textures), 'materials': len(self.materials), 'nodes': self.node_count,
                'static_meshes': len(self.meshes),
                'triangles': sum(len(mesh.indices) // 6 for mesh in self.meshes),
                'diagnostics': self.diagnostics}


def values_from_bytes(data, kind):
    values = array(kind)
    values.frombytes(data)
    if sys.byteorder != 'little':
        values.byteswap()
    return values


def load_kn5(path, source=None):
    source = source or Path(path).name
    with Path(path).open('rb') as stream:
        reader = Reader(stream, source)
        return _load(reader)


def _load(r):
    if r.read(6) != b'sc6969':
        r.fail('kn5_signature', 'Unsupported KN5 signature; protected/compressed formats are not decoded.')
    version = r.u32()
    if version not in (5, 6):
        r.fail('kn5_version', f'Unsupported KN5 version {version}.')
    diagnostics = []

    def warning(code, message):
        diagnostics.append(Diagnostic(code, message, r.source, severity='warning', byte_offset=r.offset).as_dict())

    if version == 6 and r.u32() != 0:
        warning('kn5_header_field', 'Nonzero v6 header field is retained only as an unsupported observation.')
    textures = []
    for _ in range(r.count('Texture')):
        active, name, length = r.u32(), r.string(), r.u32()
        textures.append({'name': name, 'active': active == 1, 'size_bytes': length})
        r.skip(length)
    materials = []
    properties_total = mappings_total = 0
    for _ in range(r.count('Material', 10_000)):
        material = {'name': r.string(), 'shader': r.string(), 'blend': r.read(1)[0],
                    'alpha_tested': r.flag(), 'depth_mode': r.u32(), 'properties': {}, 'textures': []}
        properties_count = r.count('Shader property', 10_000)
        properties_total += properties_count
        if properties_total > 100_000:
            r.fail('kn5_limit', 'Combined shader property count exceeds 100000.')
        for _ in range(properties_count):
            name, values = r.string(), r.floats(10)
            if name in material['properties']:
                r.fail('kn5_property', f'Duplicate shader property {name!r}.')
            material['properties'][name] = values
        mappings_count = r.count('Texture mapping', 10_000)
        mappings_total += mappings_count
        if mappings_total > 100_000:
            r.fail('kn5_limit', 'Combined texture mapping count exceeds 100000.')
        for _ in range(mappings_count):
            material['textures'].append({'sampler': r.string(), 'slot': r.u32(), 'texture': r.string()})
        materials.append(material)
    if textures:
        warning('textures_not_rendered', f'{len(textures)} embedded textures are indexed but not decoded or displayed.')
    if materials:
        warning('material_preview', 'Original shaders and alpha compositing are not implemented; the viewer uses opaque diagnostic material colours and basic lighting.')
    meshes = []
    nodes = 0
    geometry_bytes = 0

    def node(parent, ancestor_active, depth):
        nonlocal nodes, geometry_bytes
        nodes += 1
        if nodes > 200_000 or depth > 128:
            r.fail('kn5_limit', 'Node count or hierarchy depth exceeds the supported limit.')
        kind, name, children, active = r.u32(), r.string(), r.count('Child node'), r.flag()
        active = ancestor_active and active
        world = parent
        if kind == 1:
            local = r.floats(16)
            if any(abs(local[i]) > 1e-5 for i in (3, 7, 11)) or abs(local[15] - 1) > 1e-5:
                r.fail('kn5_transform', f'Non-affine transform on {name!r} is unsupported.')
            world = multiply(parent, local)
            if not all(math.isfinite(v) and abs(v) <= 3.4028235e38 for v in world):
                r.fail('kn5_transform', 'Composed transform exceeds the supported numeric range.')
        elif kind in (2, 3):
            casts_shadows, visible, transparent = r.flag(), r.flag(), r.flag()
            if kind == 3:
                for _ in range(r.count('Bone')):
                    r.string()
                    r.floats(16)
            count = r.count('Vertex', 2_000_000)
            stride = 44 if kind == 2 else 76
            geometry_bytes += count * stride
            if geometry_bytes > 256 * 1024 ** 2:
                r.fail('kn5_limit', 'Total geometry data exceeds 256 MiB.')
            raw = r.read(count * stride)
            indices_count = r.count('Index', 12_000_000)
            if indices_count % 3:
                r.fail('kn5_indices', 'Triangle index count must be divisible by three.')
            geometry_bytes += indices_count * 2
            if geometry_bytes > 256 * 1024 ** 2:
                r.fail('kn5_limit', 'Total geometry data exceeds 256 MiB.')
            indices = r.read(indices_count * 2)
            material, layer = r.u32(), r.u32()
            lod = r.floats(2)
            renderable = True
            if kind == 2:
                r.floats(4)  # Stored bounding sphere; compute independent bounds from geometry.
                renderable = r.flag()
            if material >= len(materials):
                r.fail('kn5_material', f'Material index {material} is outside the material table.')
            vertices = values_from_bytes(raw, 'f')
            if not all(math.isfinite(value) for value in vertices):
                r.fail('kn5_nonfinite', f'Nonfinite vertex data in {name!r}.')
            if indices and max(values_from_bytes(indices, 'H')) >= count:
                r.fail('kn5_indices', f'An index in {name!r} is outside the vertex array.')
            if kind == 3:
                warning('skinned_mesh_skipped', f'Skinned mesh {name!r} is parsed for structure but not displayed.')
            else:
                minimum, maximum = [math.inf] * 3, [-math.inf] * 3
                for index in range(0, len(vertices), 11):
                    point = transform(world, vertices[index:index + 3])
                    if not all(math.isfinite(value) and abs(value) <= 3.4028235e38 for value in point):
                        r.fail('kn5_transform', 'Transformed geometry exceeds the supported numeric range.')
                    for axis in range(3):
                        minimum[axis] = min(minimum[axis], point[axis])
                        maximum[axis] = max(maximum[axis], point[axis])
                bounds = (minimum, maximum) if count else None
                meshes.append(Mesh(name, material, world, raw, indices, active, visible, renderable, lod, bounds))
        else:
            r.fail('kn5_node_type', f'Unsupported node type {kind} on {name!r}; parsing stops instead of guessing its length.')
        for _ in range(children):
            node(world, active, depth + 1)

    node(IDENTITY, True, 0)
    if r.offset != r.size:
        r.fail('kn5_trailing_data', f'{r.size - r.offset} unexplained trailing bytes remain.')
    return Kn5Scene(r.source, version, textures, materials, meshes, nodes, diagnostics)
