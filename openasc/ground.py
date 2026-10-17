"""Bounded static ground triangles and a BVH for vertical wheel-support queries.

This is geometric contact preparation, not rigid-body or tyre simulation.
"""
from array import array
from dataclasses import dataclass
import math
from pathlib import Path

from .config import ContentError, Diagnostic, finite_number
from .kn5 import IDENTITY, load_kn5, multiply, transform, values_from_bytes
from .scene import confined, inspect_scene
from .surfaces import load_surfaces, mesh_surface

MAX_TRIANGLES = 1_000_000
MAX_COORDINATE = 10_000_000


@dataclass
class Node:
    bounds: tuple
    start: int = 0
    end: int = 0
    left: object = None
    right: object = None


class Ground:
    def __init__(self):
        self.triangles = array('d')
        self.mesh_ids = array('I')
        self.meshes = []
        self.order = []
        self.root = None
        self.diagnostics = []
        self.skipped_degenerate = 0
        self.input_triangles = 0

    def add_mesh(self, mesh, world, surface, model):
        """Ignore render visibility; transform the physical mesh once on loading."""
        count = len(mesh.indices) // 6
        if self.input_triangles + count > MAX_TRIANGLES:
            raise ContentError('ground_limit', f'Ground input exceeds {MAX_TRIANGLES} triangles.', model)
        self.input_triangles += count
        vertices = values_from_bytes(mesh.vertices, 'f')
        points = [transform(world, vertices[i:i + 3]) for i in range(0, len(vertices), 11)]
        if any(not math.isfinite(v) or abs(v) > MAX_COORDINATE for p in points for v in p):
            raise ContentError('ground_range', 'Ground coordinates exceed the supported +/-10000000 metre range.', model)
        indices = values_from_bytes(mesh.indices, 'H')
        mesh_id = len(self.meshes)
        start = len(self.mesh_ids)
        for i in range(0, len(indices), 3):
            a, b, c = (points[index] for index in indices[i:i + 3])
            ux, uz, vx, vz = b[0] - a[0], b[2] - a[2], c[0] - a[0], c[2] - a[2]
            area = ux * vz - uz * vx
            if abs(area) <= 1e-12 * max(1., math.hypot(ux, uz) * math.hypot(vx, vz)):
                self.skipped_degenerate += 1
                continue
            self.triangles.extend((*a, *b, *c))
            self.mesh_ids.append(mesh_id)
        self.meshes.append({'name': mesh.name, 'model': model, 'surface': surface.as_dict(),
                            'triangles': len(self.mesh_ids) - start})

    def build(self):
        """Median BVH over projected triangle bounds; leaf tests retain full heights."""
        bounds, centres = [], []
        for i in range(len(self.mesh_ids)):
            tri = self.triangles[i * 9:i * 9 + 9]
            bounds.append(tuple(min(tri[a::3]) for a in range(3)) + tuple(max(tri[a::3]) for a in range(3)))
            centres.append(tuple((bounds[-1][a] + bounds[-1][a + 3]) * .5 for a in range(3)))
        self.order = list(range(len(self.mesh_ids)))

        def node(start, end):
            selected = self.order[start:end]
            box = tuple(min(bounds[i][a] for i in selected) for a in range(3)) + tuple(max(bounds[i][a] for i in selected) for a in range(3, 6))
            if end - start <= 12:
                return Node(box, start, end)
            axis = max((0, 2), key=lambda a: box[a + 3] - box[a])
            self.order[start:end] = sorted(selected, key=lambda i: centres[i][axis])
            middle = (start + end) // 2
            return Node(box, left=node(start, middle), right=node(middle, end))

        self.root = node(0, len(self.order)) if self.order else None

    def probe(self, origin, max_distance=1000.):
        """Nearest double-sided triangle below origin, inclusive of shared edges.

        Returns an upward geometric normal. Walls/vertical faces have no support
        projection and are excluded; this is not an arbitrary collision raycast.
        """
        if len(origin) != 3:
            raise ContentError('ground_origin', 'Probe origin requires three coordinates.')
        x, y, z = (finite_number(v, '<ground query>', 0) for v in origin)
        distance = finite_number(max_distance, '<ground query>', 0)
        if any(abs(v) > MAX_COORDINATE for v in (x, y, z)) or not 0 <= distance <= MAX_COORDINATE:
            raise ContentError('ground_query_range', 'Coordinates must be within +/-10000000 metres; distance must be between 0 and 10000000.')
        closest, result = distance, None
        stack = [self.root] if self.root else []
        while stack:
            node = stack.pop()
            lo_x, lo_y, lo_z, hi_x, hi_y, hi_z = node.bounds
            if x < lo_x - 1e-7 or x > hi_x + 1e-7 or z < lo_z - 1e-7 or z > hi_z + 1e-7 or lo_y > y + 1e-7 or hi_y < y - closest - 1e-7:
                continue
            if node.left is not None:
                stack.extend((node.right, node.left))
                continue
            for index in self.order[node.start:node.end]:
                a, b, c = (self.triangles[index * 9 + i:index * 9 + i + 3] for i in (0, 3, 6))
                u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
                area = u[0] * v[2] - u[2] * v[0]
                px, pz = x - a[0], z - a[2]
                beta = (px * v[2] - pz * v[0]) / area
                gamma = (u[0] * pz - u[2] * px) / area
                if min(beta, gamma, 1 - beta - gamma) < -1e-9:
                    continue
                height = a[1] + beta * u[1] + gamma * v[1]
                travel = y - height
                if travel < -1e-7 or travel > closest + 1e-7:
                    continue
                # Keep the first source triangle at equal heights for repeatability.
                if result is not None and (travel > closest - 1e-7) and index > result['triangle']:
                    continue
                normal = [u[1] * v[2] - u[2] * v[1], -area, u[0] * v[1] - u[1] * v[0]]
                magnitude = math.hypot(*normal)
                sign = 1 if normal[1] > 0 else -1
                mesh = self.meshes[self.mesh_ids[index]]
                closest = max(0., travel)
                result = {'position': [x, height, z], 'distance': closest,
                          'normal': [sign * n / magnitude for n in normal],
                          'mesh': mesh['name'], 'model': mesh['model'],
                          'triangle': index, 'surface': mesh['surface']}
        return result

    def summary(self):
        return {'scope': 'static_ground_query', 'triangles': len(self.mesh_ids),
                'input_triangles': self.input_triangles,
                'meshes': self.meshes, 'bvh_ready': self.root is not None,
                'skipped_vertical_or_degenerate': self.skipped_degenerate,
                'diagnostics': self.diagnostics,
                'has_errors': any(d['severity'] == 'error' for d in self.diagnostics)}


def load_ground(installation, track, layout=None, encoding='utf-8-sig'):
    manifest = inspect_scene(installation, track, layout, encoding)
    surfaces, warnings = load_surfaces(installation, track, layout, encoding)
    content = confined(Path(installation).resolve(), f'content/tracks/{track}', '<ground>')
    ground = Ground()
    ground.diagnostics.extend(manifest['diagnostics'] + warnings)
    ground.diagnostics.append(Diagnostic('ground_query_scope', 'Vertical static geometry support only: walls, rigid-body response, tyres, procedural surface bumps, audio and FFB are not simulated. Recorded surface coefficients are metadata.', manifest['config'], severity='warning').as_dict())
    for model in manifest['models']:
        if any(abs(v) > 1e-6 for v in model['rotation']):
            ground.diagnostics.append(Diagnostic('model_rotation_unsupported', f'{model["section"]} is omitted because its model rotation convention is unvalidated.', manifest['config']).as_dict())
            continue
        try:
            parsed = load_kn5(confined(content, model['file'], manifest['config']), model['file'])
        except (ContentError, OSError) as error:
            ground.diagnostics.append(error.diagnostic.as_dict() if isinstance(error, ContentError) else Diagnostic('resource_io', 'Cannot read ground model.', model['file']).as_dict())
            continue
        ground.diagnostics.extend(d for d in parsed.diagnostics if d['code'] not in ('textures_not_rendered', 'material_preview'))
        root = list(IDENTITY)
        root[12:15] = model['position']
        for mesh in parsed.meshes:
            if not mesh.active or not mesh.indices or mesh.lod[0] > 0:
                continue
            if 'WALL' in mesh.name.upper():
                ground.diagnostics.append(Diagnostic('ground_wall_omitted', f'{mesh.name!r}: wall collision is not implemented by the vertical support query.', model['file'], severity='warning').as_dict())
                continue
            try:
                surface = mesh_surface(mesh.name, surfaces)
            except ContentError as error:
                ground.diagnostics.append(Diagnostic(error.diagnostic.code, error.diagnostic.message, model['file'], severity='warning').as_dict())
                continue
            if surface is not None:
                ground.add_mesh(mesh, multiply(root, mesh.world), surface, model['file'])
    if not ground.mesh_ids:
        ground.diagnostics.append(Diagnostic('no_ground', 'No supported ground triangles could be assembled.', manifest['config']).as_dict())
    ground.build()
    return ground
