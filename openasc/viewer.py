"""Local-only geometry preview server. No game content is published or exported."""
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
from urllib.parse import urlsplit

from .config import ContentError, Diagnostic
from .dds import read_embedded
from .kn5 import IDENTITY, load_kn5, multiply
from .scene import confined, inspect_scene

ASSETS = Path(__file__).resolve().parents[1] / 'viewer'


@dataclass
class Preview:
    metadata: dict
    geometry: bytes
    textures: tuple = ()


def prepare_materials(path, parsed, textures, cache, diagnostics):
    """Resolve only embedded txDiffuse resources; advanced samplers remain explicit gaps."""
    available = {}
    ambiguous = set()
    for descriptor in parsed.textures:
        key = descriptor['name'].casefold()
        if key in available:
            ambiguous.add(key)
        available[key] = descriptor
    for material in parsed.materials:
        material['preview_texture'] = None
        material['preview_blend'] = 1 if material['blend'] == 1 else 0
        material['preview_alpha_test'] = material['alpha_tested'] or material['blend'] == 2
        material['preview_depth'] = material['depth_mode'] if material['depth_mode'] in (0, 1, 2) else 0
        if material['blend'] not in (0, 1, 2) or material['depth_mode'] not in (0, 1, 2):
            diagnostics.append(Diagnostic('material_state_unsupported', f'{material["name"]}: unknown blend/depth state falls back to a known preview state.', parsed.source, severity='warning').as_dict())
        references = [mapping['texture'] for mapping in material['textures'] if mapping['sampler'] == 'txDiffuse']
        name = references[0] if len(references) == 1 else None
        descriptor = available.get(name.casefold()) if name else None
        if not descriptor or not descriptor['active'] or name.casefold() in ambiguous:
            diagnostics.append(Diagnostic('diffuse_texture_missing', f'{material["name"]}: a unique active embedded txDiffuse resource is unavailable; diagnostic colour will be used.', parsed.source, severity='warning').as_dict())
            continue
        key = (str(path), descriptor['offset'], descriptor['size_bytes'])
        if key not in cache:
            try:
                texture = read_embedded(path, descriptor)
                decoded = sum(level['width'] * level['height'] * 4 for level in texture.info['levels'])
                if texture.info['generate_mips']:
                    decoded = 0
                    width, height = texture.info['width'], texture.info['height']
                    while True:
                        decoded += width * height * 4
                        if width == height == 1:
                            break
                        width, height = max(1, width // 2), max(1, height // 2)
                used = sum(item.info['decoded_budget'] for item in textures)
                if used + decoded > 256 * 1024 ** 2 or sum(len(item.data) for item in textures) + len(texture.data) > 128 * 1024 ** 2:
                    raise ContentError('texture_budget', 'Scene texture data exceeds the 128 MiB source / 256 MiB decoded preview budget.', name)
                texture.info.update(id=len(textures), name=name, decoded_budget=decoded)
                cache[key] = len(textures)
                textures.append(texture)
            except (ContentError, OSError) as error:
                reason = error.diagnostic.code + ': ' + error.diagnostic.message if isinstance(error, ContentError) else 'Cannot read embedded texture.'
                diagnostics.append(Diagnostic('texture_unsupported', f'{name}: {reason} Diagnostic colour will be used.', parsed.source, severity='warning', byte_offset=descriptor['offset']).as_dict())
                cache[key] = None
        material['preview_texture'] = cache[key]


class PreviewError(ContentError):
    def __init__(self, diagnostics, source):
        super().__init__('no_preview_geometry', 'No supported visible static geometry could be loaded.', source)
        self.diagnostics = diagnostics + [self.diagnostic.as_dict()]


def load_preview(installation, track, layout=None, encoding='utf-8-sig'):
    manifest = inspect_scene(installation, track, layout, encoding)
    content = confined(Path(installation).resolve(), f'content/tracks/{track}', '<scene>')
    diagnostics = list(manifest['diagnostics'])
    draws, materials, blob, road_bounds = [], [], bytearray(), []
    textures, texture_cache = [], {}
    hidden, later_lod = 0, 0
    for model in manifest['models']:
        if any(abs(value) > 3.4028235e38 for value in model['position']):
            diagnostics.append(Diagnostic('preview_transform', f'{model["section"]} position exceeds the float32 preview range.', manifest['config']).as_dict())
            continue
        if any(abs(value) > 1e-6 for value in model['rotation']):
            diagnostics.append(Diagnostic('model_rotation_unsupported', f'{model["section"]} has a nonzero model rotation; it is skipped until the rotation convention is validated.', manifest['config']).as_dict())
            continue
        try:
            parsed = load_kn5(confined(content, model['file'], manifest['config']), model['file'])
        except ContentError as error:
            diagnostics.append(error.diagnostic.as_dict())
            continue
        except OSError:
            diagnostics.append(Diagnostic('resource_io', f'Cannot read {model["file"]!r}.', manifest['config']).as_dict())
            continue
        diagnostics.extend(item for item in parsed.diagnostics if item['code'] not in ('textures_not_rendered', 'material_preview'))
        prepare_materials(confined(content, model['file'], manifest['config']), parsed, textures, texture_cache, diagnostics)
        material_offset = len(materials)
        materials.extend(parsed.materials)
        root = list(IDENTITY)
        root[12:15] = model['position']
        for mesh in parsed.meshes:
            if mesh.active and mesh.bounds and 'ROAD' in mesh.name.upper():
                road_bounds.append([[mesh.bounds[side][axis] + model['position'][axis] for axis in range(3)] for side in range(2)])
            if not (mesh.active and mesh.visible and mesh.renderable and mesh.indices and mesh.bounds):
                hidden += 1
                continue
            if mesh.lod[0] > 0:
                later_lod += 1
                continue
            if len(blob) + len(mesh.vertices) + len(mesh.indices) > 384 * 1024 ** 2:
                raise ContentError('preview_limit', 'Combined preview geometry exceeds 384 MiB.', manifest['config'])
            while len(blob) % 4:
                blob.append(0)
            vertex_offset = len(blob)
            blob.extend(mesh.vertices)
            index_offset = len(blob)
            blob.extend(mesh.indices)
            world = multiply(root, mesh.world)
            bounds = [[mesh.bounds[side][axis] + model['position'][axis] for axis in range(3)] for side in range(2)]
            if not all(math.isfinite(value) and abs(value) <= 3.4028235e38 for value in (*world, *bounds[0], *bounds[1])):
                raise ContentError('preview_transform', 'Combined model transform exceeds the float32 preview range.', manifest['config'])
            draws.append({'name': mesh.name, 'model': model['file'], 'material': material_offset + mesh.material,
                          'world': world, 'bounds': bounds, 'lod': mesh.lod,
                          'vertex_offset': vertex_offset, 'vertex_bytes': len(mesh.vertices),
                          'index_offset': index_offset, 'index_count': len(mesh.indices) // 2})
    if hidden or later_lod:
        diagnostics.append(Diagnostic('preview_visibility', f'{hidden} inactive/hidden/non-renderable/empty meshes and {later_lod} later-LOD meshes are omitted. Camera-distance LOD switching is not implemented.', manifest['config'], severity='warning').as_dict())
    diagnostics.append(Diagnostic('material_approximation', 'Preview uses embedded txDiffuse, basic lighting, a 0.5 alpha cutoff, mesh-sorted straight-alpha blending, and recorded depth modes with a small read-only overlay bias. Original shaders, multilayer/detail/normal maps, alpha-to-coverage, and shadows are not reproduced.', manifest['config'], severity='warning').as_dict())
    if not draws:
        raise PreviewError(diagnostics, manifest['config'])
    minimum = [min(draw['bounds'][0][axis] for draw in draws) for axis in range(3)]
    maximum = [max(draw['bounds'][1][axis] for draw in draws) for axis in range(3)]
    metadata = {'track': track, 'layout': layout, 'scope': 'static_geometry_preview', 'geometry_loaded': True,
                'texture_preview_available': bool(textures), 'original_shaders': False, 'bounds': [minimum, maximum],
                'textures': [texture.info for texture in textures],
                'meshes': draws, 'materials': materials, 'diagnostics': diagnostics,
                'has_errors': any(item['severity'] == 'error' for item in diagnostics),
                'triangles': sum(draw['index_count'] // 3 for draw in draws), 'geometry_bytes': len(blob)}
    metadata['road_focus_bounds'] = [[min(bounds[0][axis] for bounds in road_bounds) for axis in range(3)],
                                     [max(bounds[1][axis] for bounds in road_bounds) for axis in range(3)]] if road_bounds else None
    return Preview(metadata, bytes(blob), tuple(textures))


def make_server(preview, port=4180):
    metadata = json.dumps(preview.metadata, ensure_ascii=True, allow_nan=False).encode('utf-8')

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            expected = {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}
            host = self.headers.get('Host', '')
            origin = self.headers.get('Origin')
            if host not in expected or (origin and origin not in {f'http://{value}' for value in expected}):
                self.send_error(403, 'This preview accepts local same-origin requests only.')
                return
            path = urlsplit(self.path).path
            routes = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/index.html': ('index.html', 'text/html; charset=utf-8'),
                      '/viewer.js': ('viewer.js', 'text/javascript; charset=utf-8'),
                      '/dds.js': ('dds.js', 'text/javascript; charset=utf-8'),
                      '/viewer.css': ('viewer.css', 'text/css; charset=utf-8')}
            if path == '/api/scene':
                data, mime = metadata, 'application/json'
            elif path == '/geometry.bin':
                data, mime = preview.geometry, 'application/octet-stream'
            elif match := re.fullmatch(r'/textures/(\d{1,6})\.bin', path):
                index = int(match[1])
                if index >= len(preview.textures):
                    self.send_error(404)
                    return
                data, mime = preview.textures[index].data, 'application/octet-stream'
            elif path in routes:
                file, mime = routes[path]
                data = (ASSETS / file).read_bytes()
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format, *args):
            pass

    return ThreadingHTTPServer(('127.0.0.1', port), Handler)


def serve_preview(installation, track, layout=None, encoding='utf-8-sig', port=4180):
    print('Loading local geometry and supported embedded diffuse textures...', flush=True)
    preview = load_preview(installation, track, layout, encoding)
    with make_server(preview, port) as server:
        print(f'Local viewer: http://127.0.0.1:{server.server_port}/', flush=True)
        print(f'{len(preview.metadata["meshes"])} visible meshes; {preview.metadata["triangles"]} triangles; '
              f'{len(preview.metadata["diagnostics"])} diagnostics. Press Ctrl+C to stop.', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
    return 0
