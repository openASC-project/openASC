"""Local first-drive assembly and a bounded, sequenced controller session."""
import hmac
import math
from pathlib import Path
import secrets
from threading import Event, RLock, Thread
import time

from .config import ContentError, Diagnostic
from .dynamics import Controls, Dynamics, DT
from .ground import load_ground
from .kn5 import IDENTITY, load_kn5, multiply, transform
from .placement import load_spawns
from .scene import confined
from .vehicle import load_vehicle
from .viewer import Preview, load_preview, make_server, prepare_materials
from .walls import load_body


def add_vehicle(preview, installation, vehicle):
    content = confined(Path(installation).resolve(), f'content/cars/{vehicle.car}', '<drive>')
    path = confined(content, vehicle.model['file'], '<drive>')
    parsed = load_kn5(path, f'content/cars/{vehicle.car}/{vehicle.model["file"]}')
    metadata = preview.metadata
    diagnostics = metadata['diagnostics']
    diagnostics.extend(d for d in parsed.diagnostics if d['code'] not in ('textures_not_rendered', 'material_preview'))
    textures = list(preview.textures)
    prepare_materials(path, parsed, textures, {}, diagnostics)
    material_offset = len(metadata['materials'])
    metadata['materials'].extend(parsed.materials)
    blob = bytearray(preview.geometry)
    angle = math.radians(vehicle.graphics_pitch_deg)
    graphics = list(IDENTITY)
    graphics[5], graphics[6], graphics[9], graphics[10] = math.cos(angle), math.sin(angle), -math.sin(angle), math.cos(angle)
    graphics[12:15] = vehicle.graphics_offset_m
    count = 0
    for mesh in parsed.meshes:
        if not (mesh.active and mesh.visible and mesh.renderable and mesh.indices and mesh.bounds) or mesh.lod[0] > 0:
            continue
        if len(blob) + len(mesh.vertices) + len(mesh.indices) > 384 * 1024 ** 2:
            raise ContentError('preview_limit', 'Combined track/car geometry exceeds 384 MiB.', parsed.source)
        while len(blob) % 4:
            blob.append(0)
        vertex_offset = len(blob)
        blob.extend(mesh.vertices)
        index_offset = len(blob)
        blob.extend(mesh.indices)
        corners = [transform(graphics, (mesh.bounds[x][0], mesh.bounds[y][1], mesh.bounds[z][2]))
                   for x in (0, 1) for y in (0, 1) for z in (0, 1)]
        bounds = [[operation(point[axis] for point in corners) for axis in range(3)] for operation in (min, max)]
        world = multiply(graphics, mesh.world)
        if not all(math.isfinite(n) and abs(n) <= 3.4028235e38 for n in world):
            raise ContentError('preview_transform', 'Vehicle graphics transform exceeds the float32 range.', parsed.source)
        metadata['meshes'].append({'name': mesh.name, 'model': parsed.source, 'material': material_offset + mesh.material,
                                   'world': world, 'bounds': bounds, 'lod': mesh.lod, 'dynamic': 'car',
                                   'vertex_offset': vertex_offset, 'vertex_bytes': len(mesh.vertices),
                                   'index_offset': index_offset, 'index_count': len(mesh.indices) // 2})
        count += 1
    if not count:
        raise ContentError('drive_no_car_geometry', 'No supported visible near-LOD car geometry was loaded.', parsed.source)
    metadata.update(geometry_bytes=len(blob), triangles=sum(d['index_count'] // 3 for d in metadata['meshes']),
                    textures=[texture.info for texture in textures], texture_preview_available=bool(textures))
    return Preview(metadata, bytes(blob), tuple(textures)), count


class Session:
    def __init__(self, dynamics, clock=time.monotonic):
        self.model, self.clock = dynamics, clock
        self.token = secrets.token_urlsafe(24)
        self.sequence = 0
        self.lock = RLock()
        self.last_input = clock()
        self.stop = Event()
        self.thread = None
        self.lag_events = 0

    def state(self, include_token=False):
        with self.lock:
            result = self.model.snapshot() | {'sequence': self.sequence, 'fixed_step': DT,
                                               'lag_events': self.lag_events}
            if include_token:
                result['token'] = self.token
                result['car'] = self.model.vehicle.car
                result['gears'] = len(self.model.vehicle.drivetrain['forward_ratios'])
            return result

    def command(self, payload):
        if (not isinstance(payload, dict) or not isinstance(payload.get('token'), str)
                or not payload['token'].isascii() or not hmac.compare_digest(payload['token'], self.token)):
            raise ContentError('drive_token', 'Invalid local controller token.')
        sequence = payload.get('sequence')
        if type(sequence) is not int or not 0 < sequence < 2 ** 53:
            raise ContentError('drive_sequence', 'Controller sequence must be a positive safe integer.')
        action = payload.get('action')
        required = {'token', 'sequence', 'action'} | ({'controls'} if action == 'input' else set())
        if action not in ('input', 'pause', 'resume', 'reset') or set(payload) != required:
            raise ContentError('drive_command', 'Unknown action or unexpected command fields.')
        controls = Controls.from_dict(payload['controls'], len(self.model.vehicle.drivetrain['forward_ratios'])) if action == 'input' else None
        with self.lock:
            if sequence <= self.sequence:
                raise ContentError('drive_stale', 'Stale input was rejected; reload if another tab controls this session.')
            if action == 'input':
                self.model.set_controls(controls)
            elif action == 'pause':
                self.model.pause()
            elif action == 'resume':
                self.model.resume()
            else:
                self.model.reset()
            self.sequence, self.last_input = sequence, self.clock()
            return self.state()

    def advance(self, steps):
        """Called by the clock thread; tests can advance exact steps without sleeping."""
        with self.lock:
            if not self.model.paused and self.clock() - self.last_input > .5:
                self.model.pause('Input heartbeat expired — resume to continue')
            for _ in range(steps):
                self.model.step()

    def start(self):
        def loop():
            previous, accumulator = self.clock(), 0.
            while not self.stop.wait(.002):
                now = self.clock()
                accumulator += max(0., now - previous)
                previous = now
                steps = int(accumulator / DT)
                if steps:
                    if steps > 8:
                        with self.lock:
                            self.lag_events += 1
                        accumulator = 0.
                        steps = 8
                    else:
                        accumulator -= steps * DT
                    self.advance(steps)
        self.thread = Thread(target=loop, name='openasc-fixed-step', daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)


def load_drive(installation, car, track, spawn='AC_PIT_0', layout=None, compound=None, mode='auto', encoding='utf-8-sig'):
    vehicle = load_vehicle(installation, car, compound, mode, encoding)
    ground = load_ground(installation, track, layout, encoding, include_walls=True)
    if ground.summary()['has_errors']:
        error = ContentError('drive_ground_errors', 'Ground loading errors prevent driving; inspect the ground report.')
        error.diagnostics = ground.diagnostics + [error.diagnostic.as_dict()]
        raise error
    spawns, diagnostics = load_spawns(installation, track, layout, encoding)
    if spawn not in spawns:
        error = ContentError('spawn_missing', 'Selected drive spawn is absent, ambiguous or unsupported.')
        error.diagnostics = diagnostics + [error.diagnostic.as_dict()]
        raise error
    body = load_body(installation, vehicle)
    model = Dynamics(vehicle, ground, spawns[spawn], ground.walls, body)
    preview = load_preview(installation, track, layout, encoding, hide_markers=True)
    preview, count = add_vehicle(preview, installation, vehicle)
    preview.metadata['diagnostics'].extend(diagnostics + vehicle.diagnostics + ground.diagnostics)
    if not ground.walls.mesh_ids:
        preview.metadata['diagnostics'].append(Diagnostic('wall_geometry_empty', 'No supported static WALL triangles were found; this scene has no indexed wall barriers.', '<drive>', severity='warning').as_dict())
    preview.metadata['diagnostics'].append(Diagnostic('first_drive_prototype', 'Independent planar four-patch dynamics with geometric road attachment and an upright collider-bounds body proxy against static WALL triangles. No full convex/6DOF collision, suspension dynamics, wheel animation, original tyre/drivetrain behavior, FFB, audio, AI or networking. Keyboard input; static load distribution and approximate graphics alignment.', '<drive>', severity='warning').as_dict())
    preview.metadata['wall_collision'] = ground.walls.summary() | {'body_center': body.center, 'body_half_extents': body.half,
                                                                 'body_source': body.source, 'body_shape': 'upright_collider_bounds'}
    preview.metadata['diagnostics'] = list({tuple(sorted(d.items())): d for d in preview.metadata['diagnostics']}.values())
    preview.metadata['has_errors'] = any(d['severity'] == 'error' for d in preview.metadata['diagnostics'])
    preview.metadata['scope'] = 'first_drive_prototype'
    preview.metadata['drive'] = {'car': car, 'spawn': spawn, 'car_meshes': count, 'fixed_step': DT,
                                  'model': 'independent_planar_four_patch', 'initial_world': model.world()}
    return preview, Session(model)


def serve_drive(installation, car, track, spawn='AC_PIT_0', layout=None, compound=None, mode='auto', encoding='utf-8-sig', port=4180):
    print('Loading car data, ground support and local track/car geometry...', flush=True)
    preview, session = load_drive(installation, car, track, spawn, layout, compound, mode, encoding)
    with make_server(preview, port, session) as server:
        session.start()
        print(f'Local first-drive prototype: http://127.0.0.1:{server.server_port}/', flush=True)
        print('Starts paused. Keyboard: W/S throttle/brake, A/D steer, Space handbrake, E/Q gears, P pause, R reset.', flush=True)
        print('Prototype limits: upright body proxy, no original physics, wheel animation, audio or FFB. Press Ctrl+C to stop.', flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            session.close()
    return 0
