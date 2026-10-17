"""Track spawn transforms and a geometric four-wheel placement preflight."""
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import re

from .config import ContentError, Diagnostic
from .ground import load_ground
from .kn5 import IDENTITY, load_kn5, multiply
from .scene import confined, inspect_scene
from .vehicle import load_vehicle

SPAWN = re.compile(r'^AC_(?:(?:PIT|START)_(?:0|[1-9][0-9]*)|HOTLAP_START_0)$')


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


@dataclass(frozen=True)
class Spawn:
    name: str
    model: str
    world: tuple
    position: tuple
    left: tuple
    up: tuple
    forward: tuple


def spawn_transform(name, model, world):
    """Accept a rigid, right-handed source basis; never repair a skewed spawn."""
    if (len(world) != 16 or any(abs(world[i]) > 1e-5 for i in (3, 7, 11))
            or abs(world[15] - 1) > 1e-5):
        raise ContentError('spawn_basis', f'{name!r} requires an affine 4x4 source transform.', model)
    axes = [tuple(world[i:i + 3]) for i in (0, 4, 8)]
    left, up, forward = axes
    determinant = dot(left, (up[1] * forward[2] - up[2] * forward[1],
                              up[2] * forward[0] - up[0] * forward[2],
                              up[0] * forward[1] - up[1] * forward[0]))
    if (not all(math.isfinite(value) for value in world)
            or any(abs(math.hypot(*axis) - 1) > 1e-3 for axis in axes)
            or any(abs(dot(axes[a], axes[b])) > 1e-3 for a, b in ((0, 1), (0, 2), (1, 2)))
            or abs(determinant - 1) > 1e-3 or up[1] <= .5
            or any(abs(v) > 10_000_000 for v in world[12:15])):
        raise ContentError('spawn_basis', f'{name!r} does not have a supported finite upright rigid basis.', model)
    return Spawn(name, model, tuple(world), tuple(world[12:15]), left, up, forward)


def load_spawns(installation, track, layout=None, encoding='utf-8-sig'):
    manifest = inspect_scene(installation, track, layout, encoding)
    content = confined(Path(installation).resolve(), f'content/tracks/{track}', '<spawns>')
    diagnostics, spawns = list(manifest['diagnostics']), {}
    duplicates = set()
    for model in manifest['models']:
        if any(abs(v) > 1e-6 for v in model['rotation']):
            diagnostics.append(Diagnostic('model_rotation_unsupported', f'{model["section"]} spawn nodes are omitted until model rotation is validated.', manifest['config']).as_dict())
            continue
        try:
            parsed = load_kn5(confined(content, model['file'], manifest['config']), model['file'])
        except (ContentError, OSError) as error:
            diagnostics.append(error.diagnostic.as_dict() if isinstance(error, ContentError) else Diagnostic('resource_io', 'Cannot read spawn model.', model['file']).as_dict())
            continue
        root = list(IDENTITY)
        root[12:15] = model['position']
        for node in parsed.node_transforms:
            if not node.active or not node.name.upper().startswith(('AC_PIT_', 'AC_START_', 'AC_HOTLAP_START_')):
                continue
            if node.kind in (2, 3) and node.parent >= 0:
                parent = parsed.node_transforms[node.parent]
                if parent.kind == 1 and parent.name == node.name and parent.world == node.world:
                    # A named transform plus its same-named attached cube is one marker.
                    continue
            if node.kind == 3:
                diagnostics.append(Diagnostic('spawn_skinned', f'{node.name!r}: a standalone skinned marker is unsupported.', model['file'], severity='warning').as_dict())
                continue
            if not SPAWN.fullmatch(node.name):
                diagnostics.append(Diagnostic('spawn_name', f'{node.name!r} is outside the canonical supported spawn naming subset.', model['file'], severity='warning').as_dict())
                continue
            if node.name in spawns or node.name in duplicates:
                spawns.pop(node.name, None)
                duplicates.add(node.name)
                diagnostics.append(Diagnostic('spawn_duplicate', f'{node.name!r} occurs more than once and is omitted.', model['file']).as_dict())
                continue
            try:
                spawns[node.name] = spawn_transform(node.name, model['file'], multiply(root, node.world))
            except ContentError as error:
                diagnostics.append(error.diagnostic.as_dict())
    if not spawns:
        diagnostics.append(Diagnostic('no_spawns', 'No active unambiguous supported spawn remains.', manifest['config']).as_dict())
    return spawns, diagnostics


def inspect_spawns(installation, track, layout=None, encoding='utf-8-sig'):
    spawns, diagnostics = load_spawns(installation, track, layout, encoding)
    return {'scope': 'static_spawn_transforms', 'track': track, 'layout': layout,
            'convention': 'local +X left, +Y up, +Z forward; no Euler conversion',
            'spawns': [asdict(spawns[name]) for name in sorted(spawns)],
            'diagnostics': diagnostics, 'has_errors': any(d['severity'] == 'error' for d in diagnostics)}


def inspect_placement(installation, car, track, spawn='AC_PIT_0', layout=None,
                      compound=None, mode='auto', encoding='utf-8-sig'):
    vehicle = load_vehicle(installation, car, compound, mode, encoding)
    spawns, diagnostics = load_spawns(installation, track, layout, encoding)
    if spawn not in spawns:
        error = ContentError('spawn_missing', f'Selected spawn {spawn!r} is absent, ambiguous or unsupported.', '<placement>')
        error.diagnostics = diagnostics + [error.diagnostic.as_dict()]
        raise error
    selected = spawns[spawn]
    ground = load_ground(installation, track, layout, encoding)
    diagnostics += vehicle.diagnostics + ground.diagnostics
    contacts = {}
    for name, offset in vehicle.wheel_offsets().items():
        # Probe from the spawn's footprint plane, ignoring wheel BASEY/radius:
        # the spawn is a placement marker, not a solved chassis CG equilibrium.
        origin = [selected.position[i] + selected.left[i] * offset[0]
                  + selected.forward[i] * offset[2] for i in range(3)]
        contacts[name] = {'origin': origin, 'hit': ground.probe(origin, 20.)}
        if contacts[name]['hit'] is None:
            diagnostics.append(Diagnostic('placement_no_support', f'{name}: no ground within 20 metres below the selected spawn footprint.', selected.model).as_dict())
    # Scene declarations are inspected by both independent loaders; report each once.
    diagnostics = list({tuple(sorted(d.items())): d for d in diagnostics}.values())
    return {'scope': 'geometric_vehicle_placement', 'car': car, 'track': track, 'layout': layout,
            'spawn': asdict(selected), 'compound': vehicle.compound, 'contacts': contacts,
            'all_wheels_supported': all(value['hit'] is not None for value in contacts.values()),
            'simulation_ready': False, 'diagnostics': diagnostics,
            'has_errors': any(d['severity'] == 'error' for d in diagnostics)}
