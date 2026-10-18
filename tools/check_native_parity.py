"""Compare native/Python KN5 contracts using original fixtures or local models.

No source content is copied or extracted. Agreement does not prove AC parity.
"""
import argparse
import json
import math
import os
import random
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tests'))
from openasc.kn5 import IDENTITY, load_kn5
from openasc.config import ContentError, parse_ini, parse_lut
from openasc.scene import inspect_scene
from openasc.vehicle import CarData, load_vehicle
from openasc.ground import load_ground
from openasc.placement import inspect_spawns, inspect_placement
from vehicle_fixtures import FILES, archive_bytes, install_car, suspension_source
from kn5_fixtures import base, file_bytes, mesh


def compare(native, path):
    parsed = load_kn5(path)
    result = subprocess.run([str(native), 'inspect', str(path)], capture_output=True, text=True, check=True)
    report = json.loads(result.stdout)
    material_result = subprocess.run([str(native), 'materials', str(path)], capture_output=True, text=True, check=True)
    materials = json.loads(material_result.stdout)['materials']
    for a, b in zip(parsed.materials, materials, strict=True):
        if (a['name'], a['shader'], a['textures']) != (b['name'], b['shader'], b['mappings']):
            raise AssertionError(f'{path.name}: different material mappings')
        if a['properties'].keys() != b['properties'].keys():
            raise AssertionError(f'{path.name}: different shader property names')
        for name, values in a['properties'].items():
            if len(values) != len(b['properties'][name]) or any(not math.isclose(x, y, rel_tol=1e-6, abs_tol=1e-6) for x, y in zip(values, b['properties'][name])):
                raise AssertionError(f'{path.name}: different property {name}')
    expected = {'version': parsed.version, 'textures': len(parsed.textures), 'materials': len(parsed.materials),
                'nodes': parsed.node_count, 'static_meshes': len(parsed.meshes),
                'skinned_meshes': sum(n.kind == 3 for n in parsed.node_transforms)}
    draws = [m for m in parsed.meshes if m.active and m.visible and m.renderable and m.bounds and m.indices and m.lod[0] <= 0]
    expected.update(drawn_meshes=len(draws), drawn_triangles=sum(len(m.indices) // 6 for m in draws))
    if expected['skinned_meshes'] == 0:
        expected['structural_triangles'] = sum(len(m.indices) // 6 for m in parsed.meshes)
    for key, value in expected.items():
        if report[key] != value:
            raise AssertionError(f'{path.name}: {key}: native {report[key]}, Python {value}')
    for key, meshes in [('bounds', draws), ('road_focus_bounds', [m for m in parsed.meshes if m.active and m.bounds and m.indices and m.lod[0] <= 0 and 'ROAD' in m.name.upper()])]:
        expected_bounds = [[min(m.bounds[0][a] for m in meshes) for a in range(3)],
                           [max(m.bounds[1][a] for m in meshes) for a in range(3)]] if meshes else None
        if expected_bounds is None:
            if report[key] is not None:
                raise AssertionError(f'{path.name}: unexpected {key}')
        else:
            if report[key] is None or any(not math.isclose(x, y, rel_tol=1e-6, abs_tol=1e-5)
                                         for native_row, python_row in zip(report[key], expected_bounds)
                                         for x, y in zip(native_row, python_row)):
                raise AssertionError(f'{path.name}: different {key}')
    return expected


def fixtures(native):
    translated = list(IDENTITY)
    translated[12:15] = [2., 3., -4.]
    rotated = (0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1.)
    cases = {
        'v5': file_bytes(version=5), 'v6': file_bytes(),
        'nested': file_bytes(base([base([mesh()], matrix=rotated)], matrix=translated)),
        'inactive': file_bytes(base([mesh()], active=False)),
        'nonrenderable': file_bytes(base([mesh(renderable=False)])),
        'hidden-road': file_bytes(base([mesh(name='1ROAD', renderable=False), mesh()])),
        'skinned': file_bytes(base([mesh(skinned=True), mesh()])),
        'multiple': file_bytes(base([mesh(), mesh()])),
    }
    rejected = {'truncated': file_bytes()[:-1], 'trailing': file_bytes() + b'X',
                'index': file_bytes(base([mesh(indices=(0, 3, 1))])),
                'nonfinite': file_bytes(base([mesh(points=((math.nan, 0., 0.), (1., 0., 0.), (0., 0., 1.)))]))}
    with tempfile.TemporaryDirectory(prefix='openasc-native-contracts-') as directory:
        path = Path(directory) / 'original.kn5'
        for name, data in cases.items():
            path.write_bytes(data)
            compare(native, path)
        for name, data in rejected.items():
            path.write_bytes(data)
            try:
                load_kn5(path)
            except ContentError as error:
                code = error.diagnostic.code
            else:
                raise AssertionError(f'Python accepted invalid fixture {name}')
            result = subprocess.run([str(native), 'inspect', str(path)], capture_output=True, text=True)
            if result.returncode == 0 or code not in result.stderr:
                raise AssertionError(f'Native did not reject {name} with {code}: {result.stderr}')
    return {'accepted_original_fixtures': len(cases), 'rejected_original_fixtures': len(rejected)}


def text_and_scene_contracts(native):
    """Original config/scene fixtures; agreement is not original-game validation."""
    with tempfile.TemporaryDirectory(prefix='openasc-native-text-') as directory:
        root = Path(directory)
        ini = root / 'original.ini'
        text = '\ufeff# original fixture\n[model_7]\nfile="folder\\a;b.kn5" ; tail\nempty=\nvalue=a=b\n[second]\nnumber=2'
        ini.write_text(text, encoding='utf-8')
        result = subprocess.run([str(native), 'ini', str(ini)], capture_output=True, text=True, check=True)
        report = json.loads(result.stdout)
        parsed = parse_ini(text, str(ini))
        expected = [{'name': s.name, 'line': s.line,
                     'values': {key: {'text': value, 'line': s.lines[key]} for key, value in s.values.items()}}
                    for s in parsed.sections.values()]
        if report['sections'] != expected:
            raise AssertionError('Native/Python INI values/order/source lines differ')
        lut = root / 'original.lut'
        text = '# original fixture\n0|10\n10|30\n30|-10'
        lut.write_text(text, encoding='utf-8')
        table = parse_lut(text)
        samples = [-5, 0, 5, 10, 20, 30, 99]
        for x in samples:
            result = subprocess.run([str(native), 'lut', str(lut), '--sample', str(x)], capture_output=True, text=True, check=True)
            if json.loads(result.stdout)['sample']['y'] != table.sample(x):
                raise AssertionError(f'Native/Python LUT sample differs at {x}')
        rejected = [('ini', '[A]\nx=1\nX=2'), ('ini', '[A]\nx="unfinished'),
                    ('lut', '1|2\n0|3'), ('lut', '0|nan'), ('lut', '')]
        for command, text in rejected:
            path = ini if command == 'ini' else lut
            path.write_text(text, encoding='utf-8')
            try:
                (parse_ini if command == 'ini' else parse_lut)(text)
            except ContentError as error:
                code = error.diagnostic.code
            else:
                raise AssertionError('Python accepted rejected config fixture')
            result = subprocess.run([str(native), command, str(path)], capture_output=True, text=True)
            if result.returncode == 0 or code not in result.stderr:
                raise AssertionError(f'Native did not reject config with {code}: {result.stderr}')
        content = root / 'content' / 'tracks' / 'original'
        content.mkdir(parents=True)
        (content / 'a.kn5').write_bytes(file_bytes(base([mesh()])))
        (content / 'b.kn5').write_bytes(file_bytes(base([mesh()])))
        config = content / 'models_layout.ini'
        config.write_text('[MODEL_8]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n'
                          '[MODEL_2]\nFILE=b.kn5\nPOSITION=10,2,-3\nROTATION=0,0,0\n'
                          '[DYNAMIC_OBJECT_0]\nFILE=a.kn5\n', encoding='utf-8')
        python = inspect_scene(root, 'original', 'layout')
        result = subprocess.run([str(native), 'inspect', str(config)], capture_output=True, text=True, check=True)
        report = json.loads(result.stdout)
        for a, b in zip(python['models'], report['manifest']['models'], strict=True):
            for field in ['section', 'file', 'position', 'rotation']:
                if a[field] != b[field]:
                    raise AssertionError(f'Native/Python scene {field} differs')
            if a['kn5']['version'] != b['version']:
                raise AssertionError('Native/Python scene model version differs')
        if report['scene']['drawn_triangles'] != 2 or not report['manifest']['diagnostics']:
            raise AssertionError('Native scene composition or dynamic omission diagnostics missing')
    return {'ini_documents': 1, 'lut_samples': len(samples), 'rejected_configs': len(rejected), 'composed_models': 2}


def compare_vehicle(native, path, mode='auto', compound=None):
    expected = load_vehicle(path.parents[2], path.name, mode=mode, compound=compound).summary()
    args = [str(native), 'car', str(path), '--data', mode]
    if compound is not None:
        args += ['--compound', str(compound)]
    result = subprocess.run(args, capture_output=True, text=True, check=True)
    actual = json.loads(result.stdout)
    expected.pop('diagnostics')
    actual.pop('diagnostics')
    def compare_values(a, b, location):
        if isinstance(a, dict):
            if a.keys() != b.keys():
                raise AssertionError(f'{location}: different fields')
            for key in a:
                compare_values(a[key], b[key], location + '.' + key)
        elif isinstance(a, (list, tuple)):
            if len(a) != len(b):
                raise AssertionError(f'{location}: different lengths')
            for i, (x, y) in enumerate(zip(a, b)):
                compare_values(x, y, f'{location}[{i}]')
        elif isinstance(a, (float, int)) and not isinstance(a, bool):
            if not math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12):
                raise AssertionError(f'{location}: {a} != {b}')
        elif a != b:
            raise AssertionError(f'{location}: {a!r} != {b!r}')
    compare_values(expected, actual, path.name)
    return {'car': actual['car'], 'data_source': actual['data_source'], 'mass_kg': actual['mass_kg'],
            'wheelbase_m': actual['wheelbase_m'], 'torque_points': len(actual['engine']['torque_lut']['points'])}


def vehicle_contracts(native):
    with tempfile.TemporaryDirectory(prefix='openasc-native-car-') as directory:
        root = Path(directory)
        if hasattr(os, 'mkfifo'):
            fifo = root / 'not-an-archive.acd'
            os.mkfifo(fifo)
            result = subprocess.run([str(native), 'acd', str(fifo), 'fixture'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0 or 'acd_io' not in result.stderr:
                raise AssertionError('Native archive reader accepted a nonregular input')
        unpacked = install_car(root)
        packed = install_car(root, 'packed_fixture', packed=True)
        reports = [compare_vehicle(native, unpacked), compare_vehicle(native, packed),
                   compare_vehicle(native, unpacked, compound=1)]
        (unpacked / 'data.acd').write_bytes(archive_bytes(FILES.items()))
        (unpacked / 'data' / 'engine.ini').unlink()
        result = subprocess.run([str(native), 'car', str(unpacked)], capture_output=True, text=True)
        if result.returncode == 0 or 'car_data_missing' not in result.stderr:
            raise AssertionError('Native auto mode silently mixed packed and unpacked files')
        reports.append(compare_vehicle(native, unpacked, mode='packed'))
        for text, code in [(FILES['car.ini'].replace('TOTALMASS=900', 'TOTALMASS=nan'), 'nonfinite_number'),
                           (FILES['car.ini'].replace('VERSION=1', 'VERSION=999'), 'vehicle_version')]:
            (unpacked / 'data' / 'engine.ini').write_text(FILES['engine.ini'], encoding='utf-8')
            (unpacked / 'data' / 'car.ini').write_text(text, encoding='utf-8')
            result = subprocess.run([str(native), 'car', str(unpacked)], capture_output=True, text=True)
            if result.returncode == 0 or code not in result.stderr:
                raise AssertionError(f'Native did not reject car fixture with {code}')
    return {'accepted_configurations': len(reports), 'whole_source_rejection': 1, 'invalid_fields': 2}


def compare_ground(native, installation, track, layout=None, samples=1000):
    """Compare geometric support only; source assets stay in the installation."""
    installation = installation.resolve(strict=True)
    expected = load_ground(installation, track, layout)
    content = installation / 'content/tracks' / track
    config = content / (f'models_{layout}.ini' if layout else 'models.ini')
    selected = content / (f'{layout}/data/surfaces.ini' if layout else 'data/surfaces.ini')
    command = [str(native), 'ground', str(config), str(selected), '--defaults',
               str(installation / 'system/data/surfaces.ini'), '--distance', '1']
    rng = random.Random(6743)
    indices = rng.sample(range(len(expected.mesh_ids)), min(samples, len(expected.mesh_ids)))
    origins = []
    for index in indices:
        tri = expected.triangles[index * 9:index * 9 + 9]
        x, y, z = [sum(tri[axis::3]) / 3 for axis in range(3)]
        origins.append((x, y + .1, z))
    origins += [(0., -9_999_999., 0.), (9_999_999., 0., 9_999_999.)]
    def normalize(value):
        if isinstance(value, dict):
            result = {key: normalize(v) for key, v in value.items()}
            if 'source' in result and Path(result['source']).is_absolute():
                result['source'] = Path(result['source']).relative_to(installation).as_posix()
            return result
        if isinstance(value, (list, tuple)):
            return [normalize(v) for v in value]
        return value
    def same(a, b):
        if isinstance(a, dict):
            return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
        if isinstance(a, (list, tuple)):
            return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
        if isinstance(a, float):
            return math.isclose(a, b, abs_tol=1e-7, rel_tol=1e-10)
        return a == b
    hits = 0
    for start in range(0, len(origins), 128):
        batch = origins[start:start + 128]
        args = command + [field for origin in batch for field in ('--probe', *map(str, origin))]
        result = subprocess.run(args, capture_output=True, text=True, check=True, timeout=90)
        actual = json.loads(result.stdout)
        for key in ('triangles', 'input_triangles', 'skipped_vertical_or_degenerate', 'bvh_ready', 'meshes'):
            if not same(normalize(expected.summary()[key]), normalize(actual['summary'][key])):
                raise AssertionError(f'{track}: ground summary {key} differs')
        if actual['summary']['simulation_ready'] is not False:
            raise AssertionError('Ground preparation must not claim simulation readiness')
        for origin, probe in zip(batch, actual['probes'], strict=True):
            wanted = expected.probe(origin, 1.)
            if not same(normalize(wanted), normalize(probe['hit'])):
                raise AssertionError(f'{track}: support differs at {origin}: {wanted} != {probe["hit"]}')
            hits += wanted is not None
    return {'track': track, 'layout': layout, 'triangles': len(expected.mesh_ids),
            'meshes': len(expected.meshes), 'probes': len(origins), 'hits': hits,
            'comparison': 'static geometry and surface metadata; not original vehicle physics'}


def ground_contracts(native):
    with tempfile.TemporaryDirectory(prefix='openasc-native-ground-') as directory:
        root = Path(directory)
        content = root / 'content/tracks/original'
        (content / 'data').mkdir(parents=True)
        (root / 'system/data').mkdir(parents=True)
        definition = '[SURFACE_0]\nKEY=ROAD\nFRICTION=.83\nDAMPING=.02\nIS_VALID_TRACK=1\nIS_PITLANE=0\n'
        (root / 'system/data/surfaces.ini').write_text(definition, encoding='utf-8')
        (content / 'data/surfaces.ini').write_text(definition.replace('.83', '.71'), encoding='utf-8')
        node = base([mesh(name='1ROAD', renderable=False, points=((0, 0, 0), (2, 2, 0), (0, 0, 2))),
                     mesh(name='1WALL'), mesh(name='1UNKNOWN'), mesh(name='visual')])
        (content / 'a.kn5').write_bytes(file_bytes(node, version=5))
        (content / 'b.kn5').write_bytes(file_bytes(node, version=6))
        (content / 'models.ini').write_text('[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n'
                                           '[MODEL_1]\nFILE=b.kn5\nPOSITION=10,3,20\nROTATION=0,0,0\n', encoding='utf-8')
        before = set(root.rglob('*'))
        report = compare_ground(native, root, 'original')
        if set(root.rglob('*')) != before:
            raise AssertionError('Ground preparation wrote into source content')
        return report


def compare_placement(native, car, track):
    installation = track.parents[2]
    if car.parents[2] != installation:
        raise ValueError('Car and track must belong to the same installation for this comparison')
    expected_spawns = inspect_spawns(installation, track.name)
    command = [str(native), 'spawns', str(track / 'models.ini')]
    result = subprocess.run(command, capture_output=True, text=True, timeout=90)
    actual_spawns = json.loads(result.stdout)
    if result.returncode != int(actual_spawns['has_errors']):
        raise AssertionError('Spawn process status does not reflect report errors')
    def normalize(value):
        if isinstance(value, dict):
            result = {k: normalize(v) for k, v in value.items()}
            if 'source' in result and Path(result['source']).is_absolute():
                result['source'] = Path(result['source']).relative_to(installation).as_posix()
            return result
        if isinstance(value, (tuple, list)):
            return [normalize(v) for v in value]
        return value
    def compare_values(a, b, location):
        if isinstance(a, dict):
            if a.keys() != b.keys():
                raise AssertionError(f'{location}: keys differ')
            for key in a:
                compare_values(a[key], b[key], location + '.' + key)
        elif isinstance(a, (tuple, list)):
            if len(a) != len(b):
                raise AssertionError(f'{location}: lengths differ')
            for i, (x, y) in enumerate(zip(a, b)):
                compare_values(x, y, f'{location}[{i}]')
        elif isinstance(a, float):
            if not math.isclose(a, b, abs_tol=1e-7, rel_tol=1e-10):
                raise AssertionError(f'{location}: {a} != {b}')
        elif a != b:
            raise AssertionError(f'{location}: {a!r} != {b!r}')
    compare_values(expected_spawns['spawns'], actual_spawns['spawns'], 'spawns')
    compare_values(expected_spawns['has_errors'], actual_spawns['has_errors'], 'spawns.has_errors')
    expected = inspect_placement(installation, car.name, track.name)
    command = [str(native), 'place', str(car), str(track / 'models.ini'),
               str(track / 'data/surfaces.ini'), '--defaults', str(installation / 'system/data/surfaces.ini')]
    result = subprocess.run(command, capture_output=True, text=True, timeout=90)
    actual = json.loads(result.stdout)
    if result.returncode != int(actual['has_errors']):
        raise AssertionError('Placement process status does not reflect report errors')
    for field in ('scope', 'track', 'layout', 'diagnostics'):
        expected.pop(field)
    for field in ('scope', 'config', 'diagnostics'):
        actual.pop(field)
    compare_values(normalize(expected), normalize(actual), 'placement')
    return {'car': car.name, 'track': track.name, 'usable_spawns': len(actual_spawns['spawns']),
            'all_wheels_supported': actual['all_wheels_supported'], 'has_errors': actual['has_errors'],
            'simulation_ready': actual['simulation_ready']}


def placement_contracts(native):
    with tempfile.TemporaryDirectory(prefix='openasc-native-placement-') as directory:
        root = Path(directory)
        car = install_car(root)
        packed = install_car(root, 'packed_fixture', packed=True)
        track = root / 'content/tracks/fixture'
        (track / 'data').mkdir(parents=True)
        (root / 'system/data').mkdir(parents=True)
        definition = '[SURFACE_0]\nKEY=ROAD\nFRICTION=.9\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n'
        (root / 'system/data/surfaces.ini').write_text(definition, encoding='utf-8')
        (track / 'data/surfaces.ini').write_text(definition, encoding='utf-8')
        (track / 'models.ini').write_text('[MODEL_0]\nFILE=original.kn5\nPOSITION=5,3,0\nROTATION=0,0,0\n', encoding='utf-8')
        road = mesh(name='01ROAD', renderable=False,
                    points=((-50, 0, -50), (50, 0, -50), (-50, 0, 50), (50, 0, 50)), indices=(0, 2, 1, 1, 2, 3))
        pose = (0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 10., 2., 0., 1.)
        marker = base([mesh(name='AC_PIT_0', renderable=False)], matrix=pose, name='AC_PIT_0')
        path = track / 'original.kn5'
        path.write_bytes(file_bytes(base([road, marker])))
        before = set(root.rglob('*'))
        reports = [compare_placement(native, car, track), compare_placement(native, packed, track)]
        duplicate = base([], name='AC_START_3')
        path.write_bytes(file_bytes(base([road, marker, duplicate, duplicate])))
        reports.append(compare_placement(native, car, track))
        outside = list(pose)
        outside[12] = 100
        path.write_bytes(file_bytes(base([road, base([], matrix=outside, name='AC_PIT_0')])))
        reports.append(compare_placement(native, car, track))
        path.write_bytes(file_bytes(base([road, marker, marker])))
        rejected = subprocess.run([str(native), 'place', str(car), str(track / 'models.ini'),
                                   str(track / 'data/surfaces.ini')], capture_output=True, text=True, timeout=10)
        if rejected.returncode == 0 or 'spawn_missing' not in rejected.stderr:
            raise AssertionError('Ambiguous selected spawn must not be arbitrarily selected')
        if set(root.rglob('*')) != before:
            raise AssertionError('Placement preparation wrote into source content')
        return {'compared_reports': reports, 'ambiguous_selection_rejected': True}


def compare_suspension(native, car):
    data = CarData(car.parents[2], car.name)
    document = data.ini('suspensions.ini')
    actual = json.loads(subprocess.run([str(native), 'suspension', str(car)], capture_output=True, text=True, check=True, timeout=20).stdout)
    if actual['header_version'] != int(document.sections['HEADER'].values['VERSION']):
        raise AssertionError('Suspension source header version differs')
    if actual['data_source'] != data.mode or actual['simulation_ready'] is not False or actual['has_errors'] is not False:
        raise AssertionError('Suspension source selection/readiness differs')
    expected_keys = {'HEADER.VERSION', 'BASIC.WHEELBASE', 'BASIC.CG_LOCATION'}
    def scalar(section, key, value):
        expected_keys.add(f'{section}.{key}')
        if not math.isclose(float(document.sections[section].values[key]), value, abs_tol=1e-12, rel_tol=1e-12):
            raise AssertionError(f'Suspension value differs: {section}.{key}')
    scalar('BASIC', 'WHEELBASE', actual['wheelbase_m'])
    scalar('BASIC', 'CG_LOCATION', actual['front_weight_fraction'])
    numeric = {'SPRING_RATE', 'PROGRESSIVE_SPRING_RATE', 'BUMP_STOP_RATE', 'BUMPSTOP_UP', 'BUMPSTOP_DN', 'PACKER_RANGE',
               'DAMP_BUMP', 'DAMP_FAST_BUMP', 'DAMP_FAST_BUMPTHRESHOLD', 'DAMP_REBOUND', 'DAMP_FAST_REBOUND',
               'DAMP_FAST_REBOUNDTHRESHOLD', 'TOE_OUT', 'STATIC_CAMBER'}
    for section in ('FRONT', 'REAR'):
        axle = actual['axles'][section.lower()]
        kind = document.sections[section].values['TYPE']
        if axle['type'] != kind:
            raise AssertionError('Suspension kind differs')
        expected_keys.add(f'{section}.TYPE')
        for key, field in [('TRACK', 'track_m'), ('BASEY', 'base_y_m'), ('ROD_LENGTH', 'rod_length_m'), ('HUB_MASS', 'hub_mass_kg')]:
            scalar(section, key, axle[field])
        names = {'WBCAR_BOTTOM_FRONT', 'WBCAR_BOTTOM_REAR', 'WBTYRE_BOTTOM', 'WBCAR_STEER', 'WBTYRE_STEER'}
        names |= {'STRUT_CAR', 'STRUT_TYRE'} if kind == 'STRUT' else {'WBCAR_TOP_FRONT', 'WBCAR_TOP_REAR', 'WBTYRE_TOP'}
        if axle['hardpoints'].keys() != names or axle['source_parameters'].keys() != numeric:
            raise AssertionError('Suspension record fields differ')
        for name in names:
            expected_keys.add(f'{section}.{name}')
            values = [float(v) for v in document.sections[section].values[name].split(',')]
            if values != axle['hardpoints'][name]:
                raise AssertionError(f'Suspension hardpoint differs: {section}.{name}')
        for name in numeric:
            scalar(section, name, axle['source_parameters'][name])
    expected_arb = {name: float(document.sections['ARB'].values[name]) for name in ('FRONT', 'REAR')} if 'ARB' in document.sections else {}
    if actual['antiroll_source_parameters'] != expected_arb:
        raise AssertionError('Suspension ARB source values differ')
    expected_keys.update(f'ARB.{name}' for name in expected_arb)
    if actual['field_sources'].keys() != expected_keys:
        raise AssertionError('Suspension source-field coverage differs')
    for field, source in actual['field_sources'].items():
        section, key = field.split('.')
        if source != {'source': data.source('suspensions.ini'), 'line': document.sections[section].lines[key]}:
            raise AssertionError(f'Suspension provenance differs: {field}')
    return {'car': car.name, 'data_source': actual['data_source'], 'front_type': actual['axles']['front']['type'],
            'rear_type': actual['axles']['rear']['type'], 'validated_source_fields': len(expected_keys),
            'comparison': 'source configuration only; no original suspension solver or force law'}


def suspension_contracts(native):
    with tempfile.TemporaryDirectory(prefix='openasc-native-suspension-') as directory:
        root = Path(directory)
        files = {**FILES, 'suspensions.ini': suspension_source()}
        unpacked = install_car(root, files=files)
        packed = install_car(root, 'packed_fixture', packed=True, files=files)
        before = set(root.rglob('*'))
        reports = [compare_suspension(native, unpacked), compare_suspension(native, packed)]
        if set(root.rglob('*')) != before:
            raise AssertionError('Suspension inspection wrote into source content')
        return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--model', action='append', default=[], type=Path, help='Optional local model; never exported.')
    parser.add_argument('--car', action='append', default=[], type=Path, help='Optional local content/cars/CAR directory; configuration only.')
    parser.add_argument('--ground', action='append', default=[], type=Path, help='Optional local content/tracks/TRACK directory; static support only.')
    parser.add_argument('--placement', nargs=2, metavar=('CAR_DIRECTORY', 'TRACK_DIRECTORY'), type=Path, help='Optional local geometric placement comparison at AC_PIT_0.')
    parser.add_argument('--suspension', action='append', default=[], type=Path, help='Optional local DWB/STRUT suspension source-record comparison.')
    args = parser.parse_args()
    native = args.native.resolve(strict=True)
    report = fixtures(native)
    report['text_and_scene_contracts'] = text_and_scene_contracts(native)
    report['vehicle_contracts'] = vehicle_contracts(native)
    report['ground_contracts'] = ground_contracts(native)
    report['placement_contracts'] = placement_contracts(native)
    report['suspension_contracts'] = suspension_contracts(native)
    report['local_models'] = [{'source': path.name, **compare(native, path)} for path in args.model]
    report['local_cars'] = [compare_vehicle(native, path.resolve(strict=True)) for path in args.car]
    report['local_ground'] = [compare_ground(native, path.resolve(strict=True).parents[2], path.name) for path in args.ground]
    report['local_placement'] = compare_placement(native, *(path.resolve(strict=True) for path in args.placement)) if args.placement else None
    report['local_suspension'] = [compare_suspension(native, path.resolve(strict=True)) for path in args.suspension]
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
