"""Compare native/Python KN5 contracts using original fixtures or local models.

No source content is copied or extracted. Agreement does not prove AC parity.
"""
import argparse
import json
import math
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--model', action='append', default=[], type=Path, help='Optional local model; never exported.')
    args = parser.parse_args()
    native = args.native.resolve(strict=True)
    report = fixtures(native)
    report['text_and_scene_contracts'] = text_and_scene_contracts(native)
    report['local_models'] = [{'source': path.name, **compare(native, path)} for path in args.model]
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
