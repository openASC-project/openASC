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
from openasc.config import ContentError
from kn5_fixtures import base, file_bytes, mesh


def compare(native, path):
    parsed = load_kn5(path)
    result = subprocess.run([str(native), 'inspect', str(path)], capture_output=True, text=True, check=True)
    report = json.loads(result.stdout)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', required=True, type=Path)
    parser.add_argument('--model', action='append', default=[], type=Path, help='Optional local model; never exported.')
    args = parser.parse_args()
    native = args.native.resolve(strict=True)
    report = fixtures(native)
    report['local_models'] = [{'source': path.name, **compare(native, path)} for path in args.model]
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
