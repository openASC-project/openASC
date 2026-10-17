"""Run content diagnostics with python -m openasc."""
import argparse
import json
import sys

from .config import ContentError, read_ini, read_lut
from .scene import inspect_scene
from .kn5 import load_kn5


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for command, help_text in [('ini', 'Inspect INI sections and string values'), ('lut', 'Inspect a numeric LUT')]:
        sub = commands.add_parser(command, help=help_text)
        sub.add_argument('file')
        sub.add_argument('--encoding', default='utf-8-sig')
        if command == 'lut':
            sub.add_argument('--sample', type=float, help='Interpolate at x, clamping to table endpoints')
    scene = commands.add_parser('scene', help='Inspect static track configuration and KN5 headers')
    scene.add_argument('installation')
    scene.add_argument('--track', required=True)
    scene.add_argument('--layout')
    scene.add_argument('--encoding', default='utf-8-sig')
    kn5 = commands.add_parser('kn5', help='Inspect supported KN5 static geometry without exporting assets')
    kn5.add_argument('file')
    ground = commands.add_parser('ground', help='Inspect physical surfaces and optionally probe the ground vertically')
    ground.add_argument('installation')
    ground.add_argument('--track', required=True)
    ground.add_argument('--layout')
    ground.add_argument('--encoding', default='utf-8-sig')
    ground.add_argument('--at', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    ground.add_argument('--distance', type=float, default=1000.)
    viewer = commands.add_parser('view', help='Start a local-only static geometry viewer')
    viewer.add_argument('installation')
    viewer.add_argument('--track', required=True)
    viewer.add_argument('--layout')
    viewer.add_argument('--encoding', default='utf-8-sig')
    viewer.add_argument('--port', type=int, default=4180)
    args = parser.parse_args(argv)
    try:
        if args.command == 'ini':
            document = read_ini(args.file, args.encoding)
            result = {'source': document.source, 'scope': 'ini_subset',
                      'sections': {name: section.values for name, section in document.sections.items()}}
        elif args.command == 'lut':
            table = read_lut(args.file, args.encoding)
            result = {'source': table.source, 'scope': 'lut_subset', 'points': table.points}
            if args.sample is not None:
                result['sample'] = {'x': args.sample, 'y': table.sample(args.sample), 'boundary': 'clamp'}
        elif args.command == 'kn5':
            result = load_kn5(args.file).summary()
        elif args.command == 'ground':
            from .ground import load_ground
            ground = load_ground(args.installation, args.track, args.layout, args.encoding)
            result = ground.summary()
            if args.at is not None:
                result['probe'] = {'origin': args.at, 'max_distance': args.distance,
                                   'hit': ground.probe(args.at, args.distance)}
        elif args.command == 'view':
            from .viewer import serve_preview
            if not 0 <= args.port <= 65535:
                parser.error('--port must be between 0 and 65535.')
            return serve_preview(args.installation, args.track, args.layout, args.encoding, args.port)
        else:
            result = inspect_scene(args.installation, args.track, args.layout, args.encoding)
        print(json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False))
        return 2 if result.get('has_errors') else 0
    except ContentError as error:
        print(json.dumps({'diagnostics': getattr(error, 'diagnostics', [error.diagnostic.as_dict()])}), file=sys.stderr)
        return 2
    except (OSError, LookupError) as error:
        print(json.dumps({'diagnostics': [{'code': 'input_io', 'message': str(error), 'severity': 'error'}]}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
