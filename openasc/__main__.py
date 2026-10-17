"""Run content diagnostics with python -m openasc."""
import argparse
import json
import sys

from .config import ContentError, read_ini, read_lut
from .scene import inspect_scene


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
        else:
            result = inspect_scene(args.installation, args.track, args.layout, args.encoding)
        print(json.dumps(result, indent=2, ensure_ascii=True, allow_nan=False))
        return 2 if result.get('has_errors') else 0
    except ContentError as error:
        print(json.dumps({'diagnostics': [error.diagnostic.as_dict()]}), file=sys.stderr)
        return 2
    except (OSError, LookupError) as error:
        print(json.dumps({'diagnostics': [{'code': 'input_io', 'message': str(error), 'severity': 'error'}]}), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
