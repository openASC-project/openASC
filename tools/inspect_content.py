"""Inventory a user's local AC content. Does not parse or copy game assets."""
import argparse
import json
from pathlib import Path


def inventory(root: Path) -> dict:
    root = root.resolve()
    content = root / 'content'
    if not all((content / kind).is_dir() for kind in ('cars', 'tracks')):
        raise ValueError('Expected an Assetto Corsa root with content/cars and content/tracks.')
    report = {'schema_version': 1, 'scope': 'file_presence_only', 'cars': [], 'tracks': []}
    for kind in ('cars', 'tracks'):
        for item in sorted((content / kind).iterdir(), key=lambda p: p.name):
            if not item.is_dir() or item.is_symlink() or not item.resolve().is_relative_to(root):
                continue
            # No recursion into models, skins or textures; report only folder-local evidence.
            entry = {
                'id': item.name,
                'path': item.relative_to(root).as_posix(),
                'kn5_present': any(p.is_file() for p in item.glob('*.kn5')),
            }
            if kind == 'cars':
                entry.update(ui_metadata_present=(item / 'ui/ui_car.json').is_file(),
                             unpacked_data_present=(item / 'data').is_dir(),
                             packed_data_present=(item / 'data.acd').is_file())
            else:
                entry.update(ui_metadata_present=(item / 'ui/ui_track.json').is_file(),
                             models_config_present=any(p.is_file() for p in item.glob('models*.ini')))
            report[kind].append(entry)
    report['counts'] = {kind: len(report[kind]) for kind in ('cars', 'tracks')}
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path, help='Local Assetto Corsa installation directory')
    parser.add_argument('--output', type=Path, help='Write JSON to this file instead of stdout')
    args = parser.parse_args()
    try:
        result = json.dumps(inventory(args.root), ensure_ascii=False, indent=2) + '\n'
        if args.output:
            target = args.output.resolve()
            if target.is_relative_to(args.root.resolve()):
                raise ValueError('Choose an output path outside the game installation.')
            if target.exists():
                raise ValueError('Output already exists; choose a new filename.')
            target.parent.mkdir(parents=True, exist_ok=True)
            with target.open('x', encoding='utf-8') as stream:
                stream.write(result)
        else:
            print(result, end='')
    except (ValueError, OSError) as error:
        parser.exit(2, f'Error: {error}\n')


if __name__ == '__main__':
    main()
