"""Resolve static track model declarations without decoding geometry yet."""
from pathlib import Path, PureWindowsPath
import re
import struct

from .config import ContentError, Diagnostic, finite_number, parse_ini, read_text


def component(value, description):
    if not value or value in ('.', '..') or any(c in value for c in '/\\:'):
        raise ContentError('invalid_identifier', f'{description} must be a single directory identifier.')
    return value


def confined(root, relative, source, line=0):
    portable = PureWindowsPath(relative)
    if portable.is_absolute() or portable.drive or relative.startswith(('/', '\\')):
        raise ContentError('resource_path', 'An absolute resource path is unsupported.', source, line)
    path = (root / relative.replace('\\', '/')).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ContentError('resource_escape', 'Resource path leaves the selected content root.', source, line)
    return path


def kn5_header(path, source, line):
    with path.open('rb') as stream:
        header = stream.read(14)
    if len(header) < 10 or header[:6] != b'sc6969':
        raise ContentError('kn5_header', 'Missing or truncated KN5 signature/version.', source, line)
    version, = struct.unpack('<I', header[6:10])
    if version not in (5, 6):
        raise ContentError('kn5_version', f'KN5 version {version} is unsupported by this header inspector.', source, line)
    if version == 6 and len(header) < 14:
        raise ContentError('kn5_header', 'Truncated KN5 v6 header.', source, line)
    return {'version': version, 'scope': 'header_only', 'size_bytes': path.stat().st_size}


def vector(section, key, source):
    if key not in section.values:
        raise ContentError('missing_transform', f'[{section.name}] requires explicit {key}.', source, section.line)
    fields = section.values[key].split(',')
    if len(fields) != 3:
        raise ContentError('invalid_vector', f'{key} requires exactly three comma-separated numbers.', source, section.lines[key])
    return [finite_number(value, source, section.lines[key]) for value in fields]


def inspect_scene(installation, track, layout=None, encoding='utf-8-sig'):
    installation = Path(installation).resolve()
    track = component(track, 'Track')
    if layout is not None:
        layout = component(layout, 'Layout')
    content = confined(installation, f'content/tracks/{track}', '<scene>')
    if not content.is_dir():
        raise ContentError('missing_track', f'Track directory {track!r} does not exist.', '<scene>')
    name = f'models_{layout}.ini' if layout is not None else 'models.ini'
    config_path = confined(content, name, name)
    if not config_path.is_file():
        choices = ', '.join(p.stem.removeprefix('models_') for p in sorted(content.glob('models_*.ini')))
        message = f'{name} is missing.'
        if choices:
            message += f' Select --layout from: {choices}.'
        raise ContentError('missing_model_config', message, name)
    document = parse_ini(read_text(config_path, encoding), name)
    models, diagnostics = [], []
    for section in document.sections.values():
        if not re.fullmatch(r'MODEL_\d+', section.name):
            diagnostics.append(Diagnostic('unsupported_section', f'[{section.name}] is not loaded; only static MODEL_n declarations are supported.', name, section.line, 'warning').as_dict())
            continue
        try:
            file_line = section.lines.get('FILE', section.line)
            relative = section.values.get('FILE', '')
            if not relative or Path(relative).suffix.lower() != '.kn5':
                raise ContentError('model_file', f'[{section.name}] requires FILE pointing to a KN5 file.', name, file_line)
            path = confined(content, relative, name, file_line)
            position = vector(section, 'POSITION', name)
            rotation = vector(section, 'ROTATION', name)
            if not path.is_file():
                raise ContentError('missing_resource', f'Model file {relative!r} is missing.', name, file_line)
            header = kn5_header(path, name, file_line)
            models.append({'section': section.name, 'file': path.relative_to(content).as_posix(),
                           'position': position, 'rotation': rotation, 'kn5': header})
            for key in section.values.keys() - {'FILE', 'POSITION', 'ROTATION'}:
                diagnostics.append(Diagnostic('unsupported_key', f'[{section.name}] {key} is not interpreted.', name, section.lines[key], 'warning').as_dict())
        except ContentError as error:
            diagnostics.append(error.diagnostic.as_dict())
        except OSError:
            diagnostics.append(Diagnostic('resource_io', f'Cannot read the model in [{section.name}].', name, section.line).as_dict())
    if not models:
        diagnostics.append(Diagnostic('no_models', 'No static model passed configuration and header inspection.', name).as_dict())
    diagnostics.sort(key=lambda item: (item['line'], item['code'], item['message']))
    return {'schema_version': 1, 'scope': 'configuration_and_header_only', 'track': track,
            'layout': layout, 'config': name, 'geometry_loaded': False, 'models': models,
            'diagnostics': diagnostics,
            'has_errors': any(item['severity'] == 'error' for item in diagnostics)}
