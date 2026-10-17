"""Source-aware track surface definitions, independent of rendering and tyres."""
from dataclasses import asdict, dataclass
from pathlib import Path
import re

from .config import ContentError, Diagnostic, finite_number, parse_ini, read_text
from .scene import component, confined

NUMBERS = {'FRICTION', 'DAMPING', 'WAV_PITCH', 'DIRT_ADDITIVE', 'BLACK_FLAG_TIME',
           'SIN_HEIGHT', 'SIN_LENGTH', 'VIBRATION_GAIN', 'VIBRATION_LENGTH'}
FLAGS = {'IS_VALID_TRACK', 'IS_PITLANE'}
KNOWN = NUMBERS | FLAGS | {'KEY', 'WAV', 'FF_EFFECT'}


@dataclass(frozen=True)
class Surface:
    key: str
    friction: float
    damping: float
    valid_track: bool
    pitlane: bool
    source: str
    section: str
    line: int
    parameters: dict

    def as_dict(self):
        return asdict(self)


def parse_surfaces(text, source='<surfaces>'):
    """Read explicit coefficients/flags; retain other known fields as metadata."""
    document = parse_ini(text, source)
    surfaces, diagnostics = {}, []
    for section in document.sections.values():
        if not re.fullmatch(r'SURFACE_[0-9]+', section.name):
            diagnostics.append(Diagnostic('surface_section', f'[{section.name}] is not a SURFACE_n definition.', source, section.line, 'warning').as_dict())
            continue
        for key in ('KEY', 'FRICTION', 'DAMPING', 'IS_VALID_TRACK', 'IS_PITLANE'):
            if key not in section.values:
                raise ContentError('surface_field', f'[{section.name}] requires {key}.', source, section.line)
        key = section.values['KEY'].upper()
        if not re.fullmatch(r'[A-Z][A-Z0-9_-]*', key):
            raise ContentError('surface_key', 'Surface KEY requires an ASCII letter followed by letters, digits, underscores or hyphens.', source, section.lines['KEY'])
        if key in surfaces:
            raise ContentError('surface_duplicate', f'Duplicate surface KEY {key!r} in the same file.', source, section.lines['KEY'])
        values = dict(section.values)
        for field in sorted(section.values.keys() & NUMBERS):
            values[field] = finite_number(values[field], source, section.lines[field])
            if field in ('FRICTION', 'DAMPING') and values[field] < 0:
                raise ContentError('surface_range', f'{field} must be nonnegative.', source, section.lines[field])
        for field in FLAGS:
            if values[field] not in ('0', '1'):
                raise ContentError('surface_flag', f'{field} must be 0 or 1.', source, section.lines[field])
            values[field] = values[field] == '1'
        for field in sorted(section.values.keys() - KNOWN):
            diagnostics.append(Diagnostic('surface_field_unsupported', f'[{section.name}] {field} is retained without interpretation.', source, section.lines[field], 'warning').as_dict())
        surfaces[key] = Surface(key, values['FRICTION'], values['DAMPING'], values['IS_VALID_TRACK'],
                                values['IS_PITLANE'], source, section.name, section.line, values)
    return surfaces, diagnostics


def load_surfaces(installation, track, layout=None, encoding='utf-8-sig'):
    """System defaults followed by complete same-KEY definitions in the selected track."""
    installation = Path(installation).resolve()
    component(track, 'Track')
    if layout is not None:
        component(layout, 'Layout')
    content = confined(installation, f'content/tracks/{track}', '<surfaces>')
    relative = f'{layout}/data/surfaces.ini' if layout else 'data/surfaces.ini'
    paths = [confined(installation, 'system/data/surfaces.ini', '<surfaces>'),
             confined(content, relative, '<surfaces>')]
    surfaces, diagnostics = {}, []
    for path in paths:
        source = path.relative_to(installation).as_posix()
        if not path.is_file():
            diagnostics.append(Diagnostic('surface_file_missing', 'Surface definitions are absent at this selected path.', source, severity='warning').as_dict())
            continue
        parsed, warnings = parse_surfaces(read_text(path, encoding), source)
        surfaces.update(parsed)
        diagnostics.extend(warnings)
    if not surfaces:
        raise ContentError('no_surfaces', 'No explicit surface definitions could be loaded.', '<surfaces>')
    return surfaces, diagnostics


def mesh_surface(name, surfaces):
    """Conservative supported naming policy; ambiguous prefix matches are rejected."""
    match = re.match(r'^([0-9]+)(.*)$', name.upper())
    if not match or not match[1].strip('0'):
        return None
    matches = [surface for key, surface in surfaces.items() if match[2].startswith(key)]
    if len(matches) > 1:
        raise ContentError('surface_ambiguous', f'Mesh {name!r} matches multiple surface keys; it is omitted.', '<mesh>')
    if not matches:
        raise ContentError('surface_unmatched', f'Mesh {name!r} has no supported explicit surface key; it is omitted.', '<mesh>')
    return matches[0]
