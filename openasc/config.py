"""Strict, source-aware readers for the initial supported INI/LUT subset."""
from bisect import bisect_right
from dataclasses import asdict, dataclass, field
import math
from pathlib import Path


@dataclass(frozen=True)
class Diagnostic:
    code: str
    message: str
    source: str
    line: int = 0
    severity: str = 'error'

    def as_dict(self):
        return asdict(self)


class ContentError(ValueError):
    def __init__(self, code, message, source='<text>', line=0):
        self.diagnostic = Diagnostic(code, message, source, line)
        super().__init__(f'{source}:{line}: {code}: {message}')


def uncomment(line, source, number):
    """Semicolons start comments outside double quotes; backslashes stay literal."""
    if line.lstrip().startswith(('#', '//')):
        return ''
    quoted = False
    for index, character in enumerate(line):
        if character == '"':
            quoted = not quoted
        elif character == ';' and not quoted:
            return line[:index].strip()
    if quoted:
        raise ContentError('unclosed_quote', 'Unclosed double-quoted value.', source, number)
    return line.strip()


def read_text(path, encoding='utf-8-sig'):
    try:
        return Path(path).read_text(encoding=encoding)
    except UnicodeError as error:
        raise ContentError('text_encoding', f'Cannot decode using {encoding}; select the correct encoding explicitly.', str(path)) from error


@dataclass
class IniSection:
    name: str
    line: int
    values: dict = field(default_factory=dict)
    lines: dict = field(default_factory=dict)


@dataclass
class IniDocument:
    source: str
    sections: dict = field(default_factory=dict)


def parse_ini(text, source='<text>'):
    """Normalize identifiers to uppercase, preserving values and source lines."""
    document = IniDocument(source)
    current = None
    for number, raw in enumerate(text.lstrip('\ufeff').splitlines(), 1):
        line = uncomment(raw, source, number)
        if not line:
            continue
        if line.startswith('['):
            if not line.endswith(']') or '[' in line[1:] or ']' in line[:-1]:
                raise ContentError('invalid_section', 'Expected a single [SECTION] header.', source, number)
            name = line[1:-1].strip().upper()
            if not name:
                raise ContentError('empty_section', 'Section name cannot be empty.', source, number)
            if name in document.sections:
                raise ContentError('duplicate_section', f'Duplicate section [{name}].', source, number)
            current = IniSection(name, number)
            document.sections[name] = current
            continue
        if current is None:
            raise ContentError('missing_section', 'A value must belong to a section.', source, number)
        if '=' not in line:
            raise ContentError('invalid_assignment', 'Expected KEY=VALUE.', source, number)
        key, value = line.split('=', 1)
        key, value = key.strip().upper(), value.strip()
        if not key or any(c.isspace() for c in key) or any(c in '[]"' for c in key):
            raise ContentError('invalid_key', 'Expected a nonempty key without whitespace, brackets, or quotes.', source, number)
        if key in current.values:
            raise ContentError('duplicate_key', f'Duplicate key {key} in [{current.name}].', source, number)
        if value.startswith('"'):
            if not value.endswith('"') or '"' in value[1:-1]:
                raise ContentError('invalid_quote', 'Only a whole value may be double-quoted.', source, number)
            value = value[1:-1]
        elif '"' in value:
            raise ContentError('invalid_quote', 'Only a whole value may be double-quoted.', source, number)
        current.values[key] = value
        current.lines[key] = number
    return document


def read_ini(path, encoding='utf-8-sig'):
    return parse_ini(read_text(path, encoding), str(path))


def finite_number(value, source, line):
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ContentError('invalid_number', f'Expected a number, got {value!r}.', source, line) from error
    if not math.isfinite(result):
        raise ContentError('nonfinite_number', 'NaN and infinity are unsupported.', source, line)
    return result


@dataclass(frozen=True)
class LutTable:
    source: str
    points: tuple

    def sample(self, x):
        """Piecewise linear interpolation, clamped at both table endpoints."""
        x = finite_number(x, self.source, 0)
        if x <= self.points[0][0]:
            return self.points[0][1]
        if x >= self.points[-1][0]:
            return self.points[-1][1]
        upper = bisect_right(self.points, (x, math.inf))
        x0, y0 = self.points[upper - 1]
        x1, y1 = self.points[upper]
        factor = (x - x0) / (x1 - x0)
        value = (1 - factor) * y0 + factor * y1
        if not math.isfinite(value) or not math.isfinite(x1 - x0):
            raise ContentError('numeric_range', 'Interpolation exceeds the supported numeric range.', self.source)
        return value


def parse_lut(text, source='<text>'):
    points = []
    for number, raw in enumerate(text.lstrip('\ufeff').splitlines(), 1):
        line = uncomment(raw, source, number)
        if not line:
            continue
        fields = line.split('|')
        if len(fields) != 2:
            raise ContentError('invalid_lut_row', 'Expected exactly two numbers: x|y.', source, number)
        x, y = (finite_number(value.strip(), source, number) for value in fields)
        if points and x <= points[-1][0]:
            raise ContentError('unordered_lut', 'LUT x coordinates must be strictly increasing; duplicates are unsupported.', source, number)
        points.append((x, y))
    if not points:
        raise ContentError('empty_lut', 'A LUT must contain at least one point.', source)
    return LutTable(source, tuple(points))


def read_lut(path, encoding='utf-8-sig'):
    return parse_lut(read_text(path, encoding), str(path))
