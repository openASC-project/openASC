"""Read an explicit vehicle configuration subset; no original physics is executed."""
from dataclasses import asdict, dataclass
from itertools import islice
import os
from pathlib import Path

from .acd import MAX_DECODED, MAX_ENTRIES, MAX_ENTRY, data_name, read_archive
from .config import ContentError, Diagnostic, finite_number, parse_ini, parse_lut
from .scene import component, confined, kn5_header


class CarData:
    def __init__(self, installation, car, mode='auto', encoding='utf-8-sig'):
        component(car, 'Car')
        if mode not in ('auto', 'packed', 'unpacked'):
            raise ContentError('car_data_mode', 'Choose auto, packed or unpacked data.')
        self.root = Path(installation).resolve()
        self.car = car
        self.content = confined(self.root, f'content/cars/{car}', '<car>')
        self.directory = confined(self.content, 'data', '<car>')
        self.encoding, self.cache, self.archive = encoding, {}, None
        self.names, self.diagnostics = {}, []
        packed = confined(self.content, 'data.acd', '<car>')
        self.mode = ('unpacked' if self.directory.is_dir() else 'packed') if mode == 'auto' else mode
        if mode == 'auto' and self.directory.is_dir() and packed.is_file():
            self.diagnostics.append(Diagnostic('car_data_precedence', 'Both sources exist; the unpacked data directory is selected as a whole, without per-file archive fallback.', f'content/cars/{car}', severity='warning').as_dict())
        if self.mode == 'packed':
            self.archive = read_archive(packed, car, f'content/cars/{car}/data.acd')
        else:
            if not self.directory.is_dir():
                raise ContentError('car_data_missing', 'Selected unpacked data directory is absent.', f'content/cars/{car}/data')
            # scandir streams entries; Path.iterdir may eagerly materialize them.
            with os.scandir(self.directory) as scanner:
                entries = list(islice(scanner, MAX_ENTRIES + 1))
            if len(entries) > MAX_ENTRIES:
                raise ContentError('car_data_limit', 'Unpacked data exceeds 4096 directory entries.', f'content/cars/{car}/data')
            for file in sorted(entries, key=lambda entry: entry.name):
                if not file.is_file():
                    continue
                try:
                    name = data_name(file.name)
                except ContentError:
                    self.diagnostics.append(Diagnostic('car_data_name_ignored',
                        f'Ignored unpacked file {file.name!r} with a nonportable filename; it is not a selectable car-data resource.',
                        f'content/cars/{car}/data', severity='warning').as_dict())
                    continue
                if name in self.names:
                    raise ContentError('car_data_duplicate', f'Ambiguous case-insensitive file {file.name!r}.', f'content/cars/{car}/data')
                self.names[name] = file.name

    def source(self, name):
        return f'content/cars/{self.car}/data.acd::{name}' if self.mode == 'packed' else f'content/cars/{self.car}/data/{name}'

    def text(self, name):
        key = data_name(name)
        if key not in self.cache:
            if self.archive:
                data = self.archive.read(key)
            else:
                if key not in self.names:
                    raise ContentError('car_data_missing', f'Required car data file {name!r} is absent.', self.source(name))
                path = confined(self.directory, self.names[key], self.source(name))
                with path.open('rb') as stream:
                    data = stream.read(MAX_ENTRY + 1)
            if len(data) > MAX_ENTRY or sum(len(value) for value in self.cache.values()) + len(data) > MAX_DECODED:
                raise ContentError('car_data_limit', 'Car data exceeds the 8 MiB file or 16 MiB loaded-data budget.', self.source(name))
            self.cache[key] = data
        try:
            return self.cache[key].decode(self.encoding)
        except UnicodeError as error:
            raise ContentError('text_encoding', f'Cannot decode using {self.encoding}; select the correct encoding or check the original car folder identifier.', self.source(name)) from error

    def ini(self, name):
        return parse_ini(self.text(name), self.source(name))


class Fields:
    def __init__(self, document):
        self.document, self.used = document, set()

    def value(self, section, key):
        block = self.document.sections.get(section)
        if block is None or key not in block.values:
            raise ContentError('vehicle_field', f'[{section}] requires {key} for the supported vehicle subset.', self.document.source, block.line if block else 0)
        self.used.add((section, key))
        return block.values[key]

    def fail(self, section, key, message, code='vehicle_range'):
        raise ContentError(code, message, self.document.source, self.document.sections[section].lines[key])

    def number(self, section, key, minimum=None, maximum=None, positive=False):
        value = finite_number(self.value(section, key), self.document.source, self.document.sections[section].lines[key])
        if ((minimum is not None and value < minimum) or (maximum is not None and value > maximum)
                or (positive and value <= 0)):
            self.fail(section, key, f'[{section}] {key} is outside the supported range.')
        return value

    def integer(self, section, key, minimum, maximum):
        value = self.number(section, key, minimum, maximum)
        if not value.is_integer():
            self.fail(section, key, f'[{section}] {key} must be an integer.')
        return int(value)

    def vector(self, section, key, positive=False):
        fields = self.value(section, key).split(',')
        if len(fields) != 3:
            self.fail(section, key, f'[{section}] {key} requires three coordinates.', 'vehicle_vector')
        values = tuple(finite_number(value, self.document.source, self.document.sections[section].lines[key]) for value in fields)
        if positive and min(values) <= 0:
            self.fail(section, key, f'[{section}] {key} requires positive dimensions.')
        return values


@dataclass(frozen=True)
class Vehicle:
    car: str
    data_source: str
    mass_kg: float
    inertia_dimensions_m: tuple
    graphics_offset_m: tuple
    graphics_pitch_deg: float
    steer_lock_deg: float
    steer_ratio: float
    wheelbase_m: float
    front_weight_fraction: float
    axles: dict
    engine: dict
    drivetrain: dict
    brakes: dict
    compound: int
    model: dict
    field_sources: dict
    diagnostics: list

    def wheel_offsets(self):
        front_z = self.wheelbase_m * (1 - self.front_weight_fraction)
        rear_z = -self.wheelbase_m * self.front_weight_fraction
        return {f'{axle}_{side}': (sign * self.axles[axle]['track_m'] / 2,
                                  self.axles[axle]['base_y_m'], z)
                for axle, z in [('front', front_z), ('rear', rear_z)]
                for side, sign in [('left', 1), ('right', -1)]}

    def summary(self):
        result = asdict(self)
        result.update(scope='vehicle_configuration_subset', simulation_ready=False,
                      wheel_offsets_m=self.wheel_offsets(), has_errors=False)
        return result


def load_vehicle(installation, car, compound=None, mode='auto', encoding='utf-8-sig'):
    data = CarData(installation, car, mode, encoding)
    files = ('car.ini', 'suspensions.ini', 'tyres.ini', 'engine.ini', 'drivetrain.ini', 'brakes.ini', 'lods.ini')
    fields = {name: Fields(data.ini(name)) for name in files}
    versions = {'car.ini': {1}, 'suspensions.ini': {1, 2}, 'tyres.ini': {7, 10},
                'engine.ini': {1}, 'drivetrain.ini': {1, 2, 3}, 'brakes.ini': {1}}
    for name, supported in versions.items():
        f = fields[name]
        version = f.integer('HEADER', 'VERSION', 0, 10000)
        if version not in supported:
            f.fail('HEADER', 'VERSION', f'{name} version {version} is outside the supported configuration subset.', 'vehicle_version')
    car_fields, suspension, tyres, engine, drive, brake, lod = (fields[name] for name in files)
    mass = car_fields.number('BASIC', 'TOTALMASS', positive=True)
    inertia = car_fields.vector('BASIC', 'INERTIA', positive=True)
    offset = car_fields.vector('BASIC', 'GRAPHICS_OFFSET')
    pitch = car_fields.number('BASIC', 'GRAPHICS_PITCH_ROTATION')
    lock = car_fields.number('CONTROLS', 'STEER_LOCK', positive=True)
    ratio = car_fields.number('CONTROLS', 'STEER_RATIO', positive=True)
    wheelbase = suspension.number('BASIC', 'WHEELBASE', positive=True)
    share = suspension.number('BASIC', 'CG_LOCATION', 0, 1)
    if not 0 < share < 1:
        suspension.fail('BASIC', 'CG_LOCATION', 'Front weight fraction must be strictly between zero and one.')
    if compound is None:
        compound = tyres.integer('COMPOUND_DEFAULT', 'INDEX', 0, 99)
    elif not isinstance(compound, int) or isinstance(compound, bool) or not 0 <= compound <= 99:
        raise ContentError('vehicle_compound', 'Compound index must be an integer between 0 and 99.')
    axles = {}
    for axle in ('FRONT', 'REAR'):
        section = axle if compound == 0 else f'{axle}_{compound}'
        axles[axle.lower()] = {
            'type': suspension.value(axle, 'TYPE'),
            'track_m': suspension.number(axle, 'TRACK', positive=True),
            'base_y_m': suspension.number(axle, 'BASEY'),
            'tyre_name': tyres.value(section, 'NAME'),
            'tyre_radius_m': tyres.number(section, 'RADIUS', positive=True),
            'tyre_width_m': tyres.number(section, 'WIDTH', positive=True)}
    idle = engine.number('ENGINE_DATA', 'MINIMUM', minimum=0)
    limiter = engine.number('ENGINE_DATA', 'LIMITER', positive=True)
    if limiter <= idle:
        engine.fail('ENGINE_DATA', 'LIMITER', 'Limiter must exceed idle speed in this subset.')
    engine_inertia = engine.number('ENGINE_DATA', 'INERTIA', positive=True)
    curve_name = engine.value('HEADER', 'POWER_CURVE')
    curve = parse_lut(data.text(curve_name), data.source(curve_name))
    if len(curve.points) < 2 or curve.points[0][0] < 0 or curve.points[0][0] > idle or curve.points[-1][0] < limiter:
        raise ContentError('vehicle_power_domain', 'Power LUT requires at least two nonnegative RPM coordinates covering idle through limiter.', data.source(curve_name))
    traction = drive.value('TRACTION', 'TYPE').upper()
    if traction not in ('RWD', 'FWD', 'AWD'):
        drive.fail('TRACTION', 'TYPE', 'Unknown traction type.', 'vehicle_traction')
    count = drive.integer('GEARS', 'COUNT', 1, 10)
    gears = tuple(drive.number('GEARS', f'GEAR_{i}', positive=True) for i in range(1, count + 1))
    reverse = drive.number('GEARS', 'GEAR_R')
    if reverse >= 0:
        drive.fail('GEARS', 'GEAR_R', 'Reverse ratio must be negative.')
    final = drive.number('GEARS', 'FINAL', positive=True)
    brakes = {'max_torque_nm': brake.number('DATA', 'MAX_TORQUE', positive=True),
              'front_share': brake.number('DATA', 'FRONT_SHARE', 0, 1),
              'handbrake_torque_nm': brake.number('DATA', 'HANDBRAKE_TORQUE', minimum=0)}
    model_name = lod.value('LOD_0', 'FILE')
    path = confined(data.content, model_name, lod.document.source, lod.document.sections['LOD_0'].lines['FILE'])
    if path.suffix.lower() != '.kn5' or not path.is_file():
        lod.fail('LOD_0', 'FILE', 'LOD_0 requires an existing local KN5 model.', 'vehicle_model')
    model = {'file': path.relative_to(data.content).as_posix(), 'kn5': kn5_header(path, lod.document.source, lod.document.sections['LOD_0'].lines['FILE'])}
    diagnostics = list(data.diagnostics)
    sources = {}
    for name, f in fields.items():
        sources[name] = {f'{section}.{key}': {'source': f.document.source, 'line': f.document.sections[section].lines[key]}
                         for section, key in sorted(f.used)}
        unused = sum(len(section.values) for section in f.document.sections.values()) - len(f.used)
        if unused:
            diagnostics.append(Diagnostic('vehicle_fields_uninterpreted', f'{name}: {unused} other fields are not interpreted by this configuration subset.', f.document.source, severity='warning').as_dict())
    if any(section.startswith('TURBO_') for section in engine.document.sections):
        diagnostics.append(Diagnostic('vehicle_turbo_unimplemented', 'Turbo definitions are present; the base power curve alone does not describe boosted output.', engine.document.source, severity='warning').as_dict())
    diagnostics.append(Diagnostic('vehicle_configuration_scope', 'Configuration inspection only. Inertia dimensions are not inertia tensor values. Tyre/suspension dynamics, fuel, coast torque, aero, electronics, transmission behavior, audio and FFB are not simulated; model validation is header-only.', f'content/cars/{car}', severity='warning').as_dict())
    return Vehicle(car, data.mode, mass, inertia, offset, pitch, lock, ratio, wheelbase, share, axles,
                   {'idle_rpm': idle, 'limiter_rpm': limiter, 'inertia_kg_m2': engine_inertia,
                    'torque_lut': {'source': curve.source, 'points': curve.points, 'x_unit': 'rpm', 'y_unit': 'Nm'}},
                   {'traction': traction, 'forward_ratios': gears, 'reverse_ratio': reverse, 'final_ratio': final},
                   brakes, compound, model, sources, diagnostics)
