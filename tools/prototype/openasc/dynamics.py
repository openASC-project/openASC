"""Independent first-drive dynamics: four planar force patches and geometric support.

This prototype is not the original simulator's tyre, suspension or drivetrain model.
The fixed step is independent of HTTP polling and rendering.
"""
from dataclasses import asdict, dataclass
import math

from .config import ContentError, LutTable

DT = 1 / 120
GRAVITY = 9.81
CORNER_GAIN = 7.
ROLLING = .015
DRAG = .4


def unit(vector):
    size = math.hypot(*vector)
    return tuple(v / size for v in vector)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class Controls:
    throttle: float = 0.
    brake: float = 0.
    steer: float = 0.
    handbrake: float = 0.
    gear: int = 1

    @classmethod
    def from_dict(cls, values, gears):
        if not isinstance(values, dict) or set(values) != {'throttle', 'brake', 'steer', 'handbrake', 'gear'}:
            raise ContentError('drive_input', 'A complete throttle/brake/steer/handbrake/gear input is required.')
        for name in ('throttle', 'brake', 'handbrake', 'steer'):
            value = values[name]
            low = -1 if name == 'steer' else 0
            if type(value) not in (int, float) or not low <= value <= 1 or not math.isfinite(value):
                raise ContentError('drive_input', f'{name} is outside its finite input range.')
        if type(values['gear']) is not int or not -1 <= values['gear'] <= gears:
            raise ContentError('drive_input', 'Gear must be -1 (reverse), 0 (neutral), or a supported forward gear.')
        return cls(**values)


class Dynamics:
    def __init__(self, vehicle, ground, spawn, walls=None, body=None):
        self.vehicle, self.ground, self.spawn = vehicle, ground, spawn
        self.walls, self.body = walls, body
        if (walls is None) != (body is None):
            raise ContentError('drive_body', 'Body and wall geometry must be provided together.')
        self.offsets = vehicle.wheel_offsets()
        self.power = LutTable(vehicle.engine['torque_lut']['source'], vehicle.engine['torque_lut']['points'])
        self.mass = vehicle.mass_kg
        width, _, length = vehicle.inertia_dimensions_m
        self.inertia = self.mass * (width * width + length * length) / 12
        self.max_steer = math.radians(vehicle.steer_lock_deg / vehicle.steer_ratio)
        if (vehicle.drivetrain['traction'] not in ('RWD', 'FWD')
                or any(d['code'] == 'vehicle_turbo_unimplemented' for d in vehicle.diagnostics)
                or not 50 <= self.mass <= 10000 or not .5 <= vehicle.wheelbase_m <= 10
                or not .01 <= self.max_steer <= .8 or not .1 <= width <= 20 or not .1 <= length <= 20
                or any(not .1 <= a['track_m'] <= 6 or not .05 <= a['tyre_radius_m'] <= 2
                       or abs(a['base_y_m']) > 2 for a in vehicle.axles.values())
                or any(abs(y) > 20000 for _, y in self.power.points)
                or vehicle.engine['limiter_rpm'] > 30000
                or any(abs(r) > 100 for r in (*vehicle.drivetrain['forward_ratios'], vehicle.drivetrain['reverse_ratio']))
                or vehicle.drivetrain['final_ratio'] > 20
                or vehicle.brakes['max_torque_nm'] > 100000 or vehicle.brakes['handbrake_torque_nm'] > 100000):
            raise ContentError('drive_subset', 'Vehicle is outside the bounded naturally aspirated RWD/FWD prototype subset.')
        self.reset()

    def reset(self):
        self.x, self.y, self.z = self.spawn.position
        self.yaw = math.atan2(self.spawn.forward[0], self.spawn.forward[2])
        self.vx = self.vz = self.yaw_rate = self.steer_angle = 0.
        self.tick = 0
        self.controls = Controls()
        self.paused, self.reason = True, 'Ready — resume to drive'
        self.rpm = self.vehicle.engine['idle_rpm']
        self.contacts = self.support(self.x, self.z, self.yaw, self.y + 1., 20.)
        self.y, self.up = self.pose_height(self.contacts)
        self.forces = []
        self.wall_contacts, self.last_wall_contact, self.wall_motion_clamps = 0, None, 0
        if self.walls is not None:
            from .walls import Pose
            if self.walls.overlaps(self.body, Pose((self.x, self.y, self.z), self.yaw)):
                raise ContentError('drive_spawn_wall_overlap', 'The selected spawn body proxy overlaps static WALL geometry.')

    def support(self, x, z, yaw, ray_y, distance=5.):
        left, forward = (math.cos(yaw), -math.sin(yaw)), (math.sin(yaw), math.cos(yaw))
        contacts = {}
        for name, offset in self.offsets.items():
            hit = self.ground.probe((x + offset[0] * left[0] + offset[2] * forward[0], ray_y,
                                     z + offset[0] * left[1] + offset[2] * forward[1]), distance)
            if hit is None or hit['normal'][1] < .5:
                raise ContentError('drive_no_support', f'{name} has no supported ground or exceeds the prototype slope limit.')
            if not 0 <= hit['surface']['friction'] <= 5:
                raise ContentError('drive_surface_range', f'{name} surface friction is outside the finite 0–5 prototype range.')
            contacts[name] = hit
        return contacts

    def pose_height(self, contacts):
        heights, normals = [], [0., 0., 0.]
        for name, hit in contacts.items():
            axle = name.split('_')[0]
            a = self.vehicle.axles[axle]
            fraction = self.vehicle.front_weight_fraction if axle == 'front' else 1 - self.vehicle.front_weight_fraction
            weight = fraction / 2
            heights.append((hit['position'][1] + a['tyre_radius_m'] - a['base_y_m']) * weight)
            for i in range(3):
                normals[i] += hit['normal'][i] * weight
        return sum(heights), unit(normals)

    def set_controls(self, controls):
        if not isinstance(controls, Controls):
            raise ContentError('drive_input', 'Expected a validated Controls value.')
        controls = Controls.from_dict(asdict(controls), len(self.vehicle.drivetrain['forward_ratios']))
        if controls.gear and math.copysign(1, controls.gear) * self.longitudinal_speed() < -1:
            raise ContentError('drive_direction', 'Stop before selecting the opposite travel direction.')
        self.controls = controls

    def longitudinal_speed(self):
        return self.vx * math.sin(self.yaw) + self.vz * math.cos(self.yaw)

    def pause(self, reason='Paused'):
        self.paused, self.reason = True, reason
        self.controls = Controls(gear=self.controls.gear)

    def resume(self):
        self.paused, self.reason = False, ''

    def wall_motion(self, velocity, rate):
        from .walls import Pose, resolve_contacts
        pose = Pose((self.x, self.y, self.z), self.yaw)
        remaining, events, last = DT, 0, None
        for _ in range(8):
            vx, vz = velocity
            x, z = pose.position[0] + vx * remaining, pose.position[2] + vz * remaining
            yaw = math.remainder(pose.yaw + rate * remaining, 2 * math.pi)
            y = pose.position[1]
            try:
                y, _ = self.pose_height(self.support(x, z, yaw, y + 2))
            except ContentError:
                pass  # A wall can be hit before an unsupported endpoint; final support is checked below.
            end = Pose((x, y, z), yaw)
            hits = self.walls.sweep(self.body, pose, end)
            if not hits:
                return end, velocity, rate, events, last, False
            fraction = min(hit['fraction'] for hit in hits)
            pose = pose.between(end, fraction)
            velocity, rate, impulse = resolve_contacts(hits, pose, self.mass, self.inertia, velocity, rate)
            events += 1
            last = {key: hits[0][key] for key in ('mesh', 'model', 'normal', 'point', 'triangle')}
            last['normal_impulse_ns'] = impulse
            remaining *= 1 - fraction
            if remaining < 1e-8:
                return pose, velocity, rate, events, last, False
        # Preserve the last nonpenetrating pose when a corner needs more contact events.
        return pose, velocity, rate, events, last, True

    def step(self):
        if self.paused:
            return
        c, v = self.controls, self.vehicle
        sine, cosine = math.sin(self.yaw), math.cos(self.yaw)
        u = self.vx * sine + self.vz * cosine
        lateral = self.vx * cosine - self.vz * sine
        change = max(-1.2 * DT, min(1.2 * DT, c.steer * self.max_steer - self.steer_angle))
        self.steer_angle += change
        ratio = (v.drivetrain['reverse_ratio'] if c.gear == -1 else v.drivetrain['forward_ratios'][c.gear - 1] if c.gear else 0) * v.drivetrain['final_ratio']
        driven = 'rear' if v.drivetrain['traction'] == 'RWD' else 'front'
        radius = v.axles[driven]['tyre_radius_m']
        self.rpm = max(v.engine['idle_rpm'], abs(u) / radius * abs(ratio) * 60 / (2 * math.pi)) if ratio else v.engine['idle_rpm'] + c.throttle * (v.engine['limiter_rpm'] - v.engine['idle_rpm'])
        torque = self.power.sample(min(self.rpm, v.engine['limiter_rpm'])) * c.throttle if self.rpm < v.engine['limiter_rpm'] else 0.
        fx, fy, moment = 0., 0., 0.
        gravity_x, gravity_z = (GRAVITY * self.up[i] * self.up[1] for i in (0, 2))
        gravity_u = gravity_x * sine + gravity_z * cosine
        gravity_v = gravity_x * cosine - gravity_z * sine
        forces = []
        for name, offset in self.offsets.items():
            axle = name.split('_')[0]
            x, _, z = offset
            front = axle == 'front'
            share = v.front_weight_fraction if front else 1 - v.front_weight_fraction
            load = self.mass * GRAVITY * share / 2 * self.contacts[name]['normal'][1]
            delta = self.steer_angle if front else 0.
            cs, sn = math.cos(delta), math.sin(delta)
            patch_u, patch_lateral = u - self.yaw_rate * x, lateral + self.yaw_rate * z
            wheel_u = patch_u * cs + patch_lateral * sn
            wheel_v = patch_lateral * cs - patch_u * sn
            longitudinal = torque * ratio * .9 / (2 * radius) if axle == driven else 0.
            brake_share = v.brakes['front_share'] if front else 1 - v.brakes['front_share']
            brake_force = (v.brakes['max_torque_nm'] * c.brake * brake_share
                           + (v.brakes['handbrake_torque_nm'] * c.handbrake if not front else 0)) / (2 * v.axles[axle]['tyre_radius_m'])
            resistance = brake_force + ROLLING * load
            patch_mass = self.mass * share / 2
            if abs(wheel_u) < .15:
                desired = longitudinal + patch_mass * (wheel_u / DT + gravity_u * cs + gravity_v * sn)
                longitudinal -= max(-resistance, min(resistance, desired))
            else:
                longitudinal -= math.copysign(resistance, wheel_u)
            sideways = -CORNER_GAIN * load * math.atan2(wheel_v, max(abs(wheel_u), 1.5))
            if math.hypot(wheel_u, wheel_v) < .15 and c.brake > 0 and not c.throttle:
                sideways = -patch_mass * (wheel_v / DT + gravity_v * cs - gravity_u * sn)
            limit = max(0., self.contacts[name]['surface']['friction']) * load
            magnitude = math.hypot(longitudinal, sideways)
            factor = min(1., limit / magnitude) if magnitude else 1.
            longitudinal, sideways = longitudinal * factor, sideways * factor
            patch_fx, patch_fy = longitudinal * cs - sideways * sn, longitudinal * sn + sideways * cs
            fx += patch_fx
            fy += patch_fy
            moment += z * patch_fy - x * patch_fx
            forces.append({'wheel': name, 'longitudinal_n': longitudinal, 'lateral_n': sideways,
                           'normal_n': load, 'limit_n': limit})
        fx -= DRAG * u * abs(u)
        vx = self.vx + ((fx * sine + fy * cosine) / self.mass + gravity_x) * DT
        vz = self.vz + ((fx * cosine - fy * sine) / self.mass + gravity_z) * DT
        rate = self.yaw_rate + moment / self.inertia * DT
        # Remove floating-point residue only; friction-limited forces do the stopping.
        if math.hypot(vx, vz) < 1e-9 and abs(rate) < 1e-9:
            vx = vz = rate = 0.
        yaw = math.remainder(self.yaw + rate * DT, 2 * math.pi)
        x, z = self.x + vx * DT, self.z + vz * DT
        try:
            if not all(math.isfinite(value) for value in (vx, vz, rate, yaw, x, z)) or math.hypot(vx, vz) > 150 or abs(rate) > 10:
                raise ContentError('drive_numeric_limit', 'Prototype velocity or numeric limits exceeded.')
            events, last, clamped = 0, None, False
            if self.walls is not None:
                pose, (vx, vz), rate, events, last, clamped = self.wall_motion((vx, vz), rate)
                x, _, z = pose.position
                yaw = math.remainder(pose.yaw, 2 * math.pi)
                if not all(math.isfinite(n) for n in (vx, vz, rate)) or abs(rate) > 10:
                    raise ContentError('drive_numeric_limit', 'Body contact exceeds the prototype yaw/numeric limit.')
            contacts = self.support(x, z, yaw, self.y + 2)
            height, up = self.pose_height(contacts)
            if abs(height - self.y) > .35:
                raise ContentError('drive_height_step', 'Ground height discontinuity exceeds the 0.35 metre prototype limit.')
            if self.walls is not None:
                from .walls import Pose
                if self.walls.overlaps(self.body, Pose((x, height, z), yaw)):
                    raise ContentError('wall_terrain_overlap', 'Terrain attachment would put the body proxy inside WALL geometry.')
        except ContentError as error:
            self.vx = self.vz = self.yaw_rate = 0.
            self.pause(error.diagnostic.code + ': ' + error.diagnostic.message)
            return
        self.x, self.y, self.z, self.yaw = x, height, z, yaw
        self.vx, self.vz, self.yaw_rate, self.up = vx, vz, rate, up
        self.contacts, self.forces = contacts, forces
        self.wall_contacts += events
        self.wall_motion_clamps += int(clamped)
        if last is not None:
            self.last_wall_contact = last
        self.tick += 1

    def world(self):
        forward = (math.sin(self.yaw), 0., math.cos(self.yaw))
        dot = sum(forward[i] * self.up[i] for i in range(3))
        forward = unit(tuple(forward[i] - dot * self.up[i] for i in range(3)))
        left = unit(cross(self.up, forward))
        return (*left, 0., *self.up, 0., *forward, 0., self.x, self.y, self.z, 1.)

    def snapshot(self):
        return {'tick': self.tick, 'simulation_seconds': self.tick * DT, 'position': [self.x, self.y, self.z],
                'world': self.world(), 'velocity_xz': [self.vx, self.vz], 'yaw': self.yaw,
                'yaw_rate': self.yaw_rate, 'speed_kmh': self.longitudinal_speed() * 3.6,
                'rpm': min(self.rpm, self.vehicle.engine['limiter_rpm']), 'steer_angle_rad': self.steer_angle,
                'gear': self.controls.gear, 'paused': self.paused, 'reason': self.reason,
                'controls': asdict(self.controls), 'forces': self.forces,
                'wall_contacts': self.wall_contacts, 'last_wall_contact': self.last_wall_contact,
                'wall_motion_clamps': self.wall_motion_clamps,
                'surfaces': {name: hit['surface']['key'] for name, hit in self.contacts.items()}}
