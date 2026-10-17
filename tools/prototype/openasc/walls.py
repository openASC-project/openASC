"""Static wall triangles, a yaw-oriented body proxy, and continuous SAT queries.

The body is an upright box enclosing transformed collider.kn5 vertices. This is
an independent prototype contact shape, not a recreation of original collisions.
"""
from array import array
from dataclasses import dataclass
import math
from pathlib import Path

from .config import ContentError
from .kn5 import IDENTITY, load_kn5, multiply, transform, values_from_bytes
from .scene import confined
from .spatial import build_bvh, box_candidates

SKIN = .002
EPS = 1e-7
MAX_TRIANGLES = 250_000


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class Pose:
    position: tuple
    yaw: float

    def between(self, other, fraction):
        return Pose(tuple(a + (b - a) * fraction for a, b in zip(self.position, other.position)),
                    self.yaw + math.remainder(other.yaw - self.yaw, 2 * math.pi) * fraction)


@dataclass(frozen=True)
class Body:
    center: tuple
    half: tuple
    source: str = '<body>'

    @property
    def radius(self):
        return math.hypot(abs(self.center[0]) + self.half[0], abs(self.center[2]) + self.half[2])

    def frame(self, pose):
        sine, cosine = math.sin(pose.yaw), math.cos(pose.yaw)
        axes = ((cosine, 0., -sine), (0., 1., 0.), (sine, 0., cosine))
        center = tuple(pose.position[a] + sum(axes[i][a] * self.center[i] for i in range(3)) for a in range(3))
        return center, axes


def load_body(installation, vehicle):
    root = confined(Path(installation).resolve(), f'content/cars/{vehicle.car}', '<body>')
    path = confined(root, 'collider.kn5', '<body>')
    source = f'content/cars/{vehicle.car}/collider.kn5'
    parsed = load_kn5(path, source)
    if any(d['code'] == 'skinned_mesh_skipped' for d in parsed.diagnostics):
        raise ContentError('body_unsupported', 'A skinned collider cannot define the static body proxy.', source)
    pitch = math.radians(vehicle.graphics_pitch_deg)
    graphics = list(IDENTITY)
    graphics[5], graphics[6], graphics[9], graphics[10] = math.cos(pitch), math.sin(pitch), -math.sin(pitch), math.cos(pitch)
    graphics[12:15] = vehicle.graphics_offset_m
    low, high, count = [math.inf] * 3, [-math.inf] * 3, 0
    for mesh in parsed.meshes:
        if not mesh.active or not mesh.indices or mesh.lod[0] > 0:
            continue
        vertices = values_from_bytes(mesh.vertices, 'f')
        world = multiply(graphics, mesh.world)
        for index in set(values_from_bytes(mesh.indices, 'H')):
            point = transform(world, vertices[index * 11:index * 11 + 3])
            if not all(math.isfinite(v) and abs(v) <= 20 for v in point):
                raise ContentError('body_range', 'Collider vertices exceed the finite +/-20 metre prototype range.', source)
            for a in range(3):
                low[a], high[a] = min(low[a], point[a]), max(high[a], point[a])
            count += 1
    half = tuple((high[a] - low[a]) / 2 for a in range(3))
    if not count or any(not .01 <= value <= 10 for value in half):
        raise ContentError('body_shape', 'A nonempty three-dimensional collider.kn5 is required for body contact.', source)
    return Body(tuple((low[a] + high[a]) / 2 for a in range(3)), half, source)


def separation(body, pose, triangle, overlap_only=False):
    """Normalized SAT axes: box faces, triangle face, and edge cross products.

    The maximum signed separation is a conservative lower bound on distance.
    Its normal points from the triangle toward the box.
    """
    center, basis = body.frame(pose)
    points = [sub(point, center) for point in triangle]
    edges = [sub(points[(i + 1) % 3], points[i]) for i in range(3)]
    axes = [*basis, cross(edges[0], edges[1])]
    axes.extend(cross(axis, edge) for axis in basis for edge in edges)
    best, normal = -math.inf, (0., 1., 0.)
    for raw in axes:
        size = math.hypot(*raw)
        if size <= 1e-12:
            continue
        axis = tuple(v / size for v in raw)
        radius = body.half[0] * abs(dot(axis, basis[0])) + body.half[1] * abs(axis[1]) + body.half[2] * abs(dot(axis, basis[2]))
        projections = dot(points[0], axis), dot(points[1], axis), dot(points[2], axis)
        lo, hi = min(projections), max(projections)
        positive, negative = -radius - hi, lo - radius
        gap = max(positive, negative)
        if overlap_only and gap >= -1e-6:
            return gap, axis
        if gap > best:
            best = gap
            normal = axis if positive >= negative else tuple(-v for v in axis)
    return best, normal


def projection_curves(body, start, end, axis):
    """Corner projection curves, partitioned at all derivative roots."""
    delta = math.remainder(end.yaw - start.yaw, 2 * math.pi)
    movement = dot(sub(end.position, start.position), axis)
    curves = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                x, y, z = (body.center[i] + sign * body.half[i] for i, sign in enumerate((sx, sy, sz)))
                a, b = axis[0] * x + axis[2] * z, axis[0] * z - axis[2] * x
                times = [0., 1.]
                radius = math.hypot(a, b)
                if abs(delta) > 1e-12 and radius > 1e-12:
                    value = -movement / (delta * radius)
                    if abs(value) <= 1:
                        phase = math.atan2(a, b)
                        root = math.acos(value)
                        for sign in (-1, 1):
                            angle = sign * root - phase
                            for winding in range(-2, 3):
                                t = (angle + 2 * math.pi * winding - math.remainder(start.yaw, 2 * math.pi)) / delta
                                if 0 < t < 1:
                                    times.append(t)
                base = dot(start.position, axis) + y * axis[1]
                curves.append((base, movement, a, b, start.yaw, delta, sorted(times)))
    return curves


def projection_value(curve, t):
    base, movement, a, b, angle, delta, _ = curve
    return base + movement * t + a * math.cos(angle + delta * t) + b * math.sin(angle + delta * t)


def swept_minimum(body, start, end, axis):
    """Exact minimum corner projection, including interior rotational extrema."""
    return min(projection_value(curve, t) for curve in projection_curves(body, start, end, axis) for t in curve[-1])


def axis_touch(body, start, end, axis, threshold):
    """First crossing of a fixed separating plane, found on monotonic intervals.

    Before this time the complete box remains separated, so jumping directly to
    this root is conservative even when translation and rotation almost cancel.
    """
    if abs(math.remainder(end.yaw - start.yaw, 2 * math.pi)) < 1e-12:
        movement = dot(sub(end.position, start.position), axis)
        if movement >= 0:
            return None
        center, basis = body.frame(start)
        low = dot(center, axis) - sum(body.half[i] * abs(dot(basis[i], axis)) for i in range(3))
        time = (threshold - low) / movement
        return max(0., time) if time <= 1 else None
    first = None
    for curve in projection_curves(body, start, end, axis):
        times = curve[-1]
        for lo, hi in zip(times, times[1:]):
            if projection_value(curve, lo) <= threshold:
                root = lo
            elif projection_value(curve, hi) <= threshold:
                for _ in range(40):
                    middle = (lo + hi) / 2
                    if projection_value(curve, middle) > threshold:
                        lo = middle
                    else:
                        hi = middle
                root = hi
            else:
                continue
            first = root if first is None else min(first, root)
            break
    return first


def contact_points(body, pose, triangle, normal):
    """Clip the triangle to the skin-expanded box; retain planar manifold ends."""
    center, axes = body.frame(pose)
    polygon = [tuple(dot(sub(p, center), axis) for axis in axes) for p in triangle]
    for a in range(3):
        for sign in (-1, 1):
            limit = body.half[a] + SKIN + 2e-6
            output = []
            if not polygon:
                break
            previous = polygon[-1]
            d0 = sign * previous[a] - limit
            for current in polygon:
                d1 = sign * current[a] - limit
                if (d0 <= 0) != (d1 <= 0):
                    t = d0 / (d0 - d1)
                    output.append(tuple(previous[i] + t * (current[i] - previous[i]) for i in range(3)))
                if d1 <= 0:
                    output.append(current)
                previous, d0 = current, d1
            polygon = output
    if not polygon:
        # This should only be a tolerance failure. Refuse to invent a remote lever arm.
        raise ContentError('wall_manifold', 'A wall contact could not be clipped to the local body proxy.')
    points = [tuple(center[a] + sum(p[i] * axes[i][a] for i in range(3)) for a in range(3)) for p in polygon]
    tangent = (-normal[2], 0., normal[0])
    first, last = min(points, key=lambda p: dot(p, tangent)), max(points, key=lambda p: dot(p, tangent))
    if math.hypot(first[0] - last[0], first[2] - last[2]) < 1e-5:
        return [tuple(sum(p[i] for p in points) / len(points) for i in range(3))]
    return [first, last]


class Walls:
    def __init__(self):
        self.triangles, self.mesh_ids = array('d'), array('I')
        self.meshes, self.bounds, self.order = [], [], []
        self.root, self.input_triangles, self.degenerate = None, 0, 0

    def add_mesh(self, mesh, world, model):
        count = len(mesh.indices) // 6
        if self.input_triangles + count > MAX_TRIANGLES:
            raise ContentError('wall_limit', 'Wall input exceeds 250000 triangles.', model)
        self.input_triangles += count
        vertices = values_from_bytes(mesh.vertices, 'f')
        points = [transform(world, vertices[i:i + 3]) for i in range(0, len(vertices), 11)]
        if any(not math.isfinite(v) or abs(v) > 10_000_000 for p in points for v in p):
            raise ContentError('wall_range', 'Wall vertices exceed the finite +/-10000000 metre range.', model)
        indices = values_from_bytes(mesh.indices, 'H')
        start = len(self.mesh_ids)
        for i in range(0, len(indices), 3):
            tri = tuple(points[j] for j in indices[i:i + 3])
            edges = sub(tri[1], tri[0]), sub(tri[2], tri[0])
            if math.hypot(*cross(*edges)) <= 1e-12 * max(1., math.hypot(*edges[0]) * math.hypot(*edges[1])):
                self.degenerate += 1
                continue
            self.triangles.extend(v for p in tri for v in p)
            self.mesh_ids.append(len(self.meshes))
            self.bounds.append(tuple(min(p[a] for p in tri) for a in range(3)) + tuple(max(p[a] for p in tri) for a in range(3)))
        self.meshes.append({'name': mesh.name, 'model': model, 'triangles': len(self.mesh_ids) - start})

    def build(self):
        self.root, self.order = build_bvh(self.bounds)

    def triangle(self, index):
        return tuple(tuple(self.triangles[index * 9 + i:index * 9 + i + 3]) for i in (0, 3, 6))

    def candidates(self, body, start, end):
        radius = body.radius + SKIN
        low = (min(start.position[0], end.position[0]) - radius,
               min(start.position[1], end.position[1]) + body.center[1] - body.half[1] - SKIN,
               min(start.position[2], end.position[2]) - radius)
        high = (max(start.position[0], end.position[0]) + radius,
                max(start.position[1], end.position[1]) + body.center[1] + body.half[1] + SKIN,
                max(start.position[2], end.position[2]) + radius)
        return sorted(i for i in box_candidates(self.root, self.order, (*low, *high))
                      if all(self.bounds[i][a] <= high[a] and self.bounds[i][a + 3] >= low[a] for a in range(3)))

    def overlaps(self, body, pose):
        return [index for index in self.candidates(body, pose, pose) if separation(body, pose, self.triangle(index), overlap_only=True)[0] < -1e-6]

    def sweep(self, body, start, end):
        delta = sub(end.position, start.position)
        angle = abs(math.remainder(end.yaw - start.yaw, 2 * math.pi))
        speed = math.hypot(*delta) + angle * body.radius
        hits, earliest = [], 1.
        for index in self.candidates(body, start, end):
            triangle = self.triangle(index)
            gap, normal = separation(body, start, triangle)
            if gap <= SKIN + EPS and swept_minimum(body, start, end, normal) >= max(dot(p, normal) for p in triangle) + SKIN - 2 * EPS:
                continue
            if speed < 1e-12:
                continue
            fraction, found = 0., False
            for _ in range(80):
                current = start.between(end, fraction)
                gap, normal = separation(body, current, triangle)
                if gap <= SKIN + EPS:
                    found = True
                    break
                touch = axis_touch(body, current, end, normal, max(dot(p, normal) for p in triangle) + SKIN)
                if touch is None:
                    break
                fraction += (1 - fraction) * touch
                if fraction > earliest + 1e-6:
                    break
            else:
                raise ContentError('wall_sweep_limit', 'Continuous wall contact did not converge within 80 conservative steps.')
            if not found or fraction > earliest + 1e-6:
                continue
            if fraction < earliest - 1e-6:
                hits, earliest = [], fraction
            pose = start.between(end, fraction)
            mesh = self.meshes[self.mesh_ids[index]]
            for point in contact_points(body, pose, triangle, normal):
                if len(hits) >= 64:
                    raise ContentError('wall_contact_limit', 'Wall sweep exceeds the 64-point contact budget.')
                hits.append({'fraction': fraction, 'normal': normal, 'point': point, 'triangle': index,
                             'mesh': mesh['name'], 'model': mesh['model']})
        return hits

    def summary(self):
        return {'meshes': self.meshes, 'triangles': len(self.mesh_ids), 'input_triangles': self.input_triangles,
                'degenerate_triangles': self.degenerate, 'skin_m': SKIN, 'scope': 'static_wall_triangles'}


def resolve_contacts(contacts, pose, mass, inertia, velocity, rate):
    """Inelastic projected sequential impulses with bounded tangential friction.

    No positional energy bias or restitution is introduced. The planar solver
    cannot resolve nearly vertical contact normals and reports that limitation.
    """
    vx, vz = velocity
    constraints, seen = [], set()
    for contact in contacts:
        nx, _, nz = contact['normal']
        size = math.hypot(nx, nz)
        if size < .2:
            raise ContentError('wall_vertical_contact', 'The upright planar body cannot resolve this vertical wall/ceiling contact.')
        nx, nz = nx / size, nz / size
        rx, rz = contact['point'][0] - pose.position[0], contact['point'][2] - pose.position[2]
        key = tuple(round(v, 5) for v in (rx, rz, nx, nz))
        if key in seen:
            continue
        seen.add(key)
        arm = rz * nx - rx * nz
        tangent_arm = -rz * nz - rx * nx
        constraints.append([nx, nz, rx, rz, arm, tangent_arm, 0., 0.])
    for _ in range(12):
        for constraint in constraints:
            nx, nz, rx, rz, arm, tangent_arm, normal_total, tangent_total = constraint
            relative = (vx + rate * rz) * nx + (vz - rate * rx) * nz
            normal_new = max(0., normal_total - relative / (1 / mass + arm * arm / inertia))
            impulse = normal_new - normal_total
            vx += impulse * nx / mass
            vz += impulse * nz / mass
            rate += impulse * arm / inertia
            tangent_speed = -(vx + rate * rz) * nz + (vz - rate * rx) * nx
            tangent_new = max(-.35 * normal_new, min(.35 * normal_new,
                              tangent_total - tangent_speed / (1 / mass + tangent_arm * tangent_arm / inertia)))
            impulse = tangent_new - tangent_total
            vx -= impulse * nz / mass
            vz += impulse * nx / mass
            rate += impulse * tangent_arm / inertia
            constraint[6:8] = normal_new, tangent_new
    return (vx, vz), rate, sum(c[6] for c in constraints)
