//! Inelastic static-world contact impulses, with coupled normal response and metric Coulomb disks.
use super::impulse::{ChassisImpulseWork, energy, point_velocity};
use super::*;
#[derive(Clone, Copy, Debug, Serialize)]
pub struct ChassisContact {
    pub point_world_m: [f64; 3],
    pub normal_world: [f64; 3],
    pub friction: f64,
}
#[derive(Clone, Copy, Debug, Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChassisContactSettings {
    pub max_iterations: usize,
    pub velocity_tolerance_m_s: f64,
}
impl Default for ChassisContactSettings {
    fn default() -> Self {
        Self {
            max_iterations: 128,
            velocity_tolerance_m_s: 1e-7,
        }
    }
}
#[derive(Debug, Serialize)]
pub struct ChassisContactResult {
    pub contact: ChassisContact,
    pub impulse_world_ns: [f64; 3],
    pub normal_impulse_ns: f64,
    pub tangent_impulse_world_ns: [f64; 3],
    pub point_velocity_before_world_m_s: [f64; 3],
    pub point_velocity_after_world_m_s: [f64; 3],
}
#[derive(Debug, Serialize)]
pub struct ChassisContactSample {
    pub contacts: Vec<ChassisContactResult>,
    pub iterations: usize,
    pub max_projected_velocity_residual_m_s: f64,
    pub net_impulse_world_ns: [f64; 3],
    pub net_moment_about_chassis_com_nms: [f64; 3],
    pub work: ChassisImpulseWork,
}
fn fail(message: &str) -> Error {
    Error::new("physics_chassis_contacts", message)
}
fn dot(a: [f64; 6], b: [f64; 6]) -> f64 {
    (0..6).map(|i| a[i] * b[i]).sum()
}
fn velocity(s: &State) -> [f64; 6] {
    let v = s.velocity_world_m_s;
    let w = s.angular_velocity_world_rad_s;
    [v[0], v[1], v[2], w[0], w[1], w[2]]
}
fn wrench(direction: DVec3, lever: DVec3) -> [f64; 6] {
    let m = lever.cross(direction);
    [direction.x, direction.y, direction.z, m.x, m.y, m.z]
}
fn solve2(k: [f64; 3], shift: f64, b: [f64; 2]) -> Result<[f64; 2]> {
    let scale = (k[0] + shift).max(k[2] + shift);
    if !scale.is_finite() || scale <= 0. {
        return Err(fail("Nonfinite contact tangent projection scale"));
    }
    let a = (k[0] + shift) / scale;
    let c = (k[2] + shift) / scale;
    let off = k[1] / scale;
    let det = a * c - off * off;
    if !det.is_finite() || det <= 0. {
        return Err(fail("Nonpositive contact tangent projection matrix"));
    }
    let x = b[0] / scale;
    let y = b[1] / scale;
    let result = [(c * x - off * y) / det, (a * y - off * x) / det];
    if result.iter().any(|v| !v.is_finite()) {
        return Err(fail("Nonfinite contact tangent impulse"));
    }
    Ok(result)
}
fn disk(k: [f64; 3], old: [f64; 2], speed: [f64; 2], radius: f64) -> Result<[f64; 2]> {
    if radius == 0. {
        return Ok([0.; 2]);
    }
    let b = [
        k[0] * old[0] + k[1] * old[1] - speed[0],
        k[1] * old[0] + k[2] * old[1] - speed[1],
    ];
    let free = solve2(k, 0., b)?;
    if free[0].hypot(free[1]) <= radius {
        return Ok(free);
    }
    // The shift solves (K + alpha I) lambda = b with |lambda| = mu * lambda_n.
    // Unlike radial clipping of the unconstrained solution, this respects anisotropic mobility.
    let mut hi = b[0].hypot(b[1]) / radius;
    let mut lo = 0.;
    if !hi.is_finite() || hi <= 0. {
        return Err(fail("Contact friction projection exceeds numeric range"));
    }
    for _ in 0..64 {
        let mid = lo + (hi - lo) * 0.5;
        let x = solve2(k, mid, b)?;
        if x[0].hypot(x[1]) > radius {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let mut x = solve2(k, hi, b)?;
    let norm = x[0].hypot(x[1]);
    if norm > radius {
        for value in &mut x {
            *value *= radius / norm;
        }
    }
    Ok(x)
}
struct Prepared {
    contact: ChassisContact,
    directions: [DVec3; 3],
    rows: [[f64; 6]; 3],
    response: [[f64; 6]; 3],
    kn: f64,
    kt: [f64; 3],
    lambda: [f64; 3],
}
impl Prepared {
    fn change(&mut self, axis: usize, value: f64, u: &mut [f64; 6]) -> Result<()> {
        if !value.is_finite() || value.abs() > 1e18 {
            return Err(fail("Contact impulse exceeds +/-1e18 N s"));
        }
        let delta = value - self.lambda[axis];
        for (v, response) in u.iter_mut().zip(self.response[axis]) {
            *v += response * delta;
        }
        self.lambda[axis] = value;
        if u.iter().any(|v| !v.is_finite() || v.abs() > 1e18) {
            return Err(fail("Contact velocity iteration exceeds numeric range"));
        }
        Ok(())
    }
    fn residual(&self, u: [f64; 6]) -> f64 {
        let vn = dot(self.rows[0], u);
        let normal = ((self.lambda[0] - vn / self.kn).max(0.) - self.lambda[0]).abs() * self.kn;
        let vt = [dot(self.rows[1], u), dot(self.rows[2], u)];
        let speed = vt[0].hypot(vt[1]);
        let radius = self.contact.friction * self.lambda[0];
        let length = self.lambda[1].hypot(self.lambda[2]);
        let friction = if radius == 0. {
            0.
        } else if length < radius * (1. - 1e-9) {
            speed
        } else if length > 0. {
            (vt[0] + speed * (self.lambda[1] / length))
                .hypot(vt[1] + speed * (self.lambda[2] / length))
        } else {
            speed
        };
        // An arbitrarily small friction budget cannot change velocity by more than this bound.
        normal.max(friction.min(2. * radius * self.kt[0].max(self.kt[2])))
    }
}
impl ChassisSystem {
    /// Resolve at most 64 active chassis contacts against a fixed world, without restitution.
    /// Positions/rack prescriptions remain fixed. Failure/nonconvergence commits no state.
    pub fn resolve_chassis_contacts(
        &mut self,
        contacts: &[ChassisContact],
        settings: ChassisContactSettings,
    ) -> Result<ChassisContactSample> {
        if contacts.len() > 64
            || !(1..=1024).contains(&settings.max_iterations)
            || !settings.velocity_tolerance_m_s.is_finite()
            || !(1e-9..=1e-3).contains(&settings.velocity_tolerance_m_s)
        {
            return Err(fail(
                "Expected at most 64 contacts, 1..1024 iterations and tolerance in 1e-9..1e-3 m/s",
            ));
        }
        let before = self.kinematics()?;
        let n =
            6 + self.uprights.len() + self.uprights.iter().filter(|u| u.wheel.is_some()).count();
        if contacts.is_empty() {
            let k = energy(&before);
            scalar(k, 1e30)?;
            return Ok(ChassisContactSample {
                contacts: Vec::new(),
                iterations: 0,
                max_projected_velocity_residual_m_s: 0.,
                net_impulse_world_ns: [0.; 3],
                net_moment_about_chassis_com_nms: [0.; 3],
                work: ChassisImpulseWork {
                    kinetic_energy_before_j: k,
                    kinetic_energy_after_j: k,
                    external_impulse_work_j: 0.,
                    prescribed_rack_impulse_work_j: 0.,
                    energy_balance_residual_j: 0.,
                    dof_count: n,
                },
            });
        }
        let (matrix, _) = self.impulse_matrix()?;
        let factor = crate::coupled_linear::factor(matrix, n)?;
        let mut mobility = [[0.; 6]; 6];
        for j in 0..6 {
            let mut rhs = [0.; 14];
            rhs[j] = 1.;
            let x = factor.solve(rhs)?;
            for i in 0..6 {
                mobility[i][j] = x[i];
            }
        }
        for i in 0..6 {
            for j in 0..i {
                let scale = (mobility[i][i] * mobility[j][j]).sqrt();
                if !scale.is_finite()
                    || scale <= 0.
                    || (mobility[i][j] - mobility[j][i]).abs() > 1e-8 * scale
                {
                    return Err(fail("Condensed chassis mobility is not reciprocal"));
                }
                let average = 0.5 * (mobility[i][j] + mobility[j][i]);
                mobility[i][j] = average;
                mobility[j][i] = average;
            }
        }
        let mut prepared = Vec::new();
        for &contact in contacts {
            let point = vector(DVec3::from_array(contact.point_world_m), 1e7)?;
            let normal = vector(DVec3::from_array(contact.normal_world), 2.)?;
            if (normal.length_squared() - 1.).abs() > 1e-8
                || !contact.friction.is_finite()
                || !(0. ..=2.).contains(&contact.friction)
            {
                return Err(fail(
                    "Contacts require unit normals within 1e-8 and friction in 0..2",
                ));
            }
            let normal = normal.normalize();
            let abs = normal.abs();
            let reference = if abs.x <= abs.y && abs.x <= abs.z {
                DVec3::X
            } else if abs.y <= abs.z {
                DVec3::Y
            } else {
                DVec3::Z
            };
            let tangent = normal.cross(reference).normalize();
            let directions = [normal, tangent, normal.cross(tangent)];
            let lever = point - DVec3::from_array(before.chassis.position_m);
            let rows = directions.map(|d| wrench(d, lever));
            let response = rows.map(|row| std::array::from_fn(|i| dot(mobility[i], row)));
            let kn = dot(rows[0], response[0]);
            let kt = [
                dot(rows[1], response[1]),
                0.5 * (dot(rows[1], response[2]) + dot(rows[2], response[1])),
                dot(rows[2], response[2]),
            ];
            let scale = kt[0].max(kt[2]);
            if !kn.is_finite()
                || !(1e-24..=1e24).contains(&kn)
                || !scale.is_finite()
                || !(1e-24..=1e24).contains(&scale)
                || (kt[0] / scale) * (kt[2] / scale) - (kt[1] / scale).powi(2) <= 1e-12
            {
                return Err(fail("Nonpositive or poorly conditioned contact mobility"));
            }
            prepared.push(Prepared {
                contact: ChassisContact {
                    normal_world: normal.to_array(),
                    ..contact
                },
                directions,
                rows,
                response,
                kn,
                kt,
                lambda: [0.; 3],
            });
        }
        let mut u = velocity(&before.chassis);
        let mut converged = None;
        for iteration in 1..=settings.max_iterations {
            for p in &mut prepared {
                let normal = (p.lambda[0] - dot(p.rows[0], u) / p.kn).max(0.);
                p.change(0, normal, &mut u)?;
                let tangent = disk(
                    p.kt,
                    [p.lambda[1], p.lambda[2]],
                    [dot(p.rows[1], u), dot(p.rows[2], u)],
                    p.contact.friction * p.lambda[0],
                )?;
                p.change(1, tangent[0], &mut u)?;
                p.change(2, tangent[1], &mut u)?;
            }
            let residual = prepared.iter().map(|p| p.residual(u)).fold(0., f64::max);
            if residual <= settings.velocity_tolerance_m_s {
                converged = Some(iteration);
                break;
            }
        }
        let iterations = converged.ok_or_else(|| {
            fail(&format!(
                "Contact iteration did not converge within the configured budget (residual {} m/s)",
                prepared.iter().map(|p| p.residual(u)).fold(0., f64::max)
            ))
        })?;
        let mut rhs = [0.; 14];
        for p in &prepared {
            for axis in 0..3 {
                for (i, v) in rhs.iter_mut().enumerate().take(6) {
                    *v += p.rows[axis][i] * p.lambda[axis];
                }
            }
        }
        let linear = DVec3::from_slice(&rhs[..3]);
        let moment = DVec3::from_slice(&rhs[3..6]);
        let delta = factor.solve(rhs)?;
        let mut candidate = self.clone();
        let work = candidate.apply_impulse_delta(&before, delta, n, linear, moment)?;
        let after = candidate.chassis.state();
        let actual = velocity(&after);
        let residual = prepared
            .iter()
            .map(|p| p.residual(actual))
            .fold(0., f64::max);
        if !residual.is_finite() || residual > settings.velocity_tolerance_m_s * (1. + 1e-6) {
            return Err(fail(
                "Committed contact velocities fail the convergence postcheck",
            ));
        }
        let mut records = Vec::new();
        let mut scale = 1.;
        for p in prepared {
            let tangent = p.directions[1] * p.lambda[1] + p.directions[2] * p.lambda[2];
            let impulse = p.directions[0] * p.lambda[0] + tangent;
            let v0 = point_velocity(&before.chassis, DVec3::from_array(p.contact.point_world_m));
            let v1 = point_velocity(&after, DVec3::from_array(p.contact.point_world_m));
            scale += impulse.dot(v0).abs() + impulse.dot(v1).abs();
            records.push(ChassisContactResult {
                contact: p.contact,
                impulse_world_ns: impulse.to_array(),
                normal_impulse_ns: p.lambda[0],
                tangent_impulse_world_ns: tangent.to_array(),
                point_velocity_before_world_m_s: v0.to_array(),
                point_velocity_after_world_m_s: v1.to_array(),
            });
        }
        if work.external_impulse_work_j > 1e-8 * scale {
            return Err(fail(
                "Static-world contact impulses would add external mechanical energy",
            ));
        }
        *self = candidate;
        Ok(ChassisContactSample {
            contacts: records,
            iterations,
            max_projected_velocity_residual_m_s: residual,
            net_impulse_world_ns: linear.to_array(),
            net_moment_about_chassis_com_nms: moment.to_array(),
            work,
        })
    }
}
