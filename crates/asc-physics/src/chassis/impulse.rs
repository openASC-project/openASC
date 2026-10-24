//! Force-independent instantaneous chassis impulses, retaining free travel and rotor DOFs.
use super::*;

#[derive(Debug, Serialize)]
pub struct ChassisPointResponse {
    /// Point-velocity change per N s of impulse along the queried unit direction (1/kg).
    pub point_velocity_delta_world_per_ns: [f64; 3],
    pub inverse_effective_mass_kg_inv: f64,
    pub dof_count: usize,
}
#[derive(Debug, Serialize)]
pub struct ChassisImpulseSample {
    pub point_velocity_before_world_m_s: [f64; 3],
    pub point_velocity_after_world_m_s: [f64; 3],
    pub kinetic_energy_before_j: f64,
    pub kinetic_energy_after_j: f64,
    pub external_impulse_work_j: f64,
    /// Work supplied by ideal prescribed racks during the velocity discontinuity.
    pub prescribed_rack_impulse_work_j: f64,
    pub energy_balance_residual_j: f64,
    pub dof_count: usize,
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct ChassisImpulseWork {
    pub kinetic_energy_before_j: f64,
    pub kinetic_energy_after_j: f64,
    pub external_impulse_work_j: f64,
    pub prescribed_rack_impulse_work_j: f64,
    pub energy_balance_residual_j: f64,
    pub dof_count: usize,
}
pub(super) fn point_velocity(body: &State, point: DVec3) -> DVec3 {
    DVec3::from_array(body.velocity_world_m_s)
        + DVec3::from_array(body.angular_velocity_world_rad_s)
            .cross(point - DVec3::from_array(body.position_m))
}
pub(super) fn energy(k: &ChassisKinematics) -> f64 {
    k.chassis.kinetic_energy_j
        + k.uprights
            .iter()
            .map(|u| {
                u.body.kinetic_energy_j + u.wheel.as_ref().map_or(0., |w| w.body.kinetic_energy_j)
            })
            .sum::<f64>()
}
impl ChassisSystem {
    pub(super) fn impulse_matrix(&self) -> Result<([[f64; 14]; 14], usize)> {
        let n =
            6 + self.uprights.len() + self.uprights.iter().filter(|u| u.wheel.is_some()).count();
        let mut matrix = [[0.; 14]; 14];
        let rotation = DQuat::from_array(self.chassis.state().orientation_xyzw);
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        for i in 0..3 {
            matrix[i][i] = self.chassis.properties().mass;
            for j in 0..3 {
                matrix[3 + i][3 + j] =
                    axes[i].dot(inertia(self.chassis.properties(), rotation, axes[j]));
            }
        }
        let mut spin = 6 + self.uprights.len();
        for (index, u) in self.uprights.iter().enumerate() {
            let map = u
                .geometry
                .map(u.state.travel_m, u.rack.position_m, Some(u.pose))?;
            let pose = map.pose();
            let frame = rotation * u.frame.rotation;
            let orientation = frame * pose.rotation;
            let column = map.columns()[0];
            let angular = DVec3::from_array(column.angular_rad_per_m);
            let columns = |local: DVec3| {
                let lever =
                    rotation * u.frame.position + frame * pose.position + orientation * local;
                let mut jv = [DVec3::ZERO; 14];
                let mut jw = jv;
                for i in 0..3 {
                    jv[i] = axes[i];
                    jv[3 + i] = axes[i].cross(lever);
                    jw[3 + i] = axes[i];
                }
                jv[6 + index] = frame
                    * (DVec3::from_array(column.origin_per_m)
                        + angular.cross(pose.rotation * local));
                jw[6 + index] = frame * angular;
                (jv, jw)
            };
            let (jv, jw) = columns(DVec3::from_array(u.com));
            mass_contribution(&mut matrix, n, u.properties.mass, &jv, &jw, |v| {
                inertia(&u.properties, orientation, v)
            });
            if let Some(w) = &u.wheel {
                let (jv, mut jw) = columns(w.center);
                jw[spin] = orientation * w.axis;
                spin += 1;
                mass_contribution(&mut matrix, n, w.properties.mass, &jv, &jw, |v| {
                    inertia(&w.properties, orientation, v)
                });
            }
        }
        Ok((matrix, n))
    }
    fn impulse_delta(&self, point: DVec3, impulse: DVec3) -> Result<([f64; 14], usize)> {
        let moment = (point - DVec3::from_array(self.chassis.state().position_m)).cross(impulse);
        vector(moment, 1e18)?;
        let mut rhs = [0.; 14];
        rhs[..3].copy_from_slice(&impulse.to_array());
        rhs[3..6].copy_from_slice(&moment.to_array());
        let (matrix, n) = self.impulse_matrix()?;
        Ok((crate::coupled_linear::solve(matrix, rhs, n)?, n))
    }
    /// Read-only point response for a chassis impulse along a unit world direction.
    /// No forces, gravity, springs, integration or positional repair are evaluated.
    pub fn chassis_point_response(
        &self,
        point_world_m: [f64; 3],
        direction_world: [f64; 3],
    ) -> Result<ChassisPointResponse> {
        let point = vector(DVec3::from_array(point_world_m), 1e7)?;
        let direction = vector(DVec3::from_array(direction_world), 2.)?;
        if (direction.length_squared() - 1.).abs() > 1e-8 {
            return Err(Error::new(
                "physics_impulse_direction",
                "Impulse response requires a unit direction within 1e-8",
            ));
        }
        let (delta, n) = self.impulse_delta(point, direction)?;
        let lever = point - DVec3::from_array(self.chassis.state().position_m);
        let dv = vector(
            DVec3::from_slice(&delta[..3]) + DVec3::from_slice(&delta[3..6]).cross(lever),
            1e18,
        )?;
        let inverse = direction.dot(dv);
        if !inverse.is_finite() || inverse <= 0. {
            return Err(Error::new(
                "physics_impulse_mass",
                "Chassis point response must have positive finite inverse mass",
            ));
        }
        Ok(ChassisPointResponse {
            point_velocity_delta_world_per_ns: dv.to_array(),
            inverse_effective_mass_kg_inv: inverse,
            dof_count: n,
        })
    }
    /// Apply a world impulse (N s) at an absolute point rigidly belonging to the chassis.
    /// The full coupled mass matrix distributes the velocity jump into free travel/spin.
    /// Positions, angles and prescribed rack states do not change. Errors commit nothing.
    pub fn impulse_at_world_point(
        &mut self,
        impulse_world_ns: [f64; 3],
        point_world_m: [f64; 3],
    ) -> Result<ChassisImpulseSample> {
        let impulse = vector(DVec3::from_array(impulse_world_ns), 1e18)?;
        let point = vector(DVec3::from_array(point_world_m), 1e7)?;
        let before = self.kinematics()?;
        if impulse == DVec3::ZERO {
            let kinetic = energy(&before);
            scalar(kinetic, 1e30)?;
            let velocity = point_velocity(&before.chassis, point).to_array();
            return Ok(ChassisImpulseSample {
                point_velocity_before_world_m_s: velocity,
                point_velocity_after_world_m_s: velocity,
                kinetic_energy_before_j: kinetic,
                kinetic_energy_after_j: kinetic,
                external_impulse_work_j: 0.,
                prescribed_rack_impulse_work_j: 0.,
                energy_balance_residual_j: 0.,
                dof_count: 6
                    + self.uprights.len()
                    + self.uprights.iter().filter(|u| u.wheel.is_some()).count(),
            });
        }
        let (delta, n) = self.impulse_delta(point, impulse)?;
        let work = self.apply_impulse_delta(
            &before,
            delta,
            n,
            impulse,
            (point - DVec3::from_array(before.chassis.position_m)).cross(impulse),
        )?;
        Ok(ChassisImpulseSample {
            point_velocity_before_world_m_s: point_velocity(&before.chassis, point).to_array(),
            point_velocity_after_world_m_s: point_velocity(&self.chassis.state(), point).to_array(),
            kinetic_energy_before_j: work.kinetic_energy_before_j,
            kinetic_energy_after_j: work.kinetic_energy_after_j,
            external_impulse_work_j: work.external_impulse_work_j,
            prescribed_rack_impulse_work_j: work.prescribed_rack_impulse_work_j,
            energy_balance_residual_j: work.energy_balance_residual_j,
            dof_count: n,
        })
    }
    pub(super) fn apply_impulse_delta(
        &mut self,
        before: &ChassisKinematics,
        delta: [f64; 14],
        n: usize,
        linear: DVec3,
        moment: DVec3,
    ) -> Result<ChassisImpulseWork> {
        vector(linear, 1e18)?;
        vector(moment, 1e18)?;
        if linear == DVec3::ZERO && moment == DVec3::ZERO {
            let k = energy(before);
            scalar(k, 1e30)?;
            return Ok(ChassisImpulseWork {
                kinetic_energy_before_j: k,
                kinetic_energy_after_j: k,
                external_impulse_work_j: 0.,
                prescribed_rack_impulse_work_j: 0.,
                energy_balance_residual_j: 0.,
                dof_count: n,
            });
        }
        let mut next = self.clone();
        let c = &before.chassis;
        next.chassis = self.chassis.with_velocity_delta(
            DVec3::from_slice(&delta[..3]),
            DVec3::from_slice(&delta[3..6]),
        )?;
        let mut spin = 6 + self.uprights.len();
        for (i, u) in next.uprights.iter_mut().enumerate() {
            u.state.rate_m_s += delta[6 + i];
            u.state.validate()?;
            if let Some(w) = &mut u.wheel {
                w.state.rate_rad_s += delta[spin];
                w.state.validate()?;
                spin += 1;
            }
        }
        let after = next.kinematics()?;
        let mut rack_work = 0.;
        let rotation = DQuat::from_array(c.orientation_xyzw);
        for ((u, b), a) in self
            .uprights
            .iter()
            .zip(&before.uprights)
            .zip(&after.uprights)
        {
            let map = u
                .geometry
                .map(u.state.travel_m, u.rack.position_m, Some(u.pose))?;
            let relative = map.velocity(0., u.rack.velocity_m_s)?;
            let frame = rotation * u.frame.rotation;
            let orientation = frame * map.pose().rotation;
            let affine_w = frame * DVec3::from_array(relative.angular_velocity_rad_s());
            let work = |properties: &MassProperties,
                        local: [f64; 3],
                        b: &State,
                        a: &State|
             -> Result<f64> {
                let affine_v = frame * DVec3::from_array(relative.point_velocity_m_s(local)?);
                let dv = DVec3::from_array(a.velocity_world_m_s)
                    - DVec3::from_array(b.velocity_world_m_s);
                let dw = DVec3::from_array(a.angular_velocity_world_rad_s)
                    - DVec3::from_array(b.angular_velocity_world_rad_s);
                Ok(affine_v.dot(properties.mass * dv)
                    + affine_w.dot(inertia(properties, orientation, dw)))
            };
            rack_work += work(&u.properties, u.com, &b.body, &a.body)?;
            if let (Some(w), Some(bw), Some(aw)) = (&u.wheel, &b.wheel, &a.wheel) {
                rack_work += work(&w.properties, w.center.to_array(), &bw.body, &aw.body)?;
            }
        }
        let k0 = energy(before);
        let k1 = energy(&after);
        let external_work = linear.dot(
            0.5 * (DVec3::from_array(c.velocity_world_m_s)
                + DVec3::from_array(after.chassis.velocity_world_m_s)),
        ) + moment.dot(
            0.5 * (DVec3::from_array(c.angular_velocity_world_rad_s)
                + DVec3::from_array(after.chassis.angular_velocity_world_rad_s)),
        );
        let residual = k1 - k0 - external_work - rack_work;
        for value in [k0, k1, external_work, rack_work, residual] {
            scalar(value, 1e30)?;
        }
        *self = next;
        Ok(ChassisImpulseWork {
            kinetic_energy_before_j: k0,
            kinetic_energy_after_j: k1,
            external_impulse_work_j: external_work,
            prescribed_rack_impulse_work_j: rack_work,
            energy_balance_residual_j: residual,
            dof_count: n,
        })
    }
}
