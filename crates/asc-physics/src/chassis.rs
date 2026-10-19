//! Floating chassis with 1..4 massive uprights, fixed racks and massless links.
use crate::{
    Body, ConstraintBalance, Error, LinkDifferential, LinkLoad, LinkPose, LinkSpring,
    LinkSpringSample, MassProperties, Result, State, TravelMechanism, TravelState, Wrench,
};
use glam::{DQuat, DVec3};
use serde::Serialize;

fn scalar(value: f64, limit: f64) -> Result<f64> {
    if !value.is_finite() || value.abs() > limit {
        return Err(Error::new(
            "physics_chassis",
            "Nonfinite or out-of-range coupled dynamics value",
        ));
    }
    Ok(value)
}
fn vector(value: DVec3, limit: f64) -> Result<DVec3> {
    for v in value.to_array() {
        scalar(v, limit)?;
    }
    Ok(value)
}
fn inertia(properties: &MassProperties, q: DQuat, v: DVec3) -> DVec3 {
    q * (properties.inertia * (q.conjugate() * v))
}

#[derive(Clone, Debug)]
pub struct MountedUpright {
    geometry: TravelMechanism,
    properties: MassProperties,
    com: [f64; 3],
    rack: [f64; 3],
    frame: LinkPose,
    springs: Vec<LinkSpring>,
    state: TravelState,
    pose: LinkPose,
}
impl MountedUpright {
    /// Frame pose maps source linkage coordinates into chassis-local COM axes.
    pub fn new(
        geometry: TravelMechanism,
        properties: MassProperties,
        center_of_mass_local_m: [f64; 3],
        rack_m: [f64; 3],
        frame_in_chassis: LinkPose,
        state: TravelState,
    ) -> Result<Self> {
        state.validate()?;
        crate::linkage::point(center_of_mass_local_m)?;
        let map = geometry.map(state.travel_m, rack_m, None)?;
        map.velocity(state.rate_m_s, [0.; 3])?;
        Ok(Self {
            geometry,
            properties,
            com: center_of_mass_local_m,
            rack: rack_m,
            frame: frame_in_chassis,
            springs: Vec::new(),
            state,
            pose: map.pose(),
        })
    }
    pub fn add_spring(&mut self, spring: LinkSpring) -> Result<()> {
        if self.springs.len() >= 16 {
            return Err(Error::new(
                "physics_chassis_springs",
                "At most 16 springs per upright",
            ));
        }
        let map = self
            .geometry
            .map(self.state.travel_m, self.rack, Some(self.pose))?;
        spring.sample(&map, self.state.rate_m_s, [0.; 3])?;
        self.springs.push(spring);
        Ok(())
    }
}
#[derive(Debug, Serialize)]
pub struct UprightSample {
    pub state: TravelState,
    pub travel_acceleration_m_s2: f64,
    pub source_position_m: [f64; 3],
    pub source_orientation_xyzw: [f64; 4],
    pub body: State,
    pub acceleration_world_m_s2: [f64; 3],
    pub angular_acceleration_world_rad_s2: [f64; 3],
    /// Relative element velocities/loads in the attached source frame.
    pub relative_springs: Vec<LinkSpringSample>,
    pub constraint_balance_in_source_frame: ConstraintBalance,
    pub support_force_on_chassis_world_n: [f64; 3],
    pub support_torque_about_chassis_com_world_nm: [f64; 3],
    pub spring_force_on_chassis_world_n: [f64; 3],
    pub spring_torque_about_chassis_com_world_nm: [f64; 3],
}
#[derive(Debug, Serialize)]
pub struct ChassisSample {
    pub chassis: State,
    pub acceleration_world_m_s2: [f64; 3],
    pub angular_acceleration_world_rad_s2: [f64; 3],
    pub uprights: Vec<UprightSample>,
    pub total_mass_kg: f64,
    pub center_of_mass_world_m: [f64; 3],
    pub linear_momentum_world_kg_m_s: [f64; 3],
    pub angular_momentum_about_world_origin_kg_m2_s: [f64; 3],
    pub kinetic_energy_j: f64,
    pub potential_energy_j: f64,
    pub dissipated_power_w: f64,
    pub chassis_force_balance_residual_n: [f64; 3],
    pub chassis_torque_balance_residual_nm: [f64; 3],
}
#[derive(Clone, Debug)]
pub struct ChassisSystem {
    chassis: Body,
    uprights: Vec<MountedUpright>,
    gravity: DVec3,
}

struct Prepared {
    map: LinkDifferential,
    frame_rotation: DQuat,
    frame_lever: DVec3,
    orientation: DQuat,
    center: DVec3,
    com_lever: DVec3,
    velocity: DVec3,
    omega: DVec3,
    bias_a: DVec3,
    bias_alpha: DVec3,
    jv: [DVec3; 10],
    jw: [DVec3; 10],
    applied_force: DVec3,
    applied_torque: DVec3,
    springs: Vec<LinkSpringSample>,
}
impl ChassisSystem {
    pub fn new(
        chassis: Body,
        uprights: Vec<MountedUpright>,
        gravity_world_m_s2: [f64; 3],
    ) -> Result<Self> {
        if !(1..=4).contains(&uprights.len()) {
            return Err(Error::new(
                "physics_chassis_count",
                "Expected one to four mounted uprights",
            ));
        }
        let system = Self {
            chassis,
            uprights,
            gravity: vector(DVec3::from_array(gravity_world_m_s2), 1e3)?,
        };
        system.sample(
            Wrench::default(),
            &vec![Wrench::default(); system.uprights.len()],
        )?;
        Ok(system)
    }
    /// External world wrenches are about each body's COM; one per upright.
    pub fn sample(&self, chassis_load: Wrench, upright_loads: &[Wrench]) -> Result<ChassisSample> {
        if upright_loads.len() != self.uprights.len() {
            return Err(Error::new(
                "physics_chassis_loads",
                "Expected exactly one COM wrench per upright",
            ));
        }
        let state = self.chassis.state();
        let position = DVec3::from_array(state.position_m);
        let rotation = DQuat::from_array(state.orientation_xyzw);
        let velocity = DVec3::from_array(state.velocity_world_m_s);
        let omega = DVec3::from_array(state.angular_velocity_world_rad_s);
        let properties = self.chassis.properties();
        let axes = [DVec3::X, DVec3::Y, DVec3::Z];
        let n = 6 + self.uprights.len();
        let mut matrix = [[0.; 10]; 10];
        let mut rhs = [0.; 10];
        let chassis_force =
            DVec3::from_array(chassis_load.force_world_n()) + properties.mass * self.gravity;
        let chassis_torque = DVec3::from_array(chassis_load.torque_world_nm());
        let gyro = omega.cross(inertia(properties, rotation, omega));
        for i in 0..3 {
            matrix[i][i] = properties.mass;
            rhs[i] = chassis_force[i];
            rhs[3 + i] = chassis_torque[i] - gyro[i];
            for j in 0..3 {
                matrix[3 + i][3 + j] = axes[i].dot(inertia(properties, rotation, axes[j]));
            }
        }
        let mut prepared = Vec::with_capacity(self.uprights.len());
        let mut potential = -properties.mass * self.gravity.dot(position);
        let mut dissipated = 0.;
        for (index, (upright, external)) in self.uprights.iter().zip(upright_loads).enumerate() {
            upright.state.validate()?;
            let map =
                upright
                    .geometry
                    .map(upright.state.travel_m, upright.rack, Some(upright.pose))?;
            let pose = map.pose();
            let frame_rotation = rotation * upright.frame.rotation;
            let frame_lever = rotation * upright.frame.position;
            let com_lever = frame_rotation * (pose.rotation * DVec3::from_array(upright.com));
            let lever = frame_lever + frame_rotation * pose.position + com_lever;
            let center = position + lever;
            let orientation = frame_rotation * pose.rotation;
            let relative = map.velocity(upright.state.rate_m_s, [0.; 3])?;
            let relative_v =
                frame_rotation * DVec3::from_array(relative.point_velocity_m_s(upright.com)?);
            let relative_w = frame_rotation * DVec3::from_array(relative.angular_velocity_rad_s());
            let point_v = velocity + omega.cross(lever) + relative_v;
            let point_w = omega + relative_w;
            let bias = upright.geometry.acceleration(
                upright.state.travel_m,
                upright.rack,
                pose,
                upright.state.rate_m_s,
                0.,
            )?;
            let bias_a = omega.cross(omega.cross(lever))
                + 2. * omega.cross(relative_v)
                + frame_rotation * DVec3::from_array(bias.point_acceleration_m_s2(upright.com)?);
            let bias_alpha = omega.cross(relative_w)
                + frame_rotation * DVec3::from_array(bias.angular_acceleration_rad_s2);
            let column = map.columns()[0];
            let angular_gradient = DVec3::from_array(column.angular_rad_per_m);
            let com_gradient = DVec3::from_array(column.origin_per_m)
                + angular_gradient.cross(pose.rotation * DVec3::from_array(upright.com));
            let mut jv = [DVec3::ZERO; 10];
            let mut jw = jv;
            for i in 0..3 {
                jv[i] = axes[i];
                jv[3 + i] = axes[i].cross(lever);
                jw[3 + i] = axes[i];
            }
            jv[6 + index] = frame_rotation * com_gradient;
            jw[6 + index] = frame_rotation * angular_gradient;
            let mass = upright.properties.mass;
            let apply_i = |v| inertia(&upright.properties, orientation, v);
            let applied_force = DVec3::from_array(external.force_world_n()) + mass * self.gravity;
            let applied_com_torque = DVec3::from_array(external.torque_world_nm());
            let bias_torque = apply_i(bias_alpha) + point_w.cross(apply_i(point_w));
            for i in 0..n {
                rhs[i] += jv[i].dot(applied_force - mass * bias_a)
                    + jw[i].dot(applied_com_torque - bias_torque);
                for j in 0..n {
                    matrix[i][j] += mass * jv[i].dot(jv[j]) + jw[i].dot(apply_i(jw[j]));
                }
            }
            let mut applied_origin_torque = applied_com_torque + com_lever.cross(applied_force);
            let mut total_force = applied_force;
            let mut springs = Vec::with_capacity(upright.springs.len());
            for spring in &upright.springs {
                let sample = spring.sample(&map, upright.state.rate_m_s, [0.; 3])?;
                rhs[6 + index] += sample.generalized_loads.travel_n;
                total_force += frame_rotation * DVec3::from_array(sample.upright_load.force_n());
                applied_origin_torque += frame_rotation
                    * DVec3::from_array(sample.upright_load.torque_about_upright_origin_nm());
                potential += sample.potential_energy_j;
                dissipated += sample.dissipated_power_w;
                springs.push(sample);
            }
            potential -= mass * self.gravity.dot(center);
            prepared.push(Prepared {
                map,
                frame_rotation,
                frame_lever,
                orientation,
                center,
                com_lever,
                velocity: point_v,
                omega: point_w,
                bias_a,
                bias_alpha,
                jv,
                jw,
                applied_force: total_force,
                applied_torque: applied_origin_torque,
                springs,
            });
        }
        for value in &rhs[..n] {
            scalar(*value, 1e18)?;
        }
        let accelerations = crate::coupled_linear::solve(matrix, rhs, n)?;
        for value in &accelerations[..n] {
            scalar(*value, 1e6)?;
        }
        let chassis_a = DVec3::from_slice(&accelerations[..3]);
        let chassis_alpha = DVec3::from_slice(&accelerations[3..6]);
        let mut total_mass = properties.mass;
        let mut total_center = position * properties.mass;
        let mut momentum = DVec3::from_array(state.linear_momentum_world_kg_m_s);
        let mut angular_momentum =
            position.cross(momentum) + DVec3::from_array(state.angular_momentum_world_kg_m2_s);
        let mut kinetic = state.kinetic_energy_j;
        let mut force_on_chassis = chassis_force;
        let mut torque_on_chassis = chassis_torque;
        let mut samples = Vec::with_capacity(prepared.len());
        for (index, (upright, p)) in self.uprights.iter().zip(prepared).enumerate() {
            let mut a = p.bias_a;
            let mut alpha = p.bias_alpha;
            for (i, acceleration) in accelerations.iter().enumerate().take(n) {
                a += p.jv[i] * *acceleration;
                alpha += p.jw[i] * *acceleration;
            }
            vector(a, 1e13)?;
            vector(alpha, 1e9)?;
            let body = Body::new(
                upright.properties.clone(),
                p.center.to_array(),
                p.orientation.to_array(),
                p.velocity.to_array(),
                p.omega.to_array(),
            )?
            .state();
            let inertial_force = upright.properties.mass * a;
            let inertial_torque = inertia(&upright.properties, p.orientation, alpha)
                + p.omega
                    .cross(inertia(&upright.properties, p.orientation, p.omega))
                + p.com_lever.cross(inertial_force);
            let load = LinkLoad::new(
                (p.frame_rotation.conjugate() * (p.applied_force - inertial_force)).to_array(),
                (p.frame_rotation.conjugate() * (p.applied_torque - inertial_torque)).to_array(),
            )?;
            let balance = p.map.hold_load(load)?;
            let scale = p
                .applied_force
                .abs()
                .max_element()
                .max(inertial_force.abs().max_element())
                .max(inertial_torque.abs().max_element())
                .max(1.);
            if balance.travel_actuator_n.abs() > 1e-7 * scale {
                return Err(Error::new(
                    "physics_chassis_balance",
                    "Coupled upright requires a spurious travel holding load",
                ));
            }
            let support_force =
                p.frame_rotation * DVec3::from_array(balance.support_force_on_frame_n);
            let support_torque = p.frame_rotation
                * DVec3::from_array(balance.support_torque_about_frame_origin_nm)
                + p.frame_lever.cross(support_force);
            let mut spring_force = DVec3::ZERO;
            let mut spring_torque = DVec3::ZERO;
            for spring in &p.springs {
                let f = p.frame_rotation * DVec3::from_array(spring.force_on_frame_n);
                spring_force += f;
                spring_torque += p.frame_rotation
                    * DVec3::from_array(spring.torque_about_frame_origin_nm)
                    + p.frame_lever.cross(f);
            }
            force_on_chassis += support_force + spring_force;
            torque_on_chassis += support_torque + spring_torque;
            total_mass += upright.properties.mass;
            total_center += p.center * upright.properties.mass;
            momentum += DVec3::from_array(body.linear_momentum_world_kg_m_s);
            angular_momentum += p
                .center
                .cross(DVec3::from_array(body.linear_momentum_world_kg_m_s))
                + DVec3::from_array(body.angular_momentum_world_kg_m2_s);
            kinetic += body.kinetic_energy_j;
            let pose = p.map.pose();
            samples.push(UprightSample {
                state: upright.state,
                travel_acceleration_m_s2: accelerations[6 + index],
                source_position_m: pose.position_m(),
                source_orientation_xyzw: pose.orientation_xyzw(),
                body,
                acceleration_world_m_s2: a.to_array(),
                angular_acceleration_world_rad_s2: alpha.to_array(),
                relative_springs: p.springs,
                constraint_balance_in_source_frame: balance,
                support_force_on_chassis_world_n: support_force.to_array(),
                support_torque_about_chassis_com_world_nm: support_torque.to_array(),
                spring_force_on_chassis_world_n: spring_force.to_array(),
                spring_torque_about_chassis_com_world_nm: spring_torque.to_array(),
            });
        }
        let force_error = force_on_chassis - properties.mass * chassis_a;
        let torque_error = torque_on_chassis - inertia(properties, rotation, chassis_alpha) - gyro;
        let scale = force_on_chassis
            .abs()
            .max_element()
            .max(torque_on_chassis.abs().max_element())
            .max(1.);
        if force_error.abs().max_element() > 1e-7 * scale
            || torque_error.abs().max_element() > 1e-7 * scale
        {
            return Err(Error::new(
                "physics_chassis_balance",
                "Chassis force/moment balance residual exceeds tolerance",
            ));
        }
        scalar(kinetic, 1e30)?;
        scalar(potential, 1e30)?;
        scalar(dissipated, 1e30)?;
        vector(momentum, 1e24)?;
        vector(angular_momentum, 1e24)?;
        Ok(ChassisSample {
            chassis: state,
            acceleration_world_m_s2: chassis_a.to_array(),
            angular_acceleration_world_rad_s2: chassis_alpha.to_array(),
            uprights: samples,
            total_mass_kg: total_mass,
            center_of_mass_world_m: (total_center / total_mass).to_array(),
            linear_momentum_world_kg_m_s: momentum.to_array(),
            angular_momentum_about_world_origin_kg_m2_s: angular_momentum.to_array(),
            kinetic_energy_j: kinetic,
            potential_energy_j: potential,
            dissipated_power_w: dissipated,
            chassis_force_balance_residual_n: force_error.to_array(),
            chassis_torque_balance_residual_nm: torque_error.to_array(),
        })
    }
    fn advance(&self, slope: &ChassisSample, dt: f64) -> Result<Self> {
        let initial = self.chassis.state();
        let rotation = DVec3::from_array(slope.chassis.angular_velocity_world_rad_s) * dt;
        if !rotation.is_finite() || rotation.length() > 0.25 {
            return Err(Error::new(
                "physics_chassis_rotation",
                "Rotation increment exceeds 0.25 rad; use a smaller step",
            ));
        }
        let q = (DQuat::from_scaled_axis(rotation) * DQuat::from_array(initial.orientation_xyzw))
            .normalize();
        let mut candidate = self.clone();
        candidate.chassis = Body::new(
            self.chassis.properties().clone(),
            (DVec3::from_array(initial.position_m)
                + dt * DVec3::from_array(slope.chassis.velocity_world_m_s))
            .to_array(),
            q.to_array(),
            (DVec3::from_array(initial.velocity_world_m_s)
                + dt * DVec3::from_array(slope.acceleration_world_m_s2))
            .to_array(),
            (DVec3::from_array(initial.angular_velocity_world_rad_s)
                + dt * DVec3::from_array(slope.angular_acceleration_world_rad_s2))
            .to_array(),
        )?;
        for ((next, base), slope) in candidate
            .uprights
            .iter_mut()
            .zip(&self.uprights)
            .zip(&slope.uprights)
        {
            next.state = TravelState {
                travel_m: base.state.travel_m + dt * slope.state.rate_m_s,
                rate_m_s: base.state.rate_m_s + dt * slope.travel_acceleration_m_s2,
            };
            next.state.validate()?;
            next.pose = LinkPose::new(slope.source_position_m, slope.source_orientation_xyzw)?;
        }
        Ok(candidate)
    }
    /// World COM loads are held constant through one explicit Lie-midpoint step.
    /// Every stage and every corner must validate before the whole system commits.
    pub fn step(
        &mut self,
        dt_s: f64,
        chassis_load: Wrench,
        upright_loads: &[Wrench],
    ) -> Result<ChassisSample> {
        if !dt_s.is_finite() || !(1e-6..=1. / 30.).contains(&dt_s) {
            return Err(Error::new(
                "physics_chassis_step",
                "Step must lie within 1e-6..1/30 seconds",
            ));
        }
        let first = self.sample(chassis_load, upright_loads)?;
        let middle = self.advance(&first, 0.5 * dt_s)?;
        let midpoint = middle.sample(chassis_load, upright_loads)?;
        let mut final_system = self.advance(&midpoint, dt_s)?;
        let final_sample = final_system.sample(chassis_load, upright_loads)?;
        for (upright, sample) in final_system.uprights.iter_mut().zip(&final_sample.uprights) {
            upright.pose = LinkPose::new(sample.source_position_m, sample.source_orientation_xyzw)?;
        }
        *self = final_system;
        Ok(final_sample)
    }
}
