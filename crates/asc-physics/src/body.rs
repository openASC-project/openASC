use crate::{Error, MassProperties, Result};
use glam::{DQuat, DVec3};
use serde::Serialize;

const MAX_POSITION: f64 = 10_000_000.;
const MAX_SPEED: f64 = 1_000_000.;
const MAX_OMEGA: f64 = 100_000.;
const MAX_LOAD: f64 = 1e18;
fn vector(values: [f64; 3], limit: f64, code: &'static str) -> Result<DVec3> {
    let v = DVec3::from_array(values);
    if !v.is_finite() || v.abs().max_element() > limit {
        return Err(Error::new(
            code,
            "Vector is nonfinite or exceeds the documented numeric range.",
        ));
    }
    Ok(v)
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Wrench {
    force: DVec3,
    torque: DVec3,
}
impl Wrench {
    /// Net force (N) and torque about the centre of mass (N m), both in world space.
    pub fn new(force_world_n: [f64; 3], torque_world_nm: [f64; 3]) -> Result<Self> {
        Ok(Self {
            force: vector(force_world_n, MAX_LOAD, "physics_load")?,
            torque: vector(torque_world_nm, MAX_LOAD, "physics_load")?,
        })
    }
    pub fn force_world_n(&self) -> [f64; 3] {
        self.force.to_array()
    }
    pub fn torque_world_nm(&self) -> [f64; 3] {
        self.torque.to_array()
    }
    /// Accumulate transactionally; a rejected sum leaves the wrench unchanged.
    pub fn add(&mut self, other: Self) -> Result<()> {
        *self = Self::new(
            (self.force + other.force).to_array(),
            (self.torque + other.torque).to_array(),
        )?;
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct Body {
    properties: MassProperties,
    position: DVec3,
    orientation: DQuat,
    linear_momentum: DVec3,
    angular_momentum: DVec3,
}
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct State {
    pub position_m: [f64; 3],
    /// Unit quaternion [x,y,z,w], mapping body axes into world axes.
    pub orientation_xyzw: [f64; 4],
    pub linear_momentum_world_kg_m_s: [f64; 3],
    pub angular_momentum_world_kg_m2_s: [f64; 3],
    pub velocity_world_m_s: [f64; 3],
    pub angular_velocity_world_rad_s: [f64; 3],
    pub kinetic_energy_j: f64,
}
impl Body {
    pub fn new(
        properties: MassProperties,
        position_m: [f64; 3],
        orientation_xyzw: [f64; 4],
        velocity_world_m_s: [f64; 3],
        angular_velocity_world_rad_s: [f64; 3],
    ) -> Result<Self> {
        let position = vector(position_m, MAX_POSITION, "physics_position")?;
        let mut orientation = DQuat::from_array(orientation_xyzw);
        if !orientation.is_finite() || (orientation.length_squared() - 1.).abs() > 1e-6 {
            return Err(Error::new(
                "physics_orientation",
                "Expected a finite unit body-to-world quaternion [x,y,z,w].",
            ));
        }
        orientation = orientation.normalize();
        let velocity = vector(velocity_world_m_s, MAX_SPEED, "physics_velocity")?;
        let omega = vector(
            angular_velocity_world_rad_s,
            MAX_OMEGA,
            "physics_angular_velocity",
        )?;
        let linear_momentum = velocity * properties.mass;
        let angular_momentum =
            orientation * (properties.inertia * (orientation.conjugate() * omega));
        let value = Self {
            properties,
            position,
            orientation,
            linear_momentum,
            angular_momentum,
        };
        value.validate()?;
        Ok(value)
    }
    pub fn properties(&self) -> &MassProperties {
        &self.properties
    }
    fn omega_at(&self, q: DQuat, momentum: DVec3) -> DVec3 {
        q * (self.properties.inverse * (q.conjugate() * momentum))
    }
    fn omega(&self) -> DVec3 {
        self.omega_at(self.orientation, self.angular_momentum)
    }
    fn validate(&self) -> Result<()> {
        vector(self.position.to_array(), MAX_POSITION, "physics_position")?;
        vector(
            (self.linear_momentum / self.properties.mass).to_array(),
            MAX_SPEED,
            "physics_velocity",
        )?;
        vector(
            self.omega().to_array(),
            MAX_OMEGA,
            "physics_angular_velocity",
        )?;
        if !self.linear_momentum.is_finite()
            || !self.angular_momentum.is_finite()
            || !self.orientation.is_finite()
            || (self.orientation.length_squared() - 1.).abs() > 1e-10
        {
            return Err(Error::new(
                "physics_state",
                "Rigid-body state is nonfinite or orientation is not unit length.",
            ));
        }
        Ok(())
    }
    pub fn state(&self) -> State {
        let velocity = self.linear_momentum / self.properties.mass;
        let omega = self.omega();
        State {
            position_m: self.position.to_array(),
            orientation_xyzw: self.orientation.to_array(),
            linear_momentum_world_kg_m_s: self.linear_momentum.to_array(),
            angular_momentum_world_kg_m2_s: self.angular_momentum.to_array(),
            velocity_world_m_s: velocity.to_array(),
            angular_velocity_world_rad_s: omega.to_array(),
            kinetic_energy_j: 0.5
                * (self.linear_momentum.dot(velocity) + self.angular_momentum.dot(omega)),
        }
    }
    /// Transform a body-local point relative to the centre of mass into world space.
    pub fn world_point(&self, point_body_m: [f64; 3]) -> Result<[f64; 3]> {
        let point = vector(point_body_m, MAX_POSITION, "physics_position")?;
        let world = self.position + self.orientation * point;
        Ok(vector(world.to_array(), MAX_POSITION, "physics_position")?.to_array())
    }
    /// Force at an absolute world point; no state mutation or implicit gravity.
    pub fn force_at_world_point(
        &self,
        force_world_n: [f64; 3],
        point_world_m: [f64; 3],
    ) -> Result<Wrench> {
        let force = vector(force_world_n, MAX_LOAD, "physics_load")?;
        let point = vector(point_world_m, MAX_POSITION, "physics_position")?;
        Wrench::new(
            force.to_array(),
            (point - self.position).cross(force).to_array(),
        )
    }
    /// Force and lever point in the body's local frame about its centre of mass.
    pub fn force_at_body_point(
        &self,
        force_body_n: [f64; 3],
        point_body_m: [f64; 3],
    ) -> Result<Wrench> {
        let force = vector(force_body_n, MAX_LOAD, "physics_load")?;
        let point = vector(point_body_m, MAX_POSITION, "physics_position")?;
        Wrench::new(
            (self.orientation * force).to_array(),
            (self.orientation * point.cross(force)).to_array(),
        )
    }
    pub fn point_velocity_world(&self, point_world_m: [f64; 3]) -> Result<[f64; 3]> {
        let point = vector(point_world_m, MAX_POSITION, "physics_position")?;
        let velocity =
            self.linear_momentum / self.properties.mass + self.omega().cross(point - self.position);
        if !velocity.is_finite() {
            return Err(Error::new(
                "physics_velocity",
                "Point velocity is nonfinite.",
            ));
        }
        Ok(velocity.to_array())
    }
    /// Instantaneous impulse in world space at an absolute world point (N s).
    pub fn impulse_at_world_point(
        &mut self,
        impulse_world_ns: [f64; 3],
        point_world_m: [f64; 3],
    ) -> Result<()> {
        let impulse = vector(impulse_world_ns, MAX_LOAD, "physics_impulse")?;
        let point = vector(point_world_m, MAX_POSITION, "physics_position")?;
        let mut next = self.clone();
        next.linear_momentum += impulse;
        next.angular_momentum += (point - self.position).cross(impulse);
        next.validate()?;
        *self = next;
        Ok(())
    }
    /// Hold the supplied world wrench constant for one step. Failures leave state intact.
    /// Momentum update + midpoint translation + explicit Lie-midpoint orientation.
    pub fn step(&mut self, dt_seconds: f64, load: Wrench) -> Result<()> {
        if !dt_seconds.is_finite() || !(1e-6..=1. / 30.).contains(&dt_seconds) {
            return Err(Error::new(
                "physics_step_range",
                "Step duration must be finite within 1e-6..1/30 seconds.",
            ));
        }
        let dt = dt_seconds;
        let half_momentum = self.angular_momentum + load.torque * (0.5 * dt);
        let start_rotation = self.omega() * dt;
        if !start_rotation.is_finite() || start_rotation.length() > 0.25 {
            return Err(Error::new(
                "physics_rotation_step",
                "Rotation exceeds 0.25 radians per step; use a smaller fixed step.",
            ));
        }
        let half_q = (DQuat::from_scaled_axis(start_rotation * 0.5) * self.orientation).normalize();
        let rotation = self.omega_at(half_q, half_momentum) * dt;
        if !rotation.is_finite() || rotation.length() > 0.25 {
            return Err(Error::new(
                "physics_rotation_step",
                "Midpoint rotation exceeds 0.25 radians per step; use a smaller fixed step.",
            ));
        }
        let mut next = self.clone();
        next.position +=
            (self.linear_momentum + load.force * (0.5 * dt)) * (dt / self.properties.mass);
        next.linear_momentum += load.force * dt;
        next.angular_momentum += load.torque * dt;
        next.orientation = (DQuat::from_scaled_axis(rotation) * self.orientation).normalize();
        next.validate()?;
        *self = next;
        Ok(())
    }
}
