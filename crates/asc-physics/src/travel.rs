//! One dynamic travel coordinate on a fixed frame/rack, with massless ideal links.
use crate::{
    ConstraintBalance, Error, FiveLink, LinkDifferential, LinkLoad, LinkPose, LinkSpring,
    LinkSpringSample, MassProperties, RelativeAcceleration, Result, StrutLink,
};
use glam::DVec3;
use serde::Serialize;

fn scalar(value: f64, limit: f64) -> Result<f64> {
    if !value.is_finite() || value.abs() > limit {
        return Err(Error::new(
            "physics_travel",
            "Nonfinite or out-of-range travel dynamics value",
        ));
    }
    Ok(value)
}
#[derive(Clone, Debug)]
pub enum TravelMechanism {
    Dwb(FiveLink),
    Strut(StrutLink),
}
impl TravelMechanism {
    fn map(&self, h: f64, rack: [f64; 3], seed: Option<LinkPose>) -> Result<LinkDifferential> {
        match self {
            Self::Dwb(m) => m.differential(h, rack, m.solve(h, rack, seed)?.pose()?),
            Self::Strut(m) => m.differential(h, rack, m.solve(h, rack, seed)?.pose()?),
        }
    }
    fn acceleration(
        &self,
        h: f64,
        rack: [f64; 3],
        pose: LinkPose,
        rate: f64,
        a: f64,
    ) -> Result<RelativeAcceleration> {
        match self {
            Self::Dwb(m) => m.acceleration(h, rack, pose, [rate, 0., 0., 0.], [a, 0., 0., 0.]),
            Self::Strut(m) => m.acceleration(h, rack, pose, [rate, 0., 0., 0.], [a, 0., 0., 0.]),
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct TravelState {
    pub travel_m: f64,
    pub rate_m_s: f64,
}
impl TravelState {
    fn validate(self) -> Result<()> {
        scalar(self.travel_m, 0.25)?;
        scalar(self.rate_m_s, 1e3)?;
        Ok(())
    }
}
#[derive(Debug, Serialize)]
pub struct TravelSample {
    pub state: TravelState,
    pub acceleration_m_s2: f64,
    pub effective_mass_kg: f64,
    pub velocity_bias_n: f64,
    pub generalized_load_n: f64,
    pub upright_position_m: [f64; 3],
    pub upright_orientation_xyzw: [f64; 4],
    pub center_of_mass_position_m: [f64; 3],
    pub center_of_mass_velocity_m_s: [f64; 3],
    pub center_of_mass_acceleration_m_s2: [f64; 3],
    pub angular_velocity_rad_s: [f64; 3],
    pub angular_acceleration_rad_s2: [f64; 3],
    pub kinetic_energy_j: f64,
    pub potential_energy_j: f64,
    pub dissipated_power_w: f64,
    pub springs: Vec<LinkSpringSample>,
    /// Balances applied-minus-inertial loads. Travel actuator is a numerical
    /// residual only; it must be near zero for this free travel coordinate.
    pub constraint_balance: ConstraintBalance,
}
#[derive(Clone, Debug)]
pub struct TravelSystem {
    geometry: TravelMechanism,
    properties: MassProperties,
    com: [f64; 3],
    rack: [f64; 3],
    gravity: DVec3,
    springs: Vec<LinkSpring>,
    state: TravelState,
    pose: LinkPose,
}
impl TravelSystem {
    /// Explicit upright COM/tensor; gravity is in the fixed linkage frame.
    /// Rack displacement remains fixed throughout all subsequent steps.
    pub fn new(
        geometry: TravelMechanism,
        properties: MassProperties,
        center_of_mass_local_m: [f64; 3],
        rack_m: [f64; 3],
        gravity_m_s2: [f64; 3],
        state: TravelState,
    ) -> Result<Self> {
        state.validate()?;
        crate::linkage::point(center_of_mass_local_m)?;
        for value in gravity_m_s2 {
            scalar(value, 1e3)?;
        }
        let pose = geometry.map(state.travel_m, rack_m, None)?.pose();
        let system = Self {
            geometry,
            properties,
            com: center_of_mass_local_m,
            rack: rack_m,
            gravity: DVec3::from_array(gravity_m_s2),
            springs: Vec::new(),
            state,
            pose,
        };
        system.sample(LinkLoad::new([0.; 3], [0.; 3])?)?;
        Ok(system)
    }
    pub fn state(&self) -> TravelState {
        self.state
    }
    pub fn add_spring(&mut self, spring: LinkSpring) -> Result<()> {
        if self.springs.len() >= 16 {
            return Err(Error::new(
                "physics_travel_springs",
                "At most 16 explicit springs per upright",
            ));
        }
        self.springs.push(spring);
        if let Err(error) = self.sample(LinkLoad::new([0.; 3], [0.; 3])?) {
            self.springs.pop();
            return Err(error);
        }
        Ok(())
    }
    pub fn sample(&self, external: LinkLoad) -> Result<TravelSample> {
        self.evaluate(self.state, self.pose, external)
    }
    fn evaluate(
        &self,
        state: TravelState,
        seed: LinkPose,
        external: LinkLoad,
    ) -> Result<TravelSample> {
        state.validate()?;
        let map = self.geometry.map(state.travel_m, self.rack, Some(seed))?;
        let pose = map.pose();
        let relative = map.velocity(state.rate_m_s, [0.; 3])?;
        let omega = DVec3::from_array(relative.angular_velocity_rad_s());
        let lever = pose.rotation * DVec3::from_array(self.com);
        let com = pose.position + lever;
        let com_velocity = DVec3::from_array(relative.point_velocity_m_s(self.com)?);
        let column = map.columns()[0];
        let angular_gradient = DVec3::from_array(column.angular_rad_per_m);
        let com_gradient = DVec3::from_array(column.origin_per_m) + angular_gradient.cross(lever);
        let inertia =
            |v: DVec3| pose.rotation * (self.properties.inertia * (pose.rotation.conjugate() * v));
        let effective_mass = self.properties.mass * com_gradient.length_squared()
            + angular_gradient.dot(inertia(angular_gradient));
        scalar(effective_mass, 1e24)?;
        if effective_mass < 1e-12 {
            return Err(Error::new(
                "physics_travel_mass",
                "Degenerate effective travel mass",
            ));
        }
        let bias =
            self.geometry
                .acceleration(state.travel_m, self.rack, pose, state.rate_m_s, 0.)?;
        let bias_com = DVec3::from_array(bias.point_acceleration_m_s2(self.com)?);
        let bias_alpha = DVec3::from_array(bias.angular_acceleration_rad_s2);
        let velocity_bias = self.properties.mass * com_gradient.dot(bias_com)
            + angular_gradient.dot(inertia(bias_alpha) + omega.cross(inertia(omega)));
        scalar(velocity_bias, 1e18)?;
        let mut total = external;
        total.add(LinkLoad::at_point(
            (self.properties.mass * self.gravity).to_array(),
            self.com,
            pose,
        )?)?;
        let mut springs = Vec::with_capacity(self.springs.len());
        let mut potential = -self.properties.mass * self.gravity.dot(com);
        let mut dissipated = 0.;
        for spring in &self.springs {
            let sample = spring.sample(&map, state.rate_m_s, [0.; 3])?;
            total.add(sample.upright_load)?;
            potential += sample.potential_energy_j;
            dissipated += sample.dissipated_power_w;
            springs.push(sample);
        }
        let generalized = map.project_load(total)?.travel_n;
        let acceleration = scalar((generalized - velocity_bias) / effective_mass, 1e6)?;
        let actual = self.geometry.acceleration(
            state.travel_m,
            self.rack,
            pose,
            state.rate_m_s,
            acceleration,
        )?;
        let com_acceleration = DVec3::from_array(actual.point_acceleration_m_s2(self.com)?);
        let alpha = DVec3::from_array(actual.angular_acceleration_rad_s2);
        let inertial_force = self.properties.mass * com_acceleration;
        let inertial_torque =
            inertia(alpha) + omega.cross(inertia(omega)) + lever.cross(inertial_force);
        let mut applied_minus_inertia = total;
        applied_minus_inertia.add(LinkLoad::new(
            (-inertial_force).to_array(),
            (-inertial_torque).to_array(),
        )?)?;
        let balance = map.hold_load(applied_minus_inertia)?;
        let scale = DVec3::from_array(total.force_n())
            .abs()
            .max_element()
            .max(inertial_force.abs().max_element())
            .max(inertial_torque.abs().max_element())
            .max(1.);
        if balance.travel_actuator_n.abs() > 1e-7 * scale {
            return Err(Error::new(
                "physics_travel_balance",
                "Free travel coordinate requires a spurious holding load",
            ));
        }
        let kinetic = 0.5
            * (self.properties.mass * com_velocity.length_squared() + omega.dot(inertia(omega)));
        scalar(kinetic, 1e30)?;
        scalar(potential, 1e30)?;
        scalar(dissipated, 1e30)?;
        Ok(TravelSample {
            state,
            acceleration_m_s2: acceleration,
            effective_mass_kg: effective_mass,
            velocity_bias_n: velocity_bias,
            generalized_load_n: generalized,
            upright_position_m: pose.position_m(),
            upright_orientation_xyzw: pose.orientation_xyzw(),
            center_of_mass_position_m: com.to_array(),
            center_of_mass_velocity_m_s: com_velocity.to_array(),
            center_of_mass_acceleration_m_s2: com_acceleration.to_array(),
            angular_velocity_rad_s: omega.to_array(),
            angular_acceleration_rad_s2: alpha.to_array(),
            kinetic_energy_j: kinetic,
            potential_energy_j: potential,
            dissipated_power_w: dissipated,
            springs,
            constraint_balance: balance,
        })
    }
    /// Explicit midpoint, returning the final-state sample. All stages must
    /// solve on the local branch before state commits. External load components
    /// are constant in the fixed frame, with torque about each moving origin.
    pub fn step(&mut self, dt_s: f64, external: LinkLoad) -> Result<TravelSample> {
        if !dt_s.is_finite() || !(1e-6..=1. / 30.).contains(&dt_s) {
            return Err(Error::new(
                "physics_travel_step",
                "Travel step must lie within 1e-6..1/30 seconds",
            ));
        }
        let first = self.sample(external)?;
        let middle = TravelState {
            travel_m: self.state.travel_m + 0.5 * dt_s * self.state.rate_m_s,
            rate_m_s: self.state.rate_m_s + 0.5 * dt_s * first.acceleration_m_s2,
        };
        let midpoint = self.evaluate(middle, self.pose, external)?;
        let final_state = TravelState {
            travel_m: self.state.travel_m + dt_s * middle.rate_m_s,
            rate_m_s: self.state.rate_m_s + dt_s * midpoint.acceleration_m_s2,
        };
        let seed = LinkPose::new(
            midpoint.upright_position_m,
            midpoint.upright_orientation_xyzw,
        )?;
        let sample = self.evaluate(final_state, seed, external)?;
        let pose = LinkPose::new(sample.upright_position_m, sample.upright_orientation_xyzw)?;
        self.state = final_state;
        self.pose = pose;
        Ok(sample)
    }
}
