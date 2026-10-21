//! Read-only force-independent stage snapshots for state-dependent force models.
use super::*;

#[derive(Debug, Serialize)]
pub struct WheelKinematics {
    pub state: WheelState,
    pub axis_world: [f64; 3],
    pub body: State,
}
#[derive(Debug, Serialize)]
pub struct UprightKinematics {
    pub state: TravelState,
    pub source_position_m: [f64; 3],
    pub source_orientation_xyzw: [f64; 4],
    pub body: State,
    pub wheel: Option<WheelKinematics>,
}
#[derive(Debug, Serialize)]
pub struct ChassisKinematics {
    pub chassis: State,
    pub uprights: Vec<UprightKinematics>,
}
#[derive(Clone, Debug)]
pub struct ChassisLoads {
    pub chassis: Wrench,
    pub uprights: Vec<Wrench>,
    pub wheels: Vec<WheelLoad>,
}
impl ChassisLoads {
    pub fn zero(upright_count: usize) -> Result<Self> {
        if !(1..=4).contains(&upright_count) {
            return Err(Error::new(
                "physics_chassis_loads",
                "Expected one to four upright slots",
            ));
        }
        Ok(Self {
            chassis: Wrench::default(),
            uprights: vec![Wrench::default(); upright_count],
            wheels: vec![WheelLoad::default(); upright_count],
        })
    }
}
impl ChassisSystem {
    /// Validate initialization under the actual applied forces, including
    /// support that balances preload, rather than an artificial unloaded state.
    pub fn new_with_forces(
        chassis: Body,
        uprights: Vec<MountedUpright>,
        gravity_world_m_s2: [f64; 3],
        forces: impl FnMut(f64, &ChassisKinematics) -> Result<ChassisLoads>,
    ) -> Result<Self> {
        let system = Self::configured(chassis, uprights, gravity_world_m_s2)?;
        system.sample_with_forces(forces)?;
        Ok(system)
    }
    /// No forces, accelerations, matrix solve or state mutation is needed here.
    pub fn kinematics(&self) -> Result<ChassisKinematics> {
        let chassis = self.chassis.state();
        let rotation = DQuat::from_array(chassis.orientation_xyzw);
        let position = DVec3::from_array(chassis.position_m);
        let velocity = DVec3::from_array(chassis.velocity_world_m_s);
        let omega = DVec3::from_array(chassis.angular_velocity_world_rad_s);
        let mut uprights = Vec::with_capacity(self.uprights.len());
        for upright in &self.uprights {
            let map =
                upright
                    .geometry
                    .map(upright.state.travel_m, upright.rack, Some(upright.pose))?;
            let pose = map.pose();
            let frame_rotation = rotation * upright.frame.rotation;
            let frame_lever = rotation * upright.frame.position;
            let orientation = frame_rotation * pose.rotation;
            let relative = map.velocity(upright.state.rate_m_s, [0.; 3])?;
            let up_omega =
                omega + frame_rotation * DVec3::from_array(relative.angular_velocity_rad_s());
            let point = |local: [f64; 3]| -> Result<(DVec3, DVec3)> {
                let lever = frame_lever
                    + frame_rotation * pose.position
                    + orientation * DVec3::from_array(local);
                Ok((
                    position + lever,
                    velocity
                        + omega.cross(lever)
                        + frame_rotation * DVec3::from_array(relative.point_velocity_m_s(local)?),
                ))
            };
            let (center, v) = point(upright.com)?;
            let body = Body::new(
                upright.properties.clone(),
                center.to_array(),
                orientation.to_array(),
                v.to_array(),
                up_omega.to_array(),
            )?
            .state();
            let wheel = if let Some(wheel) = &upright.wheel {
                wheel.state.validate()?;
                let axis = orientation * wheel.axis;
                let (center, v) = point(wheel.center.to_array())?;
                let spin = DQuat::from_axis_angle(
                    wheel.axis,
                    wheel.state.angle_rad.rem_euclid(std::f64::consts::TAU),
                );
                let body = Body::new(
                    wheel.properties.clone(),
                    center.to_array(),
                    (orientation * spin).to_array(),
                    v.to_array(),
                    (up_omega + axis * wheel.state.rate_rad_s).to_array(),
                )?
                .state();
                Some(WheelKinematics {
                    state: wheel.state,
                    axis_world: axis.to_array(),
                    body,
                })
            } else {
                None
            };
            uprights.push(UprightKinematics {
                state: upright.state,
                source_position_m: pose.position_m(),
                source_orientation_xyzw: pose.orientation_xyzw(),
                body,
                wheel,
            });
        }
        Ok(ChassisKinematics { chassis, uprights })
    }
    pub fn sample_with_forces(
        &self,
        mut forces: impl FnMut(f64, &ChassisKinematics) -> Result<ChassisLoads>,
    ) -> Result<ChassisSample> {
        let state = self.kinematics()?;
        let loads = forces(0., &state)?;
        self.sample_with_wheels(loads.chassis, &loads.uprights, &loads.wheels)
    }
    /// Callback offsets are 0, dt/2 and dt. Final forces validate/report the
    /// final state; only the midpoint derivative advances it. Callback-owned
    /// side effects are outside the system's transactional state guarantee.
    pub fn step_with_forces(
        &mut self,
        dt_s: f64,
        mut forces: impl FnMut(f64, &ChassisKinematics) -> Result<ChassisLoads>,
    ) -> Result<ChassisSample> {
        self.integrate(dt_s, |system, offset| {
            let state = system.kinematics()?;
            let loads = forces(offset, &state).map_err(|error| {
                Error::new(
                    error.code,
                    &format!("Force callback at +{offset} s: {}", error.message),
                )
            })?;
            system.sample_with_wheels(loads.chassis, &loads.uprights, &loads.wheels)
        })
    }
}
