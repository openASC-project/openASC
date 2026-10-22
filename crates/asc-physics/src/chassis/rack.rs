//! Explicit massless rack prescription, including velocity, acceleration and work.
use super::*;
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct RackMotion {
    pub position_m: [f64; 3],
    pub velocity_m_s: [f64; 3],
    pub acceleration_m_s2: [f64; 3],
}
impl RackMotion {
    pub fn new(
        position_m: [f64; 3],
        velocity_m_s: [f64; 3],
        acceleration_m_s2: [f64; 3],
    ) -> Result<Self> {
        let result = Self {
            position_m,
            velocity_m_s,
            acceleration_m_s2,
        };
        result.validate()?;
        Ok(result)
    }
    pub fn fixed(position_m: [f64; 3]) -> Result<Self> {
        Self::new(position_m, [0.; 3], [0.; 3])
    }
    fn validate(self) -> Result<()> {
        for (values, limit) in [
            (self.position_m, 0.25),
            (self.velocity_m_s, 1e3),
            (self.acceleration_m_s2, 1e6),
        ] {
            if values.iter().any(|v| !v.is_finite() || v.abs() > limit) {
                return Err(Error::new(
                    "physics_rack_motion",
                    "Rack components require finite position within +/-0.25 m, velocity +/-1000 m/s and acceleration +/-1e6 m/s^2",
                ));
            }
        }
        Ok(())
    }
}
impl MountedUpright {
    /// Set explicit initial or diagnostic prescription, never integrate a rack position implicitly.
    pub fn set_rack_motion(&mut self, motion: RackMotion) -> Result<()> {
        motion.validate()?;
        let map = self
            .geometry
            .map(self.state.travel_m, motion.position_m, Some(self.pose))?;
        map.velocity(self.state.rate_m_s, motion.velocity_m_s)?;
        for spring in &self.springs {
            spring.sample(&map, self.state.rate_m_s, motion.velocity_m_s)?;
        }
        self.rack = motion;
        self.pose = map.pose();
        Ok(())
    }
}
impl ChassisSystem {
    pub fn rack_motions(&self) -> Vec<RackMotion> {
        self.uprights.iter().map(|u| u.rack).collect()
    }
    fn assign_racks(&mut self, racks: &[RackMotion]) -> Result<()> {
        if racks.len() != self.uprights.len() {
            return Err(Error::new(
                "physics_rack_count",
                "Expected one rack motion per upright",
            ));
        }
        for (u, r) in self.uprights.iter_mut().zip(racks) {
            u.set_rack_motion(*r)?;
        }
        Ok(())
    }
    pub(super) fn require_fixed_racks(&self) -> Result<()> {
        if self
            .uprights
            .iter()
            .any(|u| u.rack.velocity_m_s != [0.; 3] || u.rack.acceleration_m_s2 != [0.; 3])
        {
            return Err(Error::new(
                "physics_rack_provider",
                "Moving racks require step_with_racks_and_forces",
            ));
        }
        Ok(())
    }
    /// Instantaneous diagnostic prescription; does not mutate the system.
    pub fn sample_with_racks_and_forces(
        &self,
        racks: &[RackMotion],
        forces: impl FnMut(f64, &ChassisKinematics) -> Result<ChassisLoads>,
    ) -> Result<ChassisSample> {
        let mut stage = self.clone();
        stage.assign_racks(racks)?;
        stage.sample_with_forces(forces)
    }
    /// Rack callbacks are sampled before force snapshots at 0, dt/2 and dt.
    /// Start position/velocity must match the committed state; acceleration may change.
    /// All mechanical and rack state commits atomically after final validation.
    pub fn step_with_racks_and_forces(
        &mut self,
        dt_s: f64,
        mut racks: impl FnMut(f64) -> Result<Vec<RackMotion>>,
        mut forces: impl FnMut(f64, &ChassisKinematics) -> Result<ChassisLoads>,
    ) -> Result<ChassisSample> {
        self.integrate(dt_s, |stage, offset| {
            let prescribed = racks(offset).map_err(|e| {
                Error::new(
                    e.code,
                    &format!("Rack callback at +{offset} s: {}", e.message),
                )
            })?;
            if offset == 0. && prescribed.len() == stage.uprights.len() {
                for (u, r) in stage.uprights.iter().zip(&prescribed) {
                    if (0..3).any(|i| {
                        (u.rack.position_m[i] - r.position_m[i]).abs() > 1e-10
                            || (u.rack.velocity_m_s[i] - r.velocity_m_s[i]).abs() > 1e-8
                    }) {
                        return Err(Error::new(
                            "physics_rack_discontinuity",
                            "Rack start position/velocity must match committed state",
                        ));
                    }
                }
            }
            stage.assign_racks(&prescribed)?;
            let state = stage.kinematics()?;
            let loads = forces(offset, &state).map_err(|e| {
                Error::new(
                    e.code,
                    &format!("Force callback at +{offset} s: {}", e.message),
                )
            })?;
            stage.sample_with_wheels(loads.chassis, &loads.uprights, &loads.wheels)
        })
    }
}
