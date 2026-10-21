//! Bounded passive regularized disc torque between wheel and upright.
//! No static lock, ABS, heat model or automatic original brake-data mapping.
use crate::{Error, Result};
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct WheelBrake {
    maximum_torque: f64,
    transition_rate: f64,
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct WheelBrakeSample {
    pub input: f64,
    pub relative_rate_rad_s: f64,
    pub torque_on_wheel_nm: f64,
    pub torque_on_upright_nm: f64,
    pub torque_limit_nm: f64,
    pub mechanical_power_w: f64,
    pub dissipated_power_w: f64,
}
fn fail() -> Error {
    Error::new(
        "physics_wheel_brake",
        "Expected finite maximum torque in 0..1e6 N m, transition rate in 1e-3..1e3 rad/s, wheel rate within +/-1e5 rad/s and input in 0..1",
    )
}
impl WheelBrake {
    pub fn new(maximum_torque_nm: f64, transition_rate_rad_s: f64) -> Result<Self> {
        if !maximum_torque_nm.is_finite()
            || !(0. ..=1e6).contains(&maximum_torque_nm)
            || !transition_rate_rad_s.is_finite()
            || !(1e-3..=1e3).contains(&transition_rate_rad_s)
        {
            return Err(fail());
        }
        Ok(Self {
            maximum_torque: maximum_torque_nm,
            transition_rate: transition_rate_rad_s,
        })
    }
    /// Apply through WheelLoad's INTERNAL torque, so the upright receives the
    /// opposite reaction. Relative spin, not absolute world angular speed, sets power.
    pub fn sample(&self, relative_rate_rad_s: f64, input: f64) -> Result<WheelBrakeSample> {
        if !relative_rate_rad_s.is_finite()
            || relative_rate_rad_s.abs() > 1e5
            || !input.is_finite()
            || !(0. ..=1.).contains(&input)
        {
            return Err(fail());
        }
        let limit = self.maximum_torque * input;
        let torque = -limit * (relative_rate_rad_s / self.transition_rate).clamp(-1., 1.);
        let power = torque * relative_rate_rad_s;
        Ok(WheelBrakeSample {
            input,
            relative_rate_rad_s,
            torque_on_wheel_nm: torque,
            torque_on_upright_nm: -torque,
            torque_limit_nm: limit,
            mechanical_power_w: power,
            dissipated_power_w: -power,
        })
    }
}
