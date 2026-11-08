//! Engine torque curve sampling with RPM tracking and gear ratio transmission.
use super::*;
use serde::Serialize;

fn fail(message: &str) -> Error {
    Error::new("physics_engine", message)
}

#[derive(Clone, Debug)]
pub struct TorqueCurve {
    points: Vec<[f64; 2]>,
}

impl TorqueCurve {
    pub fn new(points: &[[f64; 2]]) -> Result<Self> {
        if points.len() < 2 {
            return Err(fail("Torque curve requires at least two points"));
        }
        for pair in points.windows(2) {
            if pair[1][0] <= pair[0][0] {
                return Err(fail("Torque curve RPM values must be strictly increasing"));
            }
        }
        if points[0][0] < 0. {
            return Err(fail("Torque curve RPM must be non-negative"));
        }
        for p in points {
            if !p[0].is_finite() || !p[1].is_finite() {
                return Err(fail("Torque curve values must be finite"));
            }
        }
        Ok(Self {
            points: points.to_vec(),
        })
    }

    pub fn sample(&self, rpm: f64) -> Result<f64> {
        if !rpm.is_finite() {
            return Err(fail("RPM must be finite"));
        }
        let upper = self.points.partition_point(|p| p[0] <= rpm);
        if upper == 0 {
            return Ok(self.points[0][1]);
        }
        if upper == self.points.len() {
            return Ok(self.points[upper - 1][1]);
        }
        let [x0, y0] = self.points[upper - 1];
        let [x1, y1] = self.points[upper];
        let t = (rpm - x0) / (x1 - x0);
        Ok((1. - t) * y0 + t * y1)
    }

    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
}

#[derive(Clone, Debug)]
pub struct EngineDrivetrain {
    torque_curve: TorqueCurve,
    idle_rpm: f64,
    limiter_rpm: f64,
    inertia_kg_m2: f64,
    forward_ratios: Vec<f64>,
    reverse_ratio: f64,
    final_ratio: f64,
    traction: Traction,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Traction {
    Rwd,
    Fwd,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct EngineSample {
    pub rpm: f64,
    pub torque_nm: f64,
    pub gear: i32,
    pub ratio: f64,
    pub wheel_torque_nm: f64,
}

impl EngineDrivetrain {
    pub fn new(
        torque_curve: TorqueCurve,
        idle_rpm: f64,
        limiter_rpm: f64,
        inertia_kg_m2: f64,
        forward_ratios: &[f64],
        reverse_ratio: f64,
        final_ratio: f64,
        traction: Traction,
    ) -> Result<Self> {
        if !idle_rpm.is_finite() || idle_rpm < 0. {
            return Err(fail("Idle RPM must be finite and non-negative"));
        }
        if !limiter_rpm.is_finite() || limiter_rpm <= idle_rpm {
            return Err(fail("Limiter RPM must exceed idle RPM"));
        }
        if !inertia_kg_m2.is_finite() || inertia_kg_m2 <= 0. {
            return Err(fail("Engine inertia must be positive"));
        }
        if forward_ratios.is_empty() || forward_ratios.len() > 10 {
            return Err(fail("Expected 1..10 forward gear ratios"));
        }
        for &r in forward_ratios {
            if !r.is_finite() || r <= 0. {
                return Err(fail("Forward gear ratios must be positive"));
            }
        }
        if !reverse_ratio.is_finite() || reverse_ratio >= 0. {
            return Err(fail("Reverse ratio must be negative"));
        }
        if !final_ratio.is_finite() || final_ratio <= 0. {
            return Err(fail("Final drive ratio must be positive"));
        }
        Ok(Self {
            torque_curve,
            idle_rpm,
            limiter_rpm,
            inertia_kg_m2,
            forward_ratios: forward_ratios.to_vec(),
            reverse_ratio,
            final_ratio,
            traction,
        })
    }

    pub fn traction(&self) -> Traction {
        self.traction
    }

    pub fn idle_rpm(&self) -> f64 {
        self.idle_rpm
    }

    pub fn limiter_rpm(&self) -> f64 {
        self.limiter_rpm
    }

    pub fn inertia_kg_m2(&self) -> f64 {
        self.inertia_kg_m2
    }

    pub fn gear_count(&self) -> usize {
        self.forward_ratios.len()
    }

    fn gear_ratio(&self, gear: i32) -> f64 {
        if gear == -1 {
            self.reverse_ratio * self.final_ratio
        } else if gear >= 1 && (gear as usize) <= self.forward_ratios.len() {
            self.forward_ratios[gear as usize - 1] * self.final_ratio
        } else {
            0.
        }
    }

    /// Compute engine RPM from driven wheel angular velocity.
    /// When in neutral (gear=0), RPM is derived from throttle position.
    pub fn rpm(&self, wheel_rate_rad_s: f64, gear: i32, throttle: f64) -> Result<f64> {
        if !wheel_rate_rad_s.is_finite() || !throttle.is_finite() {
            return Err(fail("Wheel rate and throttle must be finite"));
        }
        let ratio = self.gear_ratio(gear);
        if ratio.abs() < 1e-12 {
            let t = throttle.clamp(0., 1.);
            return Ok(self.idle_rpm + t * (self.limiter_rpm - self.idle_rpm));
        }
        let rpm = (wheel_rate_rad_s * ratio.abs() * 60. / (2. * std::f64::consts::PI))
            .max(self.idle_rpm);
        Ok(rpm)
    }

    /// Compute drive torque delivered to each driven wheel.
    /// Returns the per-wheel torque and an engine sample for diagnostics.
    /// The transmission efficiency factor (0.9 in the prototype) accounts for
    /// frictional losses; this is a simplified model, not an AC clutch simulation.
    pub fn drive_torque(
        &self,
        wheel_rate_rad_s: f64,
        tyre_radius_m: f64,
        gear: i32,
        throttle: f64,
        driven_wheel_count: u32,
    ) -> Result<EngineSample> {
        if !tyre_radius_m.is_finite() || tyre_radius_m <= 0. {
            return Err(fail("Tyre radius must be positive"));
        }
        if driven_wheel_count == 0 || driven_wheel_count > 4 {
            return Err(fail("Driven wheel count must be 1..4"));
        }
        let ratio = self.gear_ratio(gear);
        let rpm = self.rpm(wheel_rate_rad_s, gear, throttle)?;
        let clamped_rpm = rpm.min(self.limiter_rpm);
        let engine_torque = if rpm >= self.limiter_rpm {
            0.
        } else {
            self.torque_curve.sample(clamped_rpm)? * throttle.clamp(0., 1.)
        };
        let wheel_torque = if ratio.abs() < 1e-12 {
            0.
        } else {
            engine_torque * ratio * 0.9 / driven_wheel_count as f64
        };
        Ok(EngineSample {
            rpm: clamped_rpm,
            torque_nm: engine_torque,
            gear,
            ratio,
            wheel_torque_nm: wheel_torque,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_curve() -> TorqueCurve {
        TorqueCurve::new(&[
            [0., 0.],
            [1000., 80.],
            [3000., 150.],
            [5000., 180.],
            [7000., 160.],
            [8500., 120.],
        ])
        .unwrap()
    }

    fn test_drivetrain() -> EngineDrivetrain {
        EngineDrivetrain::new(
            test_curve(),
            900.,
            8200.,
            0.12,
            &[3.5, 2.3, 1.7, 1.3, 1.0, 0.85],
            -3.2,
            3.7,
            Traction::Rwd,
        )
        .unwrap()
    }

    #[test]
    fn torque_curve_interpolates_and_clamps() {
        let curve = test_curve();
        assert_eq!(curve.sample(-100.).unwrap(), 0.);
        assert_eq!(curve.sample(0.).unwrap(), 0.);
        assert_eq!(curve.sample(1000.).unwrap(), 80.);
        let mid = curve.sample(2000.).unwrap();
        assert!((mid - 115.).abs() < 0.01);
        assert_eq!(curve.sample(10000.).unwrap(), 120.);
    }

    #[test]
    fn torque_curve_rejects_invalid_inputs() {
        assert!(TorqueCurve::new(&[]).is_err());
        assert!(TorqueCurve::new(&[[100., 50.]]).is_err());
        assert!(TorqueCurve::new(&[[200., 50.], [100., 60.]]).is_err());
        assert!(TorqueCurve::new(&[[-1., 0.], [100., 50.]]).is_err());
        assert!(TorqueCurve::new(&[[f64::NAN, 50.], [100., 60.]]).is_err());
    }

    #[test]
    fn neutral_rpm_tracks_throttle() {
        let dt = test_drivetrain();
        let rpm_idle = dt.rpm(0., 0, 0.).unwrap();
        assert!((rpm_idle - 900.).abs() < 0.01);
        let rpm_full = dt.rpm(0., 0, 1.).unwrap();
        assert!((rpm_full - 8200.).abs() < 0.01);
    }

    #[test]
    fn engaged_rpm_tracks_wheel_speed() {
        let dt = test_drivetrain();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 2000. * 2. * std::f64::consts::PI / (60. * ratio);
        let rpm = dt.rpm(wheel_rate, 1, 0.5).unwrap();
        assert!((rpm - 2000.).abs() < 1.);
    }

    #[test]
    fn idle_clamp_prevents_stall() {
        let dt = test_drivetrain();
        let rpm = dt.rpm(0., 1, 0.).unwrap();
        assert!((rpm - 900.).abs() < 0.01);
    }

    #[test]
    fn limiter_cuts_torque() {
        let dt = test_drivetrain();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 9000. * 2. * std::f64::consts::PI / (60. * ratio);
        let sample = dt.drive_torque(wheel_rate, 0.3, 1, 1., 2).unwrap();
        assert_eq!(sample.torque_nm, 0.);
        assert_eq!(sample.wheel_torque_nm, 0.);
    }

    #[test]
    fn drive_torque_transmits_through_ratio() {
        let dt = test_drivetrain();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 3000. * 2. * std::f64::consts::PI / (60. * ratio);
        let sample = dt.drive_torque(wheel_rate, 0.3, 1, 1., 2).unwrap();
        assert!((sample.rpm - 3000.).abs() < 1.);
        assert!((sample.torque_nm - 150.).abs() < 0.01);
        let expected = 150. * ratio * 0.9 / 2.;
        assert!((sample.wheel_torque_nm - expected).abs() < 0.1);
    }

    #[test]
    fn reverse_uses_negative_ratio() {
        let dt = test_drivetrain();
        let ratio: f64 = (-3.2) * 3.7;
        let wheel_rate = -3000. * 2. * std::f64::consts::PI / (60. * ratio.abs());
        let sample = dt.drive_torque(wheel_rate, 0.3, -1, 1., 2).unwrap();
        assert!(sample.wheel_torque_nm < 0.);
    }

    #[test]
    fn drivetrain_rejects_invalid_construction() {
        let curve = test_curve();
        assert!(EngineDrivetrain::new(curve.clone(), -1., 8000., 0.1, &[3.0], -3.0, 3.7, Traction::Rwd).is_err());
        assert!(EngineDrivetrain::new(curve.clone(), 900., 900., 0.1, &[3.0], -3.0, 3.7, Traction::Rwd).is_err());
        assert!(EngineDrivetrain::new(curve.clone(), 900., 8000., 0.1, &[], -3.0, 3.7, Traction::Rwd).is_err());
        assert!(EngineDrivetrain::new(curve.clone(), 900., 8000., 0.1, &[3.0], 3.0, 3.7, Traction::Rwd).is_err());
        assert!(EngineDrivetrain::new(curve, 900., 8000., 0.1, &[3.0], -3.0, -1., Traction::Rwd).is_err());
    }
}
