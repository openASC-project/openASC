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

#[derive(Clone, Copy, Debug)]
pub struct CoastRef {
    pub rpm: f64,
    pub torque_nm: f64,
    pub non_linearity: f64,
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
    coast: Option<CoastRef>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Traction {
    Rwd,
    Fwd,
}

#[derive(Clone, Copy, Debug)]
pub struct DiffParams {
    pub power_lock: f64,
    pub coast_lock: f64,
    pub preload_nm: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct DiffSplit {
    pub left_torque_nm: f64,
    pub right_torque_nm: f64,
}

impl DiffParams {
    pub fn validate(&self) -> Result<()> {
        if !self.power_lock.is_finite() || !(0. ..=1.).contains(&self.power_lock) {
            return Err(fail("Differential power lock must be in [0, 1]"));
        }
        if !self.coast_lock.is_finite() || !(0. ..=1.).contains(&self.coast_lock) {
            return Err(fail("Differential coast lock must be in [0, 1]"));
        }
        if !self.preload_nm.is_finite() || self.preload_nm < 0. {
            return Err(fail("Differential preload must be non-negative"));
        }
        Ok(())
    }

    pub fn split(
        &self,
        total_torque_nm: f64,
        left_rate_rad_s: f64,
        right_rate_rad_s: f64,
    ) -> DiffSplit {
        let lock = if total_torque_nm >= 0. {
            self.power_lock
        } else {
            self.coast_lock
        };
        let locking_torque = lock * total_torque_nm.abs() / 2. + self.preload_nm;
        let speed_diff = left_rate_rad_s - right_rate_rad_s;
        let transfer = if speed_diff.abs() < 1e-9 {
            0.
        } else {
            locking_torque.copysign(speed_diff)
        };
        let half = total_torque_nm / 2.;
        DiffSplit {
            left_torque_nm: half + transfer,
            right_torque_nm: half - transfer,
        }
    }
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
        coast: Option<CoastRef>,
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
        if let Some(c) = &coast {
            if !c.rpm.is_finite() || c.rpm <= 0. {
                return Err(fail("Coast reference RPM must be positive"));
            }
            if !c.torque_nm.is_finite() || c.torque_nm < 0. {
                return Err(fail("Coast reference torque must be non-negative"));
            }
            if !c.non_linearity.is_finite() || !(0. ..=1.).contains(&c.non_linearity) {
                return Err(fail("Coast non-linearity must be in [0, 1]"));
            }
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
            coast,
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
        let rpm =
            (wheel_rate_rad_s * ratio.abs() * 60. / (2. * std::f64::consts::PI)).max(self.idle_rpm);
        Ok(rpm)
    }

    /// Coast (engine braking) torque at a given RPM, from COAST_REF parameters.
    /// Returns a non-negative value representing the retarding torque magnitude.
    fn coast_torque_at_rpm(&self, rpm: f64) -> f64 {
        match self.coast {
            Some(c) if c.torque_nm > 0. && c.rpm > 0. => {
                let t = (rpm / c.rpm).clamp(0., 1.);
                let exponent = 1. + c.non_linearity;
                c.torque_nm * t.powf(exponent)
            }
            _ => 0.,
        }
    }

    /// Compute drive torque delivered to each driven wheel.
    /// Returns the per-wheel torque and an engine sample for diagnostics.
    /// The transmission efficiency factor (0.9 in the prototype) accounts for
    /// frictional losses; this is a simplified model, not an AC clutch simulation.
    /// When throttle < 1 and a gear is engaged, coast (engine braking) torque
    /// opposes the driven wheels proportional to (1 - throttle).
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
        let throttle_clamped = throttle.clamp(0., 1.);
        let power_torque = if rpm >= self.limiter_rpm {
            0.
        } else {
            self.torque_curve.sample(clamped_rpm)? * throttle_clamped
        };
        let coast = self.coast_torque_at_rpm(clamped_rpm) * (1. - throttle_clamped);
        let engine_torque = power_torque - coast;
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
            Some(CoastRef {
                rpm: 7000.,
                torque_nm: 80.,
                non_linearity: 0.,
            }),
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
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                -1.,
                8000.,
                0.1,
                &[3.0],
                -3.0,
                3.7,
                Traction::Rwd,
                None
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                900.,
                900.,
                0.1,
                &[3.0],
                -3.0,
                3.7,
                Traction::Rwd,
                None
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                900.,
                8000.,
                0.1,
                &[],
                -3.0,
                3.7,
                Traction::Rwd,
                None
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                900.,
                8000.,
                0.1,
                &[3.0],
                3.0,
                3.7,
                Traction::Rwd,
                None
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                900.,
                8000.,
                0.1,
                &[3.0],
                -3.0,
                -1.,
                Traction::Rwd,
                None
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve.clone(),
                900.,
                8000.,
                0.1,
                &[3.0],
                -3.0,
                3.7,
                Traction::Rwd,
                Some(CoastRef {
                    rpm: 0.,
                    torque_nm: 50.,
                    non_linearity: 0.
                })
            )
            .is_err()
        );
        assert!(
            EngineDrivetrain::new(
                curve,
                900.,
                8000.,
                0.1,
                &[3.0],
                -3.0,
                3.7,
                Traction::Rwd,
                Some(CoastRef {
                    rpm: 7000.,
                    torque_nm: 50.,
                    non_linearity: 1.5
                })
            )
            .is_err()
        );
    }

    #[test]
    fn coast_torque_retards_at_zero_throttle() {
        let dt = test_drivetrain();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 3000. * 2. * std::f64::consts::PI / (60. * ratio);
        let sample = dt.drive_torque(wheel_rate, 0.3, 1, 0., 2).unwrap();
        assert!(
            sample.torque_nm < 0.,
            "engine torque should be negative (retarding)"
        );
        assert!(
            sample.wheel_torque_nm < 0.,
            "wheel torque should retard forward motion"
        );
        let expected_coast = 80. * (3000. / 7000.);
        assert!((sample.torque_nm + expected_coast).abs() < 0.1);
    }

    #[test]
    fn coast_torque_scales_with_throttle_complement() {
        let dt = test_drivetrain();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 3000. * 2. * std::f64::consts::PI / (60. * ratio);
        let full = dt.drive_torque(wheel_rate, 0.3, 1, 1., 2).unwrap();
        let half = dt.drive_torque(wheel_rate, 0.3, 1, 0.5, 2).unwrap();
        let zero = dt.drive_torque(wheel_rate, 0.3, 1, 0., 2).unwrap();
        assert!(full.torque_nm > half.torque_nm);
        assert!(half.torque_nm > zero.torque_nm);
        assert!(full.torque_nm > 0.);
        assert!(zero.torque_nm < 0.);
    }

    #[test]
    fn no_coast_without_coast_ref() {
        let dt = EngineDrivetrain::new(
            test_curve(),
            900.,
            8200.,
            0.12,
            &[3.5, 2.3],
            -3.2,
            3.7,
            Traction::Rwd,
            None,
        )
        .unwrap();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 3000. * 2. * std::f64::consts::PI / (60. * ratio);
        let sample = dt.drive_torque(wheel_rate, 0.3, 1, 0., 2).unwrap();
        assert_eq!(sample.torque_nm, 0.);
        assert_eq!(sample.wheel_torque_nm, 0.);
    }

    #[test]
    fn coast_nonlinearity_increases_curvature() {
        let linear = EngineDrivetrain::new(
            test_curve(),
            900.,
            8200.,
            0.12,
            &[3.5],
            -3.2,
            3.7,
            Traction::Rwd,
            Some(CoastRef {
                rpm: 7000.,
                torque_nm: 80.,
                non_linearity: 0.,
            }),
        )
        .unwrap();
        let curved = EngineDrivetrain::new(
            test_curve(),
            900.,
            8200.,
            0.12,
            &[3.5],
            -3.2,
            3.7,
            Traction::Rwd,
            Some(CoastRef {
                rpm: 7000.,
                torque_nm: 80.,
                non_linearity: 1.,
            }),
        )
        .unwrap();
        let ratio = 3.5 * 3.7;
        let wheel_rate = 3500. * 2. * std::f64::consts::PI / (60. * ratio);
        let s_lin = linear.drive_torque(wheel_rate, 0.3, 1, 0., 2).unwrap();
        let s_cur = curved.drive_torque(wheel_rate, 0.3, 1, 0., 2).unwrap();
        assert!(
            s_cur.torque_nm > s_lin.torque_nm,
            "nonlinear coast at low RPM should be less retarding (closer to zero)"
        );
    }

    #[test]
    fn diff_equal_speed_splits_evenly() {
        let diff = DiffParams {
            power_lock: 0.5,
            coast_lock: 0.3,
            preload_nm: 0.,
        };
        diff.validate().unwrap();
        let split = diff.split(200., 100., 100.);
        assert!((split.left_torque_nm - 100.).abs() < 1e-9);
        assert!((split.right_torque_nm - 100.).abs() < 1e-9);
    }

    #[test]
    fn diff_transfers_torque_to_slower_wheel() {
        let diff = DiffParams {
            power_lock: 0.6,
            coast_lock: 0.2,
            preload_nm: 0.,
        };
        let split = diff.split(200., 80., 120.);
        assert!(
            split.left_torque_nm < split.right_torque_nm,
            "faster left wheel should get less torque"
        );
        let transfer = 0.6 * 200. / 2.;
        assert!((split.left_torque_nm - (100. - transfer)).abs() < 1e-9);
        assert!((split.right_torque_nm - (100. + transfer)).abs() < 1e-9);
    }

    #[test]
    fn diff_uses_coast_lock_for_negative_torque() {
        let diff = DiffParams {
            power_lock: 0.8,
            coast_lock: 0.2,
            preload_nm: 0.,
        };
        let split = diff.split(-100., 80., 120.);
        let transfer = 0.2 * 100. / 2.;
        assert!((split.left_torque_nm - (-50. - transfer)).abs() < 1e-9);
        assert!((split.right_torque_nm - (-50. + transfer)).abs() < 1e-9);
    }

    #[test]
    fn diff_preload_adds_minimum_locking() {
        let diff = DiffParams {
            power_lock: 0.,
            coast_lock: 0.,
            preload_nm: 30.,
        };
        let split = diff.split(200., 80., 120.);
        assert!((split.left_torque_nm - (100. - 30.)).abs() < 1e-9);
        assert!((split.right_torque_nm - (100. + 30.)).abs() < 1e-9);
    }

    #[test]
    fn diff_open_splits_evenly_regardless_of_speed_diff() {
        let diff = DiffParams {
            power_lock: 0.,
            coast_lock: 0.,
            preload_nm: 0.,
        };
        let split = diff.split(200., 50., 150.);
        assert!((split.left_torque_nm - 100.).abs() < 1e-9);
        assert!((split.right_torque_nm - 100.).abs() < 1e-9);
    }

    #[test]
    fn diff_rejects_invalid_params() {
        assert!(
            DiffParams {
                power_lock: -0.1,
                coast_lock: 0.,
                preload_nm: 0.
            }
            .validate()
            .is_err()
        );
        assert!(
            DiffParams {
                power_lock: 1.1,
                coast_lock: 0.,
                preload_nm: 0.
            }
            .validate()
            .is_err()
        );
        assert!(
            DiffParams {
                power_lock: 0.,
                coast_lock: -0.1,
                preload_nm: 0.
            }
            .validate()
            .is_err()
        );
        assert!(
            DiffParams {
                power_lock: 0.,
                coast_lock: 0.,
                preload_nm: -1.
            }
            .validate()
            .is_err()
        );
        assert!(
            DiffParams {
                power_lock: 0.5,
                coast_lock: 0.5,
                preload_nm: 10.
            }
            .validate()
            .is_ok()
        );
    }
}
