//! AC-style tyre force model using Pacejka magic formula with load sensitivity.
//! This replaces the generic viscous tangential contact for AC vehicle simulation.
use crate::{Error, Result};
use serde::Serialize;

fn fail(message: &str) -> Error {
    Error::new("physics_tyre", message)
}

#[derive(Clone, Debug)]
pub struct TyreModel {
    fz0: f64,
    dx_ref: f64,
    dy_ref: f64,
    ls_expx: f64,
    ls_expy: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct TyreModelConfig {
    pub fz0: f64,
    pub dx_ref: f64,
    pub dy_ref: f64,
    pub ls_expx: f64,
    pub ls_expy: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct TyreSample {
    pub slip_angle_rad: f64,
    pub slip_ratio: f64,
    pub lateral_force_n: f64,
    pub longitudinal_force_n: f64,
    pub normal_load_n: f64,
    pub mu_x: f64,
    pub mu_y: f64,
}

const C_LAT: f64 = 1.3;
const C_LONG: f64 = 1.65;
const B_LAT: f64 = 19.6;
const B_LONG: f64 = 14.0;

fn pacejka(b: f64, c: f64, d: f64, slip: f64) -> f64 {
    let bx = b * slip;
    d * (c * bx.atan()).sin()
}

impl TyreModel {
    pub fn new(config: TyreModelConfig) -> Result<Self> {
        if !config.fz0.is_finite() || config.fz0 <= 0. {
            return Err(fail("FZ0 must be positive"));
        }
        if !config.dx_ref.is_finite() || config.dx_ref <= 0. {
            return Err(fail("DX_REF must be positive"));
        }
        if !config.dy_ref.is_finite() || config.dy_ref <= 0. {
            return Err(fail("DY_REF must be positive"));
        }
        if !config.ls_expx.is_finite() || !(0. ..=2.).contains(&config.ls_expx) {
            return Err(fail("LS_EXPX must be in [0, 2]"));
        }
        if !config.ls_expy.is_finite() || !(0. ..=2.).contains(&config.ls_expy) {
            return Err(fail("LS_EXPY must be in [0, 2]"));
        }
        Ok(Self {
            fz0: config.fz0,
            dx_ref: config.dx_ref,
            dy_ref: config.dy_ref,
            ls_expx: config.ls_expx,
            ls_expy: config.ls_expy,
        })
    }

    fn mu_x(&self, fz: f64) -> f64 {
        self.dx_ref * (fz / self.fz0).powf(self.ls_expx - 1.)
    }

    fn mu_y(&self, fz: f64) -> f64 {
        self.dy_ref * (fz / self.fz0).powf(self.ls_expy - 1.)
    }

    pub fn forces(
        &self,
        normal_load_n: f64,
        slip_angle_rad: f64,
        slip_ratio: f64,
        surface_mu: f64,
    ) -> Result<TyreSample> {
        if !normal_load_n.is_finite() || normal_load_n < 0. {
            return Err(fail("Normal load must be non-negative"));
        }
        if !slip_angle_rad.is_finite() || !slip_ratio.is_finite() {
            return Err(fail("Slip values must be finite"));
        }
        if !surface_mu.is_finite() || surface_mu < 0. {
            return Err(fail("Surface friction must be non-negative"));
        }
        if normal_load_n < 1e-6 {
            return Ok(TyreSample {
                slip_angle_rad,
                slip_ratio,
                lateral_force_n: 0.,
                longitudinal_force_n: 0.,
                normal_load_n,
                mu_x: 0.,
                mu_y: 0.,
            });
        }
        let mu_x = self.mu_x(normal_load_n) * surface_mu;
        let mu_y = self.mu_y(normal_load_n) * surface_mu;
        let dx = mu_x * normal_load_n;
        let dy = mu_y * normal_load_n;
        let raw_fx = pacejka(B_LONG, C_LONG, dx, slip_ratio);
        let raw_fy = pacejka(B_LAT, C_LAT, dy, slip_angle_rad);
        let fx_norm = if dx > 0. { raw_fx / dx } else { 0. };
        let fy_norm = if dy > 0. { raw_fy / dy } else { 0. };
        let combined = fx_norm.hypot(fy_norm);
        let scale = if combined > 1. { 1. / combined } else { 1. };
        Ok(TyreSample {
            slip_angle_rad,
            slip_ratio,
            longitudinal_force_n: raw_fx * scale,
            lateral_force_n: raw_fy * scale,
            normal_load_n,
            mu_x,
            mu_y,
        })
    }
}

#[derive(Clone, Debug)]
pub struct TyreRelaxation {
    relaxation_length_m: f64,
    fx: f64,
    fy: f64,
}

impl TyreRelaxation {
    pub fn new(relaxation_length_m: f64) -> Result<Self> {
        if !relaxation_length_m.is_finite() || relaxation_length_m <= 0. {
            return Err(fail("Relaxation length must be positive"));
        }
        Ok(Self {
            relaxation_length_m,
            fx: 0.,
            fy: 0.,
        })
    }

    pub fn step(
        &mut self,
        steady_fx: f64,
        steady_fy: f64,
        ground_speed_m_s: f64,
        dt_s: f64,
    ) -> Result<(f64, f64)> {
        if !steady_fx.is_finite() || !steady_fy.is_finite() {
            return Err(fail("Steady-state forces must be finite"));
        }
        if !ground_speed_m_s.is_finite() || !dt_s.is_finite() || dt_s < 0. {
            return Err(fail("Speed and timestep must be finite and non-negative"));
        }
        let speed = ground_speed_m_s.abs().max(0.5);
        let alpha = (speed * dt_s / self.relaxation_length_m).min(1.);
        self.fx += alpha * (steady_fx - self.fx);
        self.fy += alpha * (steady_fy - self.fy);
        Ok((self.fx, self.fy))
    }

    pub fn forces(&self) -> (f64, f64) {
        (self.fx, self.fy)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_tyre() -> TyreModel {
        TyreModel::new(TyreModelConfig {
            fz0: 4000.,
            dx_ref: 1.3,
            dy_ref: 1.3,
            ls_expx: 0.85,
            ls_expy: 0.85,
        })
        .unwrap()
    }

    #[test]
    fn zero_load_gives_zero_force() {
        let t = test_tyre();
        let s = t.forces(0., 0.1, 0.05, 1.).unwrap();
        assert_eq!(s.lateral_force_n, 0.);
        assert_eq!(s.longitudinal_force_n, 0.);
    }

    #[test]
    fn zero_slip_gives_zero_force() {
        let t = test_tyre();
        let s = t.forces(4000., 0., 0., 1.).unwrap();
        assert!(s.lateral_force_n.abs() < 1e-9);
        assert!(s.longitudinal_force_n.abs() < 1e-9);
    }

    #[test]
    fn lateral_force_increases_with_slip_angle() {
        let t = test_tyre();
        let s1 = t.forces(4000., 0.02, 0., 1.).unwrap();
        let s2 = t.forces(4000., 0.06, 0., 1.).unwrap();
        assert!(s2.lateral_force_n.abs() > s1.lateral_force_n.abs());
    }

    #[test]
    fn longitudinal_force_increases_with_slip_ratio() {
        let t = test_tyre();
        let s1 = t.forces(4000., 0., 0.02, 1.).unwrap();
        let s2 = t.forces(4000., 0., 0.06, 1.).unwrap();
        assert!(s2.longitudinal_force_n.abs() > s1.longitudinal_force_n.abs());
    }

    #[test]
    fn load_sensitivity_reduces_mu_at_high_load() {
        let t = test_tyre();
        let s_ref = t.forces(4000., 0.1, 0., 1.).unwrap();
        let s_high = t.forces(8000., 0.1, 0., 1.).unwrap();
        let ratio_ref = s_ref.lateral_force_n / 4000.;
        let ratio_high = s_high.lateral_force_n / 8000.;
        assert!(
            ratio_high < ratio_ref,
            "mu should decrease with increasing load"
        );
    }

    #[test]
    fn combined_slip_limits_total_force() {
        let t = test_tyre();
        let pure_lat = t.forces(4000., 0.15, 0., 1.).unwrap();
        let combined = t.forces(4000., 0.15, 0.1, 1.).unwrap();
        assert!(
            combined.lateral_force_n.abs() < pure_lat.lateral_force_n.abs(),
            "combined slip should reduce lateral force"
        );
    }

    #[test]
    fn surface_friction_scales_force() {
        let t = test_tyre();
        let wet = t.forces(4000., 0.1, 0., 0.7).unwrap();
        let dry = t.forces(4000., 0.1, 0., 1.).unwrap();
        assert!(
            (wet.lateral_force_n / dry.lateral_force_n - 0.7).abs() < 0.05,
            "wet should be ~70% of dry"
        );
    }

    #[test]
    fn peak_lateral_near_reference_load() {
        let t = test_tyre();
        let s = t.forces(4000., 0.15, 0., 1.).unwrap();
        let peak_mu = s.lateral_force_n.abs() / 4000.;
        assert!(
            (peak_mu - 1.3).abs() < 0.15,
            "peak mu at FZ0 should be close to DY_REF: {peak_mu}"
        );
    }

    #[test]
    fn rejects_invalid_config() {
        assert!(
            TyreModel::new(TyreModelConfig {
                fz0: 0.,
                dx_ref: 1.3,
                dy_ref: 1.3,
                ls_expx: 0.85,
                ls_expy: 0.85,
            })
            .is_err()
        );
        assert!(
            TyreModel::new(TyreModelConfig {
                fz0: 4000.,
                dx_ref: -1.,
                dy_ref: 1.3,
                ls_expx: 0.85,
                ls_expy: 0.85,
            })
            .is_err()
        );
    }

    #[test]
    fn relaxation_converges_to_steady_state() {
        let mut r = TyreRelaxation::new(0.2).unwrap();
        for _ in 0..200 {
            r.step(1000., 500., 10., 1. / 120.).unwrap();
        }
        let (fx, fy) = r.forces();
        assert!((fx - 1000.).abs() < 1.0, "fx should converge: {fx}");
        assert!((fy - 500.).abs() < 0.5, "fy should converge: {fy}");
    }

    #[test]
    fn relaxation_delays_force_buildup() {
        let mut r = TyreRelaxation::new(0.3).unwrap();
        let (fx, _fy) = r.step(1000., 0., 10., 1. / 120.).unwrap();
        assert!(fx < 1000., "first step should be less than steady: {fx}");
        assert!(fx > 0., "first step should be positive: {fx}");
    }

    #[test]
    fn relaxation_rejects_invalid() {
        assert!(TyreRelaxation::new(0.).is_err());
        assert!(TyreRelaxation::new(-1.).is_err());
        assert!(TyreRelaxation::new(f64::NAN).is_err());
    }

    #[test]
    fn rejects_invalid_inputs() {
        let t = test_tyre();
        assert!(t.forces(-100., 0.1, 0., 1.).is_err());
        assert!(t.forces(4000., f64::NAN, 0., 1.).is_err());
        assert!(t.forces(4000., 0.1, 0., -0.1).is_err());
    }
}
