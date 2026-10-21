//! Independent memoryless viscous slip law with an anisotropic friction ellipse.
//! This is not an AC tyre model, static-friction constraint or transient brush model.
use crate::{CircleSupport, Error, NormalContactSample, Result, WheelKinematics, Wrench};
use glam::DVec3;
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct TangentialContact {
    viscosity: [f64; 2],
    friction: [f64; 2],
}
#[derive(Debug, Serialize)]
pub struct TangentialContactSample {
    pub longitudinal_world: [f64; 3],
    pub lateral_world: [f64; 3],
    pub slip_longitudinal_m_s: f64,
    pub slip_lateral_m_s: f64,
    pub longitudinal_force_n: f64,
    pub lateral_force_n: f64,
    pub longitudinal_limit_n: f64,
    pub lateral_limit_n: f64,
    pub surface_friction_scale: f64,
    pub saturation_scale: f64,
    pub force_world_n: [f64; 3],
    pub torque_about_wheel_com_world_nm: [f64; 3],
    pub dissipated_power_w: f64,
    pub mechanical_power_w: f64,
}
impl TangentialContactSample {
    pub fn wrench(&self) -> Result<Wrench> {
        Wrench::new(self.force_world_n, self.torque_about_wheel_com_world_nm)
    }
}
#[derive(Debug, Serialize)]
pub struct ContactSample {
    pub normal: NormalContactSample,
    pub tangential: TangentialContactSample,
}
impl ContactSample {
    pub fn wrench(&self) -> Result<Wrench> {
        let force = DVec3::from_array(self.normal.force_world_n)
            + DVec3::from_array(self.tangential.force_world_n);
        let torque = DVec3::from_array(self.normal.torque_about_wheel_com_world_nm)
            + DVec3::from_array(self.tangential.torque_about_wheel_com_world_nm);
        Wrench::new(force.to_array(), torque.to_array())
    }
}
fn fail() -> Error {
    Error::new(
        "physics_tangential_contact",
        "Invalid tangential contact parameters, surface scale, wheel state or load",
    )
}
impl TangentialContact {
    /// Viscosities 0..1e7 N s/m; friction coefficients 0..10. Zero disables a direction.
    pub fn new(
        longitudinal_viscosity_n_s_m: f64,
        lateral_viscosity_n_s_m: f64,
        longitudinal_friction: f64,
        lateral_friction: f64,
    ) -> Result<Self> {
        let viscosity = [longitudinal_viscosity_n_s_m, lateral_viscosity_n_s_m];
        let friction = [longitudinal_friction, lateral_friction];
        if viscosity
            .iter()
            .any(|v| !v.is_finite() || !(0. ..=1e7).contains(v))
            || friction
                .iter()
                .any(|v| !v.is_finite() || !(0. ..=10.).contains(v))
        {
            return Err(fail());
        }
        Ok(Self {
            viscosity,
            friction,
        })
    }
    pub(crate) fn sample(
        &self,
        wheel: &WheelKinematics,
        geometry: CircleSupport,
        normal_load_n: f64,
        surface_friction_scale: f64,
    ) -> Result<TangentialContactSample> {
        if !normal_load_n.is_finite()
            || !(0. ..=1e18).contains(&normal_load_n)
            || !surface_friction_scale.is_finite()
            || !(0. ..=10.).contains(&surface_friction_scale)
        {
            return Err(fail());
        }
        // Geometry and wheel vectors were validated by NormalContact::support_geometry.
        let normal = DVec3::from_array(geometry.normal_world);
        let longitudinal = DVec3::from_array(wheel.axis_world)
            .cross(normal)
            .normalize();
        let lateral = normal.cross(longitudinal);
        let velocity = DVec3::from_array(wheel.body.velocity_world_m_s)
            + DVec3::from_array(wheel.body.angular_velocity_world_rad_s).cross(geometry.lever);
        let slip = [velocity.dot(longitudinal), velocity.dot(lateral)];
        let limits = self
            .friction
            .map(|mu| mu * surface_friction_scale * normal_load_n);
        let raw: [f64; 2] = std::array::from_fn(|i| {
            if limits[i] > 0. {
                -self.viscosity[i] * slip[i]
            } else {
                0.
            }
        });
        let ratio: [f64; 2] = std::array::from_fn(|i| {
            if limits[i] > 0. {
                raw[i] / limits[i]
            } else {
                0.
            }
        });
        let utilization = ratio[0].hypot(ratio[1]);
        if !utilization.is_finite() {
            return Err(fail());
        }
        let scale = if utilization > 1. {
            1. / utilization
        } else {
            1.
        };
        let f = raw.map(|v| v * scale);
        let force = longitudinal * f[0] + lateral * f[1];
        // Apply at the material wheel point, not the projected plane point:
        // tangential force at those different points has a different wheel moment.
        let torque = geometry.lever.cross(force);
        Wrench::new(force.to_array(), torque.to_array())?;
        let power = f[0] * slip[0] + f[1] * slip[1];
        if !force.is_finite() || !torque.is_finite() || !power.is_finite() || power > 0. {
            return Err(fail());
        }
        Ok(TangentialContactSample {
            longitudinal_world: longitudinal.to_array(),
            lateral_world: lateral.to_array(),
            slip_longitudinal_m_s: slip[0],
            slip_lateral_m_s: slip[1],
            longitudinal_force_n: f[0],
            lateral_force_n: f[1],
            longitudinal_limit_n: limits[0],
            lateral_limit_n: limits[1],
            surface_friction_scale,
            saturation_scale: scale,
            force_world_n: force.to_array(),
            torque_about_wheel_com_world_nm: torque.to_array(),
            mechanical_power_w: power,
            dissipated_power_w: -power,
        })
    }
}
