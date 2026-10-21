//! Independent compliant thin-circle contact against a static infinite plane.
use crate::{Error, Result, WheelKinematics, Wrench};
use glam::DVec3;
use serde::Serialize;
fn fail() -> Error {
    Error::new(
        "physics_normal_contact",
        "Invalid normal-contact geometry, parameters or numeric bounds",
    )
}
fn vector(value: [f64; 3], limit: f64) -> Result<DVec3> {
    let value = DVec3::from_array(value);
    if !value.is_finite() || value.abs().max_element() > limit {
        return Err(fail());
    }
    Ok(value)
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct SupportPlane {
    point_world_m: [f64; 3],
    normal_world: [f64; 3],
}
impl SupportPlane {
    pub fn new(point_world_m: [f64; 3], normal_world: [f64; 3]) -> Result<Self> {
        vector(point_world_m, 1e7)?;
        let normal = vector(normal_world, 2.)?;
        if (normal.length_squared() - 1.).abs() > 1e-8 {
            return Err(fail());
        }
        Ok(Self {
            point_world_m,
            normal_world: normal.normalize().to_array(),
        })
    }
}
#[derive(Clone, Debug)]
pub struct NormalContact {
    radius: f64,
    stiffness: f64,
    damping: f64,
    max_penetration: f64,
}
#[derive(Clone, Copy, Debug, Serialize)]
pub struct CircleSupport {
    pub wheel_point_world_m: [f64; 3],
    pub plane_point_world_m: [f64; 3],
    pub normal_world: [f64; 3],
    pub gap_m: f64,
    pub normal_velocity_m_s: f64,
    #[serde(skip)]
    pub(crate) lever: DVec3,
}
#[derive(Debug, Serialize)]
pub struct NormalContactSample {
    pub wheel_point_world_m: [f64; 3],
    pub plane_point_world_m: [f64; 3],
    pub normal_world: [f64; 3],
    pub gap_m: f64,
    pub penetration_m: f64,
    pub normal_velocity_m_s: f64,
    pub geometrically_touching: bool,
    pub normal_force_n: f64,
    pub force_world_n: [f64; 3],
    pub torque_about_wheel_com_world_nm: [f64; 3],
    pub potential_energy_j: f64,
    pub dissipated_power_w: f64,
    pub mechanical_power_w: f64,
}
impl NormalContactSample {
    pub fn wrench(&self) -> Result<Wrench> {
        Wrench::new(self.force_world_n, self.torque_about_wheel_com_world_nm)
    }
}
impl NormalContact {
    pub fn new(
        radius_m: f64,
        stiffness_n_m: f64,
        damping_n_s_m: f64,
        max_penetration_m: f64,
    ) -> Result<Self> {
        if !radius_m.is_finite()
            || !(1e-4..=10.).contains(&radius_m)
            || !stiffness_n_m.is_finite()
            || !(1. ..=1e9).contains(&stiffness_n_m)
            || !damping_n_s_m.is_finite()
            || !(0. ..=1e9).contains(&damping_n_s_m)
            || !max_penetration_m.is_finite()
            || !(1e-6..=0.5 * radius_m).contains(&max_penetration_m)
        {
            return Err(fail());
        }
        Ok(Self {
            radius: radius_m,
            stiffness: stiffness_n_m,
            damping: damping_n_s_m,
            max_penetration: max_penetration_m,
        })
    }
    pub fn sample(
        &self,
        wheel: &WheelKinematics,
        plane: SupportPlane,
    ) -> Result<NormalContactSample> {
        let geometry = self.support_geometry(wheel, plane)?;
        self.normal_sample(geometry)
    }
    pub fn sample_with_friction(
        &self,
        wheel: &WheelKinematics,
        plane: SupportPlane,
        friction: &crate::TangentialContact,
        surface_friction_scale: f64,
    ) -> Result<crate::ContactSample> {
        let geometry = self.support_geometry(wheel, plane)?;
        let normal = self.normal_sample(geometry)?;
        let tangential = friction.sample(
            wheel,
            geometry,
            normal.normal_force_n,
            surface_friction_scale,
        )?;
        Ok(crate::ContactSample { normal, tangential })
    }
    fn normal_sample(&self, geometry: CircleSupport) -> Result<NormalContactSample> {
        let gap = geometry.gap_m;
        let penetration = (-gap).max(0.);
        if penetration > self.max_penetration {
            return Err(Error::new(
                "physics_contact_penetration",
                "Normal contact penetration exceeds the explicit limit; reduce the step or correct placement",
            ));
        }
        let normal = DVec3::from_array(geometry.normal_world);
        let normal_velocity = geometry.normal_velocity_m_s;
        let elastic = self.stiffness * penetration;
        let touching = gap <= 0.;
        let raw = elastic - self.damping * normal_velocity;
        let force = if touching { raw.max(0.) } else { 0. };
        let dissipated = if !touching {
            0.
        } else if raw >= 0. {
            self.damping * normal_velocity.powi(2)
        } else {
            elastic * normal_velocity
        };
        let force_vector = normal * force;
        let torque = geometry.lever.cross(force_vector);
        Wrench::new(force_vector.to_array(), torque.to_array())?;
        Ok(NormalContactSample {
            wheel_point_world_m: geometry.wheel_point_world_m,
            plane_point_world_m: geometry.plane_point_world_m,
            normal_world: geometry.normal_world,
            gap_m: gap,
            penetration_m: penetration,
            normal_velocity_m_s: normal_velocity,
            geometrically_touching: touching,
            normal_force_n: force,
            force_world_n: force_vector.to_array(),
            torque_about_wheel_com_world_nm: torque.to_array(),
            potential_energy_j: 0.5 * self.stiffness * penetration * penetration,
            dissipated_power_w: dissipated,
            mechanical_power_w: force * normal_velocity,
        })
    }
    pub fn radius_m(&self) -> f64 {
        self.radius
    }
    pub fn max_penetration_m(&self) -> f64 {
        self.max_penetration
    }
    /// Geometry/velocity only; finite-face callers can clip the projected point
    /// before evaluating the force or enforcing the penetration bound.
    pub fn support_geometry(
        &self,
        wheel: &WheelKinematics,
        plane: SupportPlane,
    ) -> Result<CircleSupport> {
        let center = vector(wheel.body.position_m, 1e7)?;
        let axis = vector(wheel.axis_world, 2.)?;
        if (axis.length_squared() - 1.).abs() > 1e-8 {
            return Err(fail());
        }
        let axis = axis.normalize();
        let normal = DVec3::from_array(plane.normal_world);
        let radial = normal - axis * axis.dot(normal);
        let projection = radial.length();
        if projection < 1e-6 {
            return Err(Error::new(
                "physics_contact_axis",
                "Wheel axle is nearly parallel to the support normal",
            ));
        }
        let lever = -self.radius * radial / projection;
        let point = center + lever;
        vector(point.to_array(), 1e7)?;
        let gap = normal.dot(point - DVec3::from_array(plane.point_world_m));
        let velocity = vector(wheel.body.velocity_world_m_s, 1e6)?
            + vector(wheel.body.angular_velocity_world_rad_s, 1e5)?.cross(lever);
        vector(velocity.to_array(), 1e7)?;
        let normal_velocity = normal.dot(velocity);
        let plane_point = point - gap * normal;
        vector(plane_point.to_array(), 1e7)?;
        Ok(CircleSupport {
            wheel_point_world_m: point.to_array(),
            plane_point_world_m: plane_point.to_array(),
            normal_world: plane.normal_world,
            gap_m: gap,
            normal_velocity_m_s: normal_velocity,
            lever,
        })
    }
}
