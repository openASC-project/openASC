//! Differential constraints and ideal massless statics, not a vehicle force law.
use crate::{
    Error, LinkPose, Result,
    linkage::{TOLERANCE, linear, point},
};
use glam::DVec3;
use serde::Serialize;

fn error(message: &str) -> Error {
    Error::new("physics_linkage_differential", message)
}
fn bounded(value: DVec3, limit: f64) -> Result<DVec3> {
    if !value.is_finite() || value.abs().max_element() > limit {
        return Err(error(
            "Nonfinite or out-of-range differential velocity/load",
        ));
    }
    Ok(value)
}
#[derive(Debug, Clone, Copy, Serialize)]
pub struct LinkLoad {
    force_n: [f64; 3],
    torque_about_upright_origin_nm: [f64; 3],
}
impl LinkLoad {
    /// Both vectors use the fixed linkage frame; torque is about its moving upright origin.
    pub fn new(force_n: [f64; 3], torque_about_upright_origin_nm: [f64; 3]) -> Result<Self> {
        bounded(DVec3::from_array(force_n), 1e18)?;
        bounded(DVec3::from_array(torque_about_upright_origin_nm), 1e18)?;
        Ok(Self {
            force_n,
            torque_about_upright_origin_nm,
        })
    }
    pub fn at_point(
        force_n: [f64; 3],
        upright_local_point_m: [f64; 3],
        pose: LinkPose,
    ) -> Result<Self> {
        let force = bounded(DVec3::from_array(force_n), 1e18)?;
        let lever = pose.rotation * point(upright_local_point_m)?;
        Self::new(force_n, lever.cross(force).to_array())
    }
    pub fn force_n(&self) -> [f64; 3] {
        self.force_n
    }
    pub fn torque_about_upright_origin_nm(&self) -> [f64; 3] {
        self.torque_about_upright_origin_nm
    }
    pub fn add(&mut self, other: Self) -> Result<()> {
        *self = Self::new(
            (DVec3::from_array(self.force_n) + DVec3::from_array(other.force_n)).to_array(),
            (DVec3::from_array(self.torque_about_upright_origin_nm)
                + DVec3::from_array(other.torque_about_upright_origin_nm))
            .to_array(),
        )?;
        Ok(())
    }
}
#[derive(Debug, Clone, Copy, Serialize)]
pub struct TangentColumn {
    pub origin_per_m: [f64; 3],
    pub angular_rad_per_m: [f64; 3],
}
#[derive(Debug, Clone, Copy, Serialize)]
pub struct RelativeVelocity {
    velocity_m_s: [f64; 3],
    angular_velocity_rad_s: [f64; 3],
    #[serde(skip)]
    pose: LinkPose,
}
impl RelativeVelocity {
    pub fn velocity_m_s(&self) -> [f64; 3] {
        self.velocity_m_s
    }
    pub fn angular_velocity_rad_s(&self) -> [f64; 3] {
        self.angular_velocity_rad_s
    }
    pub fn point_velocity_m_s(&self, upright_local_point_m: [f64; 3]) -> Result<[f64; 3]> {
        let lever = self.pose.rotation * point(upright_local_point_m)?;
        let result = DVec3::from_array(self.velocity_m_s)
            + DVec3::from_array(self.angular_velocity_rad_s).cross(lever);
        Ok(bounded(result, 1e6)?.to_array())
    }
}
#[derive(Debug, Clone, Copy, Serialize)]
pub struct RelativeAcceleration {
    pub acceleration_m_s2: [f64; 3],
    pub angular_acceleration_rad_s2: [f64; 3],
    #[serde(skip)]
    pose: LinkPose,
    #[serde(skip)]
    omega: DVec3,
}
impl RelativeAcceleration {
    pub fn point_acceleration_m_s2(&self, upright_local_point_m: [f64; 3]) -> Result<[f64; 3]> {
        let r = self.pose.rotation * point(upright_local_point_m)?;
        let value = DVec3::from_array(self.acceleration_m_s2)
            + DVec3::from_array(self.angular_acceleration_rad_s2).cross(r)
            + self.omega.cross(self.omega.cross(r));
        Ok(bounded(value, 1e13)?.to_array())
    }
}

pub(super) fn rod_bias(lever: DVec3, delta: DVec3, v: DVec3, w: DVec3, anchor_v: DVec3) -> f64 {
    let length = delta.length();
    let n = delta / length;
    let relative = v + w.cross(lever) - anchor_v;
    n.dot(w.cross(w.cross(lever))) + (relative.length_squared() - n.dot(relative).powi(2)) / length
}
#[derive(Debug, Serialize)]
pub struct GeneralizedLoads {
    pub travel_n: f64,
    pub steering_anchor_n: [f64; 3],
}
#[derive(Debug, Serialize)]
pub struct ConstraintBalance {
    /// Multipliers for the five topology constraints, excluding the travel actuator.
    pub constraint_multipliers_n: [f64; 5],
    pub travel_actuator_n: f64,
    pub support_force_on_upright_n: [f64; 3],
    pub support_torque_on_upright_nm: [f64; 3],
    pub travel_actuator_force_on_upright_n: [f64; 3],
    /// Support reaction only; the abstract travel actuator is excluded.
    pub support_force_on_frame_n: [f64; 3],
    pub support_torque_about_frame_origin_nm: [f64; 3],
    pub steering_anchor_force_n: [f64; 3],
    pub force_balance_residual_n: [f64; 3],
    pub torque_balance_residual_nm: [f64; 3],
}
#[derive(Debug, Serialize)]
pub struct LinkDifferential {
    control_order: [&'static str; 4],
    columns: [TangentColumn; 4],
    #[serde(skip)]
    pose: LinkPose,
    #[serde(skip)]
    jacobian: [[f64; 6]; 6],
    #[serde(skip)]
    rotational_scale: f64,
    #[serde(skip)]
    steering_row: usize,
}
impl LinkDifferential {
    pub(super) fn new(
        pose: LinkPose,
        rotational_scale: f64,
        residual: [f64; 6],
        jacobian: [[f64; 6]; 6],
        steering_row: usize,
    ) -> Result<Self> {
        if residual
            .iter()
            .any(|v| !v.is_finite() || v.abs() > TOLERANCE)
        {
            return Err(error(
                "Differential requires a converged pose for these travel/steering coordinates",
            ));
        }
        let mut columns = [TangentColumn {
            origin_per_m: [0.; 3],
            angular_rad_per_m: [0.; 3],
        }; 4];
        for (control, column) in columns.iter_mut().enumerate() {
            let mut rhs = [0.; 6];
            if control == 0 {
                rhs[5] = 1.;
            } else {
                rhs[steering_row] = jacobian[steering_row][control - 1];
            }
            let solved = linear(jacobian, rhs)?;
            column.origin_per_m = solved[..3].try_into().unwrap();
            column.angular_rad_per_m = std::array::from_fn(|i| solved[3 + i] / rotational_scale);
            if column.angular_rad_per_m.iter().any(|v| !v.is_finite()) {
                return Err(error("Nonfinite differential column"));
            }
        }
        Ok(Self {
            control_order: [
                "travel",
                "steering_anchor_x",
                "steering_anchor_y",
                "steering_anchor_z",
            ],
            columns,
            pose,
            jacobian,
            rotational_scale,
            steering_row,
        })
    }
    pub fn columns(&self) -> &[TangentColumn; 4] {
        &self.columns
    }
    pub fn pose(&self) -> LinkPose {
        self.pose
    }
    pub(super) fn solve_acceleration(
        &self,
        rates: [f64; 4],
        accelerations: [f64; 4],
        bias: [f64; 5],
    ) -> Result<RelativeAcceleration> {
        let velocity = self.velocity(rates[0], rates[1..].try_into().unwrap())?;
        if accelerations
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 1e6)
            || bias.iter().any(|v| !v.is_finite())
        {
            return Err(error(
                "Expected finite control accelerations within +/-1e6 m/s^2",
            ));
        }
        let mut rhs = std::array::from_fn(|i| if i < 5 { -bias[i] } else { accelerations[0] });
        for i in 0..3 {
            rhs[self.steering_row] += self.jacobian[self.steering_row][i] * accelerations[i + 1];
        }
        let result = linear(self.jacobian, rhs)?;
        Ok(RelativeAcceleration {
            acceleration_m_s2: bounded(DVec3::from_slice(&result[..3]), 1e9)?.to_array(),
            angular_acceleration_rad_s2: bounded(
                DVec3::from_slice(&result[3..]) / self.rotational_scale,
                1e9,
            )?
            .to_array(),
            pose: self.pose,
            omega: DVec3::from_array(velocity.angular_velocity_rad_s()),
        })
    }
    /// Velocities are relative to a fixed linkage frame, not world/body motion.
    pub fn velocity(
        &self,
        travel_rate_m_s: f64,
        steering_anchor_velocity_m_s: [f64; 3],
    ) -> Result<RelativeVelocity> {
        let rates = [
            travel_rate_m_s,
            steering_anchor_velocity_m_s[0],
            steering_anchor_velocity_m_s[1],
            steering_anchor_velocity_m_s[2],
        ];
        if rates.iter().any(|v| !v.is_finite() || v.abs() > 1e3) {
            return Err(error(
                "Input rates must be finite and within +/-1000 m/s per component",
            ));
        }
        let mut v = DVec3::ZERO;
        let mut w = DVec3::ZERO;
        for (column, rate) in self.columns.iter().zip(rates) {
            v += rate * DVec3::from_array(column.origin_per_m);
            w += rate * DVec3::from_array(column.angular_rad_per_m);
        }
        Ok(RelativeVelocity {
            velocity_m_s: bounded(v, 1e6)?.to_array(),
            angular_velocity_rad_s: bounded(w, 1e5)?.to_array(),
            pose: self.pose,
        })
    }
    pub fn project_load(&self, load: LinkLoad) -> Result<GeneralizedLoads> {
        let f = DVec3::from_array(load.force_n);
        let t = DVec3::from_array(load.torque_about_upright_origin_nm);
        let values = self.columns.map(|c| {
            f.dot(DVec3::from_array(c.origin_per_m)) + t.dot(DVec3::from_array(c.angular_rad_per_m))
        });
        if values.iter().any(|v| !v.is_finite() || v.abs() > 1e18) {
            return Err(error("Projected actuator load exceeds +/-1e18 N"));
        }
        Ok(GeneralizedLoads {
            travel_n: values[0],
            steering_anchor_n: values[1..].try_into().unwrap(),
        })
    }
    /// Ideal massless equilibrium at a held pose. No acceleration/inertia or
    /// spring, tyre, gravity or damping force is generated here.
    pub fn hold_load(&self, load: LinkLoad) -> Result<ConstraintBalance> {
        let f = DVec3::from_array(load.force_n);
        let t = DVec3::from_array(load.torque_about_upright_origin_nm);
        let scaled = [
            f.x,
            f.y,
            f.z,
            t.x / self.rotational_scale,
            t.y / self.rotational_scale,
            t.z / self.rotational_scale,
        ];
        let transpose =
            std::array::from_fn(|row| std::array::from_fn(|col| self.jacobian[col][row]));
        let multipliers = linear(transpose, scaled.map(|v| -v))?;
        if multipliers.iter().any(|v| !v.is_finite() || v.abs() > 1e18) {
            return Err(error("Holding constraint load exceeds +/-1e18 N"));
        }
        let mut force = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        for (row, value) in self.jacobian.iter().zip(multipliers).take(5) {
            force += value * DVec3::new(row[0], row[1], row[2]);
            torque += value * self.rotational_scale * DVec3::new(row[3], row[4], row[5]);
        }
        let travel_force = multipliers[5] * DVec3::from_slice(&self.jacobian[5][..3]);
        let force_error = f + force + travel_force;
        let torque_error = t + torque;
        let threshold = 1e-8 * scaled.iter().map(|v| v.abs()).fold(1., f64::max);
        if force_error.abs().max_element() > threshold
            || torque_error.abs().max_element() / self.rotational_scale > threshold
        {
            return Err(error(
                "Holding equilibrium residual exceeds scaled relative tolerance",
            ));
        }
        let on_frame_force = -force;
        let on_frame_torque = -torque - self.pose.position.cross(force);
        bounded(force, 1e18)?;
        bounded(torque, 1e18)?;
        bounded(travel_force, 1e18)?;
        bounded(on_frame_force, 1e18)?;
        bounded(on_frame_torque, 1e18)?;
        let steering = -multipliers[self.steering_row]
            * DVec3::from_slice(&self.jacobian[self.steering_row][..3]);
        bounded(steering, 1e18)?;
        Ok(ConstraintBalance {
            constraint_multipliers_n: multipliers[..5].try_into().unwrap(),
            travel_actuator_n: multipliers[5],
            support_force_on_upright_n: force.to_array(),
            support_torque_on_upright_nm: torque.to_array(),
            travel_actuator_force_on_upright_n: travel_force.to_array(),
            support_force_on_frame_n: on_frame_force.to_array(),
            support_torque_about_frame_origin_nm: on_frame_torque.to_array(),
            steering_anchor_force_n: steering.to_array(),
            force_balance_residual_n: force_error.to_array(),
            torque_balance_residual_nm: torque_error.to_array(),
        })
    }
}
