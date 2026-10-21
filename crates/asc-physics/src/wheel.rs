//! Separate axisymmetric rotor mass/inertia, spinning relative to an upright.
use crate::{Error, MassProperties, Result, State, Wrench};
use glam::{DMat3, DVec3};
use serde::Serialize;

fn fail() -> Error {
    Error::new(
        "physics_wheel",
        "Expected a unit axle, bounded center/state and physically realizable axisymmetric wheel mass/inertia",
    )
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct WheelState {
    pub angle_rad: f64,
    pub rate_rad_s: f64,
}
impl WheelState {
    pub(super) fn validate(self) -> Result<()> {
        if !self.angle_rad.is_finite()
            || self.angle_rad.abs() > 1e9
            || !self.rate_rad_s.is_finite()
            || self.rate_rad_s.abs() > 1e5
        {
            return Err(fail());
        }
        Ok(())
    }
}
#[derive(Clone, Debug)]
pub struct Wheel {
    pub(super) center: DVec3,
    pub(super) axis: DVec3,
    pub(super) properties: MassProperties,
    pub(super) state: WheelState,
}
impl Wheel {
    /// Wheel mass is additional to the upright. Both inertias are about the
    /// wheel COM; axial <= 2*transverse is required for a physical rotor.
    pub fn new(
        center_in_upright_m: [f64; 3],
        axle_in_upright: [f64; 3],
        mass_kg: f64,
        axial_inertia_kg_m2: f64,
        transverse_inertia_kg_m2: f64,
        state: WheelState,
    ) -> Result<Self> {
        let center = crate::linkage::point(center_in_upright_m)?;
        let axis = crate::linkage::point(axle_in_upright)?;
        if (axis.length_squared() - 1.).abs() > 1e-8
            || !axial_inertia_kg_m2.is_finite()
            || axial_inertia_kg_m2 <= 0.
            || !transverse_inertia_kg_m2.is_finite()
            || transverse_inertia_kg_m2 <= 0.
        {
            return Err(fail());
        }
        state.validate()?;
        let axis = axis.normalize();
        let outer = DMat3::from_cols(axis * axis.x, axis * axis.y, axis * axis.z);
        let tensor = DMat3::IDENTITY * transverse_inertia_kg_m2
            + outer * (axial_inertia_kg_m2 - transverse_inertia_kg_m2);
        let rows = std::array::from_fn(|i| std::array::from_fn(|j| tensor.col(j)[i]));
        Ok(Self {
            center,
            axis,
            properties: MassProperties::new(mass_kg, rows)?,
            state,
        })
    }
}
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelLoad {
    pub(super) external: Wrench,
    pub(super) drive: f64,
}
impl WheelLoad {
    /// External world COM wrench, plus internal motor torque relative to the upright.
    pub fn new(external_world_com_wrench: Wrench, drive_torque_nm: f64) -> Result<Self> {
        if !drive_torque_nm.is_finite() || drive_torque_nm.abs() > 1e18 {
            return Err(fail());
        }
        Ok(Self {
            external: external_world_com_wrench,
            drive: drive_torque_nm,
        })
    }
    pub(super) fn is_zero(self) -> bool {
        self.drive == 0.
            && self.external.force_world_n() == [0.; 3]
            && self.external.torque_world_nm() == [0.; 3]
    }
    pub fn add_external(&mut self, world_com_wrench: Wrench) -> Result<()> {
        self.external.add(world_com_wrench)
    }
    /// Combined internal actuator torque about the signed wheel axle.
    pub fn internal_torque_nm(&self) -> f64 {
        self.drive
    }
    pub fn set_internal_torque(&mut self, torque_nm: f64) -> Result<()> {
        if !torque_nm.is_finite() || torque_nm.abs() > 1e18 {
            return Err(fail());
        }
        self.drive = torque_nm;
        Ok(())
    }
    /// Transactional addition of an internal motor/brake torque, including its
    /// opposite upright reaction in the coupled solver.
    pub fn add_internal_torque(&mut self, torque_nm: f64) -> Result<()> {
        let total = self.drive + torque_nm;
        if !torque_nm.is_finite()
            || torque_nm.abs() > 1e18
            || !total.is_finite()
            || total.abs() > 1e18
        {
            return Err(fail());
        }
        self.drive = total;
        Ok(())
    }
}
#[derive(Debug, Serialize)]
pub struct WheelSample {
    pub state: WheelState,
    pub spin_acceleration_rad_s2: f64,
    pub axis_world: [f64; 3],
    pub body: State,
    pub acceleration_world_m_s2: [f64; 3],
    pub angular_acceleration_world_rad_s2: [f64; 3],
    pub drive_torque_nm: f64,
    pub spin_balance_residual_nm: f64,
    pub force_on_upright_world_n: [f64; 3],
    /// Combined bearing and motor reaction, about the upright origin.
    pub torque_about_upright_origin_world_nm: [f64; 3],
    pub motor_reaction_on_upright_world_nm: [f64; 3],
    pub bearing_torque_about_wheel_com_on_upright_world_nm: [f64; 3],
}
