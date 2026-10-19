//! Explicit fixed-frame to upright spring/damper; no inferred AC mounting/law.
use crate::{Damper, GeneralizedLoads, LinkDifferential, LinkLoad, Result, SpringLink};
use glam::DVec3;
use serde::Serialize;

#[derive(Debug, Clone)]
pub struct LinkSpring(SpringLink);

#[derive(Debug, Serialize)]
pub struct LinkSpringSample {
    pub fixed_anchor_m: [f64; 3],
    pub upright_local_point_m: [f64; 3],
    pub upright_attachment_in_frame_m: [f64; 3],
    pub upright_point_velocity_m_s: [f64; 3],
    pub length_m: f64,
    pub extension_m: f64,
    pub extension_rate_m_s: f64,
    /// dl/d[travel, rack_x, rack_y, rack_z], metres per metre.
    pub length_gradient: [f64; 4],
    pub elastic_tension_n: f64,
    pub damping_tension_n: f64,
    pub potential_energy_j: f64,
    pub dissipated_power_w: f64,
    pub upright_load: LinkLoad,
    pub generalized_loads: GeneralizedLoads,
    /// Direct spring reaction only; excludes linkage supports and travel actuator.
    pub force_on_frame_n: [f64; 3],
    pub torque_about_frame_origin_nm: [f64; 3],
}
impl LinkSpring {
    pub fn new(
        fixed_anchor_m: [f64; 3],
        upright_local_point_m: [f64; 3],
        rest_length_m: f64,
        stiffness_n_m: f64,
        damper: Damper,
    ) -> Result<Self> {
        crate::linkage::point(fixed_anchor_m)?;
        crate::linkage::point(upright_local_point_m)?;
        Ok(Self(SpringLink::new(
            fixed_anchor_m,
            upright_local_point_m,
            rest_length_m,
            stiffness_n_m,
            damper,
        )?))
    }
    /// Instantaneous loads at a converged mechanism pose, in its fixed frame.
    /// No state is advanced and no inertia, tyre or gravity load is generated.
    pub fn sample(
        &self,
        mapping: &LinkDifferential,
        travel_rate_m_s: f64,
        steering_anchor_velocity_m_s: [f64; 3],
    ) -> Result<LinkSpringSample> {
        let (anchor, attachment) = self.0.points();
        let pose = mapping.pose();
        let end = pose.transform(attachment)?;
        let velocity = mapping
            .velocity(travel_rate_m_s, steering_anchor_velocity_m_s)?
            .point_velocity_m_s(attachment)?;
        let sample = self.0.sample_between(anchor, end, [0.; 3], velocity)?;
        let load = LinkLoad::at_point(sample.force_on_b_world_n, attachment, pose)?;
        let axis = (DVec3::from_array(end) - DVec3::from_array(anchor)) / sample.length_m;
        let lever = pose.rotation * DVec3::from_array(attachment);
        let gradient = mapping.columns().map(|column| {
            axis.dot(
                DVec3::from_array(column.origin_per_m)
                    + DVec3::from_array(column.angular_rad_per_m).cross(lever),
            )
        });
        let frame_torque =
            DVec3::from_array(anchor).cross(DVec3::from_array(sample.force_on_a_world_n));
        Ok(LinkSpringSample {
            fixed_anchor_m: anchor,
            upright_local_point_m: attachment,
            upright_attachment_in_frame_m: end,
            upright_point_velocity_m_s: velocity,
            length_m: sample.length_m,
            extension_m: sample.extension_m,
            extension_rate_m_s: sample.extension_rate_m_s,
            length_gradient: gradient,
            elastic_tension_n: sample.elastic_tension_n,
            damping_tension_n: sample.damping_tension_n,
            potential_energy_j: sample.potential_energy_j,
            dissipated_power_w: sample.dissipated_power_w,
            upright_load: load,
            generalized_loads: mapping.project_load(load)?,
            force_on_frame_n: sample.force_on_a_world_n,
            torque_about_frame_origin_nm: frame_torque.to_array(),
        })
    }
}
