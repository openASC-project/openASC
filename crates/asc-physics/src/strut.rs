//! Rigid upright with a telescoping coaxial strut and three fixed distances.
use crate::{
    LinkPose, Result,
    linkage::{System, error, linear, point, solve_local},
};
use glam::DVec3;
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct StrutLink {
    anchors: [DVec3; 3],
    attachments: [DVec3; 3],
    lengths: [f64; 3],
    top: DVec3,
    strut_point: DVec3,
    strut_axis: DVec3,
    normals: [DVec3; 2],
    reference: LinkPose,
    axis: DVec3,
    rotational_scale: f64,
    reference_length: f64,
}
#[derive(Debug, Serialize)]
pub struct StrutSolution {
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub link_length_errors_m: [f64; 3],
    pub coaxial_errors_m: [f64; 2],
    pub travel_error_m: f64,
    pub strut_length_m: f64,
    /// Geometric extension from the reference; not a spring deformation law.
    pub strut_length_change_m: f64,
    pub iterations: u32,
}
impl StrutSolution {
    pub fn pose(&self) -> Result<LinkPose> {
        LinkPose::new(self.position_m, self.orientation_xyzw)
    }
}
impl StrutLink {
    /// Anchors/attachments 0,1 locate the lower joint; 2 is the steering link.
    /// A fixed chassis top lies on an upright-local line through strut_point.
    /// The line direction is determined from the explicit reference pose.
    pub fn new(
        anchors_m: [[f64; 3]; 3],
        attachments_m: [[f64; 3]; 3],
        top_m: [f64; 3],
        strut_point_m: [f64; 3],
        reference: LinkPose,
        travel_axis: [f64; 3],
    ) -> Result<Self> {
        let mut anchors = [DVec3::ZERO; 3];
        let mut attachments = anchors;
        let mut lengths = [0.; 3];
        for i in 0..3 {
            anchors[i] = point(anchors_m[i])?;
            attachments[i] = point(attachments_m[i])?;
            lengths[i] =
                (reference.position + reference.rotation * attachments[i] - anchors[i]).length();
            if !(1e-6..=100.).contains(&lengths[i]) {
                return Err(error("Strut link lengths must lie within 1e-6..100 metres"));
            }
        }
        let top = point(top_m)?;
        let strut_point = point(strut_point_m)?;
        let local_offset =
            reference.rotation.conjugate() * (top - reference.position) - strut_point;
        let reference_length = local_offset.length();
        if !(1e-6..=100.).contains(&reference_length) {
            return Err(error(
                "Strut reference length must lie within 1e-6..100 metres",
            ));
        }
        let strut_axis = local_offset / reference_length;
        let helper = if strut_axis.x.abs() < 0.8 {
            DVec3::X
        } else {
            DVec3::Y
        };
        let normal = strut_axis.cross(helper).normalize();
        let axis = point(travel_axis)?;
        if (axis.length_squared() - 1.).abs() > 1e-8 {
            return Err(error("Travel axis must have unit length within 1e-8"));
        }
        let rotational_scale = attachments
            .iter()
            .map(|p| p.length())
            .fold(0.001, f64::max)
            .max(strut_point.length())
            .max((top - reference.position).length());
        let model = Self {
            anchors,
            attachments,
            lengths,
            top,
            strut_point,
            strut_axis,
            normals: [normal, strut_axis.cross(normal)],
            reference,
            axis: axis.normalize(),
            rotational_scale,
            reference_length,
        };
        let (_, jacobian) = model.system(reference, 0., DVec3::ZERO)?;
        linear(jacobian, [0.; 6])?;
        Ok(model)
    }
    pub fn reference_pose(&self) -> LinkPose {
        self.reference
    }
    pub fn rest_lengths_m(&self) -> [f64; 3] {
        self.lengths
    }
    pub fn reference_strut_length_m(&self) -> f64 {
        self.reference_length
    }
    pub fn differential(
        &self,
        travel_m: f64,
        steering_anchor_offset_m: [f64; 3],
        pose: LinkPose,
    ) -> Result<crate::LinkDifferential> {
        let steering = crate::linkage::command(travel_m, steering_anchor_offset_m)?;
        crate::linkage::local_pose(self.reference, pose)?;
        let (residual, jacobian) = self.system(pose, travel_m, steering)?;
        crate::LinkDifferential::new(pose, self.rotational_scale, residual, jacobian, 2)
    }
    fn axial_length(&self, pose: LinkPose) -> f64 {
        (pose.rotation * self.strut_axis)
            .dot(self.top - pose.position - pose.rotation * self.strut_point)
    }
    pub fn acceleration(
        &self,
        travel_m: f64,
        rack_m: [f64; 3],
        pose: LinkPose,
        rates: [f64; 4],
        accelerations: [f64; 4],
    ) -> Result<crate::RelativeAcceleration> {
        let map = self.differential(travel_m, rack_m, pose)?;
        let velocity = map.velocity(rates[0], rates[1..].try_into().unwrap())?;
        let v = DVec3::from_array(velocity.velocity_m_s());
        let w = DVec3::from_array(velocity.angular_velocity_rad_s());
        let rack = DVec3::from_array(rack_m);
        let rack_v = DVec3::from_slice(&rates[1..]);
        let bias = std::array::from_fn(|i| {
            if i < 3 {
                let lever = pose.rotation * self.attachments[i];
                let delta = pose.position + lever
                    - self.anchors[i]
                    - if i == 2 { rack } else { DVec3::ZERO };
                crate::differential::rod_bias(
                    lever,
                    delta,
                    v,
                    w,
                    if i == 2 { rack_v } else { DVec3::ZERO },
                )
            } else {
                let normal = pose.rotation * self.normals[i - 3];
                w.cross(w.cross(normal)).dot(self.top - pose.position) - 2. * w.cross(normal).dot(v)
            }
        });
        map.solve_acceleration(rates, accelerations, bias)
    }
    fn system(&self, pose: LinkPose, travel: f64, steering: DVec3) -> Result<System> {
        let mut residual = [0.; 6];
        let mut jacobian = [[0.; 6]; 6];
        for i in 0..3 {
            let lever = pose.rotation * self.attachments[i];
            let offset = pose.position + lever
                - self.anchors[i]
                - if i == 2 { steering } else { DVec3::ZERO };
            let length = offset.length();
            if !length.is_finite() || length < 1e-9 {
                return Err(error("Collapsed/nonfinite strut linkage distance"));
            }
            let u = offset / length;
            let rotation = lever.cross(u) / self.rotational_scale;
            residual[i] = length - self.lengths[i];
            jacobian[i] = [u.x, u.y, u.z, rotation.x, rotation.y, rotation.z];
        }
        let axial = self.axial_length(pose);
        if !(1e-6..=100.).contains(&axial) {
            return Err(error(
                "Strut axis collapsed, reversed or outside 1e-6..100 metres",
            ));
        }
        let offset = self.top - pose.position - pose.rotation * self.strut_point;
        for (i, normal) in self.normals.iter().enumerate() {
            let normal = pose.rotation * *normal;
            residual[3 + i] = normal.dot(offset);
            let rotation = normal.cross(self.top - pose.position) / self.rotational_scale;
            jacobian[3 + i] = [
                -normal.x, -normal.y, -normal.z, rotation.x, rotation.y, rotation.z,
            ];
        }
        residual[5] = self.axis.dot(pose.position - self.reference.position) - travel;
        jacobian[5][..3].copy_from_slice(&self.axis.to_array());
        Ok((residual, jacobian))
    }
    pub fn solve(
        &self,
        travel_m: f64,
        steering_anchor_offset_m: [f64; 3],
        seed: Option<LinkPose>,
    ) -> Result<StrutSolution> {
        let (pose, residual, iterations) = solve_local(
            self.reference,
            self.rotational_scale,
            travel_m,
            steering_anchor_offset_m,
            seed,
            |pose, travel, steering| self.system(pose, travel, steering),
        )?;
        let length = self.axial_length(pose);
        Ok(StrutSolution {
            position_m: pose.position_m(),
            orientation_xyzw: pose.orientation_xyzw(),
            link_length_errors_m: residual[..3].try_into().unwrap(),
            coaxial_errors_m: residual[3..5].try_into().unwrap(),
            travel_error_m: residual[5],
            strut_length_m: length,
            strut_length_change_m: length - self.reference_length,
            iterations,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DQuat;
    #[test]
    fn strut_constraint_tangent_matches_central_differences_away_from_reference() {
        let reference = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
        let model = StrutLink::new(
            [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]],
            [[0.; 3], [0.; 3], [0., 0., 0.15]],
            [0., 0.6, 0.],
            [0.05, -0.1, 0.03],
            reference,
            [0., 1., 0.],
        )
        .unwrap();
        let pose = LinkPose::new(
            [0.003, 0.04, -0.002],
            DQuat::from_scaled_axis(DVec3::new(0.02, 0.03, -0.01)).to_array(),
        )
        .unwrap();
        let steering = DVec3::new(0.004, 0.001, -0.002);
        let (_, jacobian) = model.system(pose, 0.04, steering).unwrap();
        let h = 1e-6;
        for col in 0..6 {
            let mut low = pose;
            let mut high = pose;
            if col < 3 {
                low.position[col] -= h;
                high.position[col] += h;
            } else {
                let mut angle = DVec3::ZERO;
                angle[col - 3] = h / model.rotational_scale;
                low.rotation = DQuat::from_scaled_axis(-angle) * pose.rotation;
                high.rotation = DQuat::from_scaled_axis(angle) * pose.rotation;
            }
            let low = model.system(low, 0.04, steering).unwrap().0;
            let high = model.system(high, 0.04, steering).unwrap().0;
            for row in 0..6 {
                assert!(
                    ((high[row] - low[row]) / (2. * h) - jacobian[row][col]).abs() < 1e-9,
                    "row {row} col {col}"
                );
            }
        }
    }
    #[test]
    fn coaxial_constraint_rejects_collapsed_or_reversed_axis_directly() {
        let reference = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
        let model = StrutLink::new(
            [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]],
            [[0.; 3], [0.; 3], [0., 0., 0.15]],
            [0., 0.6, 0.],
            [0.; 3],
            reference,
            [0., 1., 0.],
        )
        .unwrap();
        let collapsed = LinkPose::new([0., 0.6, 0.], [0., 0., 0., 1.]).unwrap();
        assert!(model.system(collapsed, 0.6, DVec3::ZERO).is_err());
        let reversed = LinkPose::new(
            [0.; 3],
            DQuat::from_rotation_x(std::f64::consts::PI).to_array(),
        )
        .unwrap();
        assert!(model.system(reversed, 0., DVec3::ZERO).is_err());
    }
}
