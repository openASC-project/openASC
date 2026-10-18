//! Position-level rigid upright geometry; no suspension force/mass model.
use crate::{Error, Result};
use glam::{DQuat, DVec3};
use serde::Serialize;

const TOLERANCE: f64 = 1e-10;
#[derive(Clone, Copy, Debug)]
pub struct LinkPose {
    pub(super) position: DVec3,
    pub(super) rotation: DQuat,
}
impl LinkPose {
    pub fn new(position_m: [f64; 3], orientation_xyzw: [f64; 4]) -> Result<Self> {
        let position = point(position_m)?;
        let rotation = DQuat::from_array(orientation_xyzw);
        if !rotation.is_finite() || (rotation.length_squared() - 1.).abs() > 1e-8 {
            return Err(error(
                "Expected a finite unit upright quaternion within 1e-8",
            ));
        }
        Ok(Self {
            position,
            rotation: rotation.normalize(),
        })
    }
    pub fn position_m(self) -> [f64; 3] {
        self.position.to_array()
    }
    pub fn orientation_xyzw(self) -> [f64; 4] {
        self.rotation.to_array()
    }
    pub fn transform(self, attachment: [f64; 3]) -> Result<[f64; 3]> {
        Ok((self.position + self.rotation * point(attachment)?).to_array())
    }
}
#[derive(Debug, Serialize)]
pub struct LinkSolution {
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub link_length_errors_m: [f64; 5],
    pub travel_error_m: f64,
    pub iterations: u32,
}
impl LinkSolution {
    pub fn pose(&self) -> Result<LinkPose> {
        LinkPose::new(self.position_m, self.orientation_xyzw)
    }
}
/// Five fixed-length rods constrain one rigid upright. The origin of its local
/// coordinates is the caller's chosen travel point. Only anchor 4 is steered.
#[derive(Clone, Debug)]
pub struct FiveLink {
    anchors: [DVec3; 5],
    attachments: [DVec3; 5],
    lengths: [f64; 5],
    reference: LinkPose,
    axis: DVec3,
    rotational_scale: f64,
}
pub(super) fn error(message: &str) -> Error {
    Error::new("physics_linkage", message)
}
pub(super) fn point(value: [f64; 3]) -> Result<DVec3> {
    let value = DVec3::from_array(value);
    if !value.is_finite() || value.abs().max_element() > 100. {
        return Err(error(
            "Expected finite linkage coordinates within +/-100 metres",
        ));
    }
    Ok(value)
}
pub(super) type System = ([f64; 6], [[f64; 6]; 6]);
impl FiveLink {
    pub fn new(
        anchors_m: [[f64; 3]; 5],
        attachments_m: [[f64; 3]; 5],
        reference: LinkPose,
        travel_axis: [f64; 3],
    ) -> Result<Self> {
        let mut anchors = [DVec3::ZERO; 5];
        let mut attachments = anchors;
        let mut lengths = [0.; 5];
        for i in 0..5 {
            anchors[i] = point(anchors_m[i])?;
            attachments[i] = point(attachments_m[i])?;
            lengths[i] =
                (reference.position + reference.rotation * attachments[i] - anchors[i]).length();
            if !(1e-6..=100.).contains(&lengths[i]) {
                return Err(error("Link lengths must lie within 1e-6..100 metres"));
            }
        }
        let axis = point(travel_axis)?;
        if (axis.length_squared() - 1.).abs() > 1e-8 {
            return Err(error("Travel axis must have unit length within 1e-8"));
        }
        let rotational_scale = attachments.iter().map(|p| p.length()).fold(0.001, f64::max);
        let model = Self {
            anchors,
            attachments,
            lengths,
            reference,
            axis: axis.normalize(),
            rotational_scale,
        };
        // Even a zero-residual reference must have a nonsingular local mechanism.
        let (_, jacobian) = model.system(reference, 0., DVec3::ZERO)?;
        linear(jacobian, [0.; 6])?;
        Ok(model)
    }
    pub fn reference_pose(&self) -> LinkPose {
        self.reference
    }
    pub fn rest_lengths_m(&self) -> [f64; 5] {
        self.lengths
    }
    fn system(&self, pose: LinkPose, travel: f64, steering: DVec3) -> Result<System> {
        let mut residual = [0.; 6];
        let mut jacobian = [[0.; 6]; 6];
        for i in 0..5 {
            let lever = pose.rotation * self.attachments[i];
            let offset = pose.position + lever
                - self.anchors[i]
                - if i == 4 { steering } else { DVec3::ZERO };
            let length = offset.length();
            if !length.is_finite() || length < 1e-9 {
                return Err(error("Collapsed/nonfinite link during solve"));
            }
            let direction = offset / length;
            residual[i] = length - self.lengths[i];
            let rotation = lever.cross(direction) / self.rotational_scale;
            jacobian[i] = [
                direction.x,
                direction.y,
                direction.z,
                rotation.x,
                rotation.y,
                rotation.z,
            ];
        }
        residual[5] = self.axis.dot(pose.position - self.reference.position) - travel;
        jacobian[5][..3].copy_from_slice(&self.axis.to_array());
        Ok((residual, jacobian))
    }
    /// Bounded local inverse kinematics. No state is mutated on failure.
    /// Seeds select a nearby assembly branch; folds/global root search are unsupported.
    pub fn solve(
        &self,
        travel_m: f64,
        steering_anchor_offset_m: [f64; 3],
        seed: Option<LinkPose>,
    ) -> Result<LinkSolution> {
        let (pose, residual, iterations) = solve_local(
            self.reference,
            self.rotational_scale,
            travel_m,
            steering_anchor_offset_m,
            seed,
            |pose, travel, steering| self.system(pose, travel, steering),
        )?;
        Ok(LinkSolution {
            position_m: pose.position_m(),
            orientation_xyzw: pose.orientation_xyzw(),
            link_length_errors_m: residual[..5].try_into().unwrap(),
            travel_error_m: residual[5],
            iterations,
        })
    }
}

/// Shared bounded Newton iteration; each topology supplies its own constraints.
pub(super) fn solve_local(
    reference: LinkPose,
    rotational_scale: f64,
    travel_m: f64,
    steering_anchor_offset_m: [f64; 3],
    seed: Option<LinkPose>,
    system: impl Fn(LinkPose, f64, DVec3) -> Result<System>,
) -> Result<(LinkPose, [f64; 6], u32)> {
    let steering = point(steering_anchor_offset_m)?;
    if !travel_m.is_finite() || travel_m.abs() > 0.25 || steering.length() > 0.25 {
        return Err(error(
            "Travel and steering anchor displacement are limited to +/-0.25 metres",
        ));
    }
    let mut pose = seed.unwrap_or(reference);
    for iteration in 0..=40 {
        if !pose.position.is_finite()
            || pose.position.abs().max_element() > 100.
            || (pose.position - reference.position).length() > 0.5
            || pose.rotation.angle_between(reference.rotation) > 1.
        {
            return Err(error(
                "Solution/seed exceeds the local 0.5 m / 1 radian assembly branch",
            ));
        }
        let (residual, jacobian) = system(pose, travel_m, steering)?;
        let step = linear(jacobian, residual.map(|r| -r))?;
        if residual.iter().all(|r| r.abs() <= TOLERANCE) {
            return Ok((pose, residual, iteration));
        }
        if iteration == 40 {
            break;
        }
        let translation = DVec3::new(step[0], step[1], step[2]);
        let rotation = DVec3::new(step[3], step[4], step[5]) / rotational_scale;
        let mut fraction =
            (0.05 / translation.length().max(0.05)).min(0.15 / rotation.length().max(0.15));
        let objective: f64 = residual.iter().map(|v| v * v).sum();
        let mut accepted = None;
        for _ in 0..12 {
            let candidate = LinkPose {
                position: pose.position + fraction * translation,
                rotation: (DQuat::from_scaled_axis(fraction * rotation) * pose.rotation)
                    .normalize(),
            };
            if let Ok((candidate_residual, _)) = system(candidate, travel_m, steering) {
                let next: f64 = candidate_residual.iter().map(|v| v * v).sum();
                if next < objective {
                    accepted = Some(candidate);
                    break;
                }
            }
            fraction *= 0.5;
        }
        pose = accepted
            .ok_or_else(|| error("Local linkage Newton search made no residual progress"))?;
    }
    Err(error("Local linkage solve exceeded 40 iterations"))
}

/// Partial-pivot elimination of column-scaled constraints; small pivots reject
/// singular/near-singular mechanisms instead of inventing a fallback pose.
pub(super) fn linear(mut matrix: [[f64; 6]; 6], mut rhs: [f64; 6]) -> Result<[f64; 6]> {
    for col in 0..6 {
        let row = (col..6)
            .max_by(|a, b| matrix[*a][col].abs().total_cmp(&matrix[*b][col].abs()))
            .unwrap();
        if !matrix[row][col].is_finite() || matrix[row][col].abs() < 1e-10 {
            return Err(error(
                "Singular or poorly conditioned local linkage Jacobian",
            ));
        }
        matrix.swap(row, col);
        rhs.swap(row, col);
        for row in col + 1..6 {
            let factor = matrix[row][col] / matrix[col][col];
            let pivot = matrix[col];
            for (entry, pivot) in matrix[row].iter_mut().zip(pivot).skip(col) {
                *entry -= factor * pivot;
            }
            rhs[row] -= factor * rhs[col];
        }
    }
    let mut result = [0.; 6];
    for row in (0..6).rev() {
        let known: f64 = (row + 1..6).map(|j| matrix[row][j] * result[j]).sum();
        result[row] = (rhs[row] - known) / matrix[row][row];
    }
    if result.iter().any(|v| !v.is_finite()) {
        return Err(error("Nonfinite linkage Newton increment"));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn analytic_tangent_matches_independent_central_differences() {
        let reference = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
        let model = FiveLink::new(
            [
                [-0.3, 0.2, 0.12],
                [-0.3, 0.2, -0.12],
                [-0.3, -0.2, 0.12],
                [-0.3, -0.2, -0.12],
                [-0.3, 0., 0.15],
            ],
            [
                [0., 0.2, 0.],
                [0., 0.2, 0.],
                [0., -0.2, 0.],
                [0., -0.2, 0.],
                [0., 0., 0.15],
            ],
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
}
