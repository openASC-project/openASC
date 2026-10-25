//! Prescribed reference translation and shortest-arc rotation, with a conservative speed bound.
use super::{Bounds, OrientedBox};
use asc_config::{Error, Result};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
fn fail(message: &str) -> Error {
    Error::new("wall_motion_range", "<wall sweep>", 0, message)
}
#[derive(Clone, Debug)]
pub struct BoxMotion {
    start: DVec3,
    end: DVec3,
    rotation: DQuat,
    relative_axis_angle: DVec3,
    local_center: DVec3,
    half: DVec3,
    radius: f64,
}
#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct WallSweepSettings {
    pub distance_tolerance_m: f64,
    pub max_iterations: usize,
}
impl Default for WallSweepSettings {
    fn default() -> Self {
        Self {
            distance_tolerance_m: 1e-6,
            max_iterations: 512,
        }
    }
}
impl WallSweepSettings {
    pub(super) fn validate(self) -> Result<()> {
        if !self.distance_tolerance_m.is_finite()
            || !(1e-9..=1e-3).contains(&self.distance_tolerance_m)
            || !(1..=4096).contains(&self.max_iterations)
        {
            return Err(fail(
                "Sweep requires tolerance in 1e-9..1e-3 m and 1..4096 iterations",
            ));
        }
        Ok(())
    }
}
impl BoxMotion {
    /// The box centre may be offset from the translating/rotating reference (e.g. chassis COM).
    /// Rotation follows the shortest quaternion arc, at most pi radians, over fraction 0..1.
    pub fn new(
        reference_start_world_m: [f64; 3],
        reference_end_world_m: [f64; 3],
        orientation_start_xyzw: [f64; 4],
        orientation_end_xyzw: [f64; 4],
        box_center_in_reference_m: [f64; 3],
        half_extent_m: [f64; 3],
    ) -> Result<Self> {
        let start = DVec3::from_array(reference_start_world_m);
        let end = DVec3::from_array(reference_end_world_m);
        let rotation = DQuat::from_array(orientation_start_xyzw);
        let end_rotation = DQuat::from_array(orientation_end_xyzw);
        let local_center = DVec3::from_array(box_center_in_reference_m);
        for v in [start, end] {
            if !v.is_finite() || v.abs().max_element() > 1e7 {
                return Err(fail("Sweep reference positions exceed +/-10000000 m"));
            }
        }
        if !local_center.is_finite() || local_center.abs().max_element() > 20. {
            return Err(fail("Sweep local box centre exceeds +/-20 m"));
        }
        for q in [rotation, end_rotation] {
            if !q.is_finite() || (q.length_squared() - 1.).abs() > 1e-8 {
                return Err(fail("Sweep requires unit quaternions within 1e-8"));
            }
        }
        let rotation = rotation.normalize();
        let mut relative = rotation.conjugate() * end_rotation.normalize();
        // q and -q represent the same endpoint. Canonicalize the pi tie as well.
        let first = [relative.w, relative.x, relative.y, relative.z]
            .into_iter()
            .find(|v| v.abs() > 1e-15)
            .unwrap_or(1.);
        if first < 0. {
            relative = -relative;
        }
        let vector = DVec3::new(relative.x, relative.y, relative.z);
        let length = vector.length();
        let relative_axis_angle = if length > 0. {
            vector * (2. * length.atan2(relative.w.max(0.)) / length)
        } else {
            DVec3::ZERO
        };
        let half = DVec3::from_array(half_extent_m);
        let motion = Self {
            start,
            end,
            rotation,
            relative_axis_angle,
            local_center,
            half,
            radius: local_center.length() + half.length(),
        };
        motion.at(0.)?;
        motion.at(1.)?;
        motion.bounds(0.)?;
        Ok(motion)
    }
    pub fn at(&self, fraction: f64) -> Result<OrientedBox> {
        if !fraction.is_finite() || !(0. ..=1.).contains(&fraction) {
            return Err(fail("Sweep fraction must lie in 0..1"));
        }
        let q = self.rotation * DQuat::from_scaled_axis(self.relative_axis_angle * fraction);
        let center = self.start.lerp(self.end, fraction) + q * self.local_center;
        OrientedBox::new(center.to_array(), self.half.to_array(), q.to_array())
    }
    pub(super) fn speed_bound(&self) -> f64 {
        (self.end - self.start).length() + self.relative_axis_angle.length() * self.radius
    }
    pub(super) fn bounds(&self, tolerance: f64) -> Result<Bounds> {
        let radius = self.radius + tolerance + 1e-9;
        let lo = self.start.min(self.end) - DVec3::splat(radius);
        let hi = self.start.max(self.end) + DVec3::splat(radius);
        if lo.abs().max_element().max(hi.abs().max_element()) > 1e7 {
            return Err(fail("Conservative sweep bounds exceed +/-10000000 m"));
        }
        Ok(Bounds {
            lo: lo.to_array(),
            hi: hi.to_array(),
        })
    }
}
