//! Rigid body-to-world roots. Geometry/UV buffers remain immutable.
use crate::gpu::scene::Result;
use glam::{DMat3, DMat4, DQuat, DVec3};
use serde::Serialize;

#[derive(Clone, Copy, Debug, Serialize)]
pub struct RigidPose {
    position: [f64; 3],
    orientation_xyzw: [f64; 4],
}
impl Default for RigidPose {
    fn default() -> Self {
        Self {
            position: [0.; 3],
            orientation_xyzw: [0., 0., 0., 1.],
        }
    }
}
impl RigidPose {
    pub fn new(position: [f64; 3], orientation_xyzw: [f64; 4]) -> Result<Self> {
        let q = DQuat::from_array(orientation_xyzw);
        if position.iter().any(|v| !v.is_finite() || v.abs() > 1e7)
            || !q.is_finite()
            || (q.length_squared() - 1.).abs() > 1e-6
        {
            return Err(
                "Root pose requires a finite position within +/-10000000 m and a unit quaternion."
                    .into(),
            );
        }
        Ok(Self {
            position,
            orientation_xyzw: q.normalize().to_array(),
        })
    }
    pub fn from_basis(
        position: [f64; 3],
        left: [f64; 3],
        up: [f64; 3],
        forward: [f64; 3],
    ) -> Result<Self> {
        let axes = [
            DVec3::from_array(left),
            DVec3::from_array(up),
            DVec3::from_array(forward),
        ];
        let basis = DMat3::from_cols(axes[0], axes[1], axes[2]);
        if !basis.is_finite()
            || axes.iter().any(|a| (a.length_squared() - 1.).abs() > 1e-6)
            || [(0, 1), (0, 2), (1, 2)]
                .iter()
                .any(|&(a, b)| axes[a].dot(axes[b]).abs() > 1e-6)
            || (basis.determinant() - 1.).abs() > 1e-6
        {
            return Err("Root basis must be rigid and right-handed within 1e-6; scale/skew are not repaired.".into());
        }
        Self::new(position, DQuat::from_mat3(&basis).normalize().to_array())
    }
    pub fn compose(self, local: Self) -> Result<Self> {
        let q = DQuat::from_array(self.orientation_xyzw);
        Self::new(
            (DVec3::from_array(self.position) + q * DVec3::from_array(local.position)).to_array(),
            (q * DQuat::from_array(local.orientation_xyzw)).to_array(),
        )
    }
    pub fn matrix(self) -> [f32; 16] {
        DMat4::from_rotation_translation(
            DQuat::from_array(self.orientation_xyzw),
            DVec3::from_array(self.position),
        )
        .as_mat4()
        .to_cols_array()
    }
    pub fn point(self, point: [f32; 3]) -> Result<[f32; 3]> {
        let p = DVec3::from_array(self.position)
            + DQuat::from_array(self.orientation_xyzw) * DVec3::from_array(point.map(f64::from));
        if !p.is_finite() || p.abs().max_element() > 1e7 {
            return Err("Transformed root geometry exceeds +/-10000000 metres.".into());
        }
        Ok(p.as_vec3().to_array())
    }
    pub fn bounds(self, bounds: [[f32; 3]; 2]) -> Result<[[f32; 3]; 2]> {
        if (0..3).any(|i| {
            !bounds[0][i].is_finite() || !bounds[1][i].is_finite() || bounds[0][i] > bounds[1][i]
        }) {
            return Err("Expected finite ordered root bounds.".into());
        }
        let mut result = [[f32::INFINITY; 3], [f32::NEG_INFINITY; 3]];
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    let p = self.point([bounds[x][0], bounds[y][1], bounds[z][2]])?;
                    for (i, v) in p.into_iter().enumerate() {
                        result[0][i] = result[0][i].min(v);
                        result[1][i] = result[1][i].max(v);
                    }
                }
            }
        }
        Ok(result)
    }
}
