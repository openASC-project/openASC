//! Finite triangle versus arbitrarily oriented box. Double-sided SAT and clipped contact patch.
use asc_config::{Error, Result};
use glam::{DQuat, DVec3};
use serde::Serialize;
fn fail(message: &str) -> Error {
    Error::new("wall_box_range", "<wall box query>", 0, message)
}
#[derive(Clone, Copy, Debug)]
pub struct OrientedBox {
    center: DVec3,
    half: DVec3,
    rotation: DQuat,
}
#[derive(Clone, Debug, Serialize)]
pub struct BoxTriangleContact {
    /// SAT separation, a lower bound on distance; negative is overlap along the selected axis.
    pub separation_m: f64,
    /// Unit world normal from the triangle toward the box.
    pub normal_world: [f64; 3],
    /// Triangle patch clipped to the box with an axis-wise margin; at most nine vertices ordinarily.
    pub points_world_m: Vec<[f64; 3]>,
}
impl OrientedBox {
    pub fn new(
        center_world_m: [f64; 3],
        half_extent_m: [f64; 3],
        orientation_xyzw: [f64; 4],
    ) -> Result<Self> {
        let center = DVec3::from_array(center_world_m);
        let half = DVec3::from_array(half_extent_m);
        let rotation = DQuat::from_array(orientation_xyzw);
        if !center.is_finite()
            || center.abs().max_element() > 1e7
            || !half.is_finite()
            || half.min_element() < 1e-4
            || half.max_element() > 10.
            || !rotation.is_finite()
            || (rotation.length_squared() - 1.).abs() > 1e-8
        {
            return Err(fail(
                "Expected a bounded centre, half extents in 0.0001..10 m and a unit quaternion within 1e-8",
            ));
        }
        Ok(Self {
            center,
            half,
            rotation: rotation.normalize(),
        })
    }
    pub fn aabb(&self, margin_m: f64) -> Result<([f64; 3], [f64; 3])> {
        margin(margin_m)?;
        let axes = [
            self.rotation * DVec3::X,
            self.rotation * DVec3::Y,
            self.rotation * DVec3::Z,
        ];
        let half = axes[0].abs() * self.half.x
            + axes[1].abs() * self.half.y
            + axes[2].abs() * self.half.z
            + DVec3::splat(margin_m);
        if (self.center.abs() + half).max_element() > 1e7 {
            return Err(fail("World box bounds exceed +/-10000000 m"));
        }
        Ok((self.center.to_array(), half.to_array()))
    }
    /// Static overlap/margin query, not a sweep or a continuous collision guarantee.
    pub fn contact_triangle(
        &self,
        triangle_world_m: [[f64; 3]; 3],
        margin_m: f64,
    ) -> Result<Option<BoxTriangleContact>> {
        margin(margin_m)?;
        self.aabb(margin_m)?;
        let mut p = [DVec3::ZERO; 3];
        for (i, point) in triangle_world_m.into_iter().enumerate() {
            let point = DVec3::from_array(point);
            if !point.is_finite() || point.abs().max_element() > 1e7 {
                return Err(fail("Triangle coordinates exceed the finite world range"));
            }
            p[i] = self.rotation.conjugate() * (point - self.center);
        }
        let edges = [p[1] - p[0], p[2] - p[1], p[0] - p[2]];
        let normal = edges[0].cross(p[2] - p[0]);
        if normal.length() <= 1e-12 * (edges[0].length() * (p[2] - p[0]).length()).max(1.) {
            return Err(fail(
                "Triangle is degenerate at the documented relative area threshold",
            ));
        }
        let mut axes = vec![normal.normalize(), DVec3::X, DVec3::Y, DVec3::Z];
        for basis in [DVec3::X, DVec3::Y, DVec3::Z] {
            for edge in edges {
                let cross = basis.cross(edge);
                if cross.length_squared() > 1e-24 * edge.length_squared() {
                    axes.push(cross.normalize());
                }
            }
        }
        for axis in &mut axes {
            let first = axis
                .to_array()
                .into_iter()
                .find(|x| x.abs() > 1e-12)
                .unwrap_or(1.);
            if first < 0. {
                *axis = -*axis;
            }
        }
        axes[4..].sort_by(|a, b| {
            a.x.total_cmp(&b.x)
                .then(a.y.total_cmp(&b.y))
                .then(a.z.total_cmp(&b.z))
        });
        let mut best = f64::NEG_INFINITY;
        let mut direction = DVec3::ZERO;
        for axis in axes {
            let projection = p.map(|p| axis.dot(p));
            let low = projection.into_iter().fold(f64::INFINITY, f64::min);
            let high = projection.into_iter().fold(f64::NEG_INFINITY, f64::max);
            let radius = axis.abs().dot(self.half);
            let a = low - radius;
            let b = -radius - high;
            let gap = a.max(b);
            if gap > margin_m + 1e-9 {
                return Ok(None);
            }
            if gap > best + 1e-10 {
                best = gap;
                direction = if a >= b { -axis } else { axis };
            }
        }
        let mut polygon = p.to_vec();
        let expanded = self.half + DVec3::splat(margin_m + 1e-9);
        for axis in 0..3 {
            for sign in [-1., 1.] {
                let input = std::mem::take(&mut polygon);
                if input.is_empty() {
                    return Err(fail("SAT overlap produced an empty clipped patch"));
                }
                let mut previous = *input.last().unwrap();
                let mut previous_d = sign * previous[axis] - expanded[axis];
                for current in input {
                    let current_d = sign * current[axis] - expanded[axis];
                    if (current_d <= 0.) != (previous_d <= 0.) {
                        let fraction = previous_d / (previous_d - current_d);
                        polygon.push(previous + (current - previous) * fraction);
                    }
                    if current_d <= 0. {
                        polygon.push(current);
                    }
                    previous = current;
                    previous_d = current_d;
                }
                if polygon.len() > 32 {
                    return Err(fail("Clipped patch exceeds 32 vertices"));
                }
            }
        }
        let mut points = Vec::new();
        for point in polygon {
            if points
                .iter()
                .all(|p: &DVec3| (*p - point).length_squared() > 1e-18)
            {
                points.push(point);
            }
        }
        if points.is_empty() {
            return Err(fail("Clipped patch has no retained contact points"));
        }
        // Source winding affects patch ordering only; sort for a deterministic public report.
        points.sort_by(|a, b| {
            a.x.total_cmp(&b.x)
                .then(a.y.total_cmp(&b.y))
                .then(a.z.total_cmp(&b.z))
        });
        Ok(Some(BoxTriangleContact {
            separation_m: best,
            normal_world: (self.rotation * direction).to_array(),
            points_world_m: points
                .into_iter()
                .map(|p| (self.center + self.rotation * p).to_array())
                .collect(),
        }))
    }
}
fn margin(value: f64) -> Result<()> {
    if !value.is_finite() || !(0. ..=0.1).contains(&value) {
        return Err(fail("SAT margin must lie within 0..0.1 m"));
    }
    Ok(())
}
