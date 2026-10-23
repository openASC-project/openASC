//! Bounded local face views; no force law or infinite-plane extrapolation.
use super::*;
#[derive(Debug)]
pub struct Face<'a> {
    points: [[f64; 3]; 3],
    normal: [f64; 3],
    pub triangle: usize,
    pub mesh: &'a str,
    pub model: &'a str,
    pub surface: &'a Surface,
}
impl Face<'_> {
    pub fn points(&self) -> [[f64; 3]; 3] {
        self.points
    }
    pub fn normal(&self) -> [f64; 3] {
        self.normal
    }
    /// Plane tolerance 1e-7 m; projected XZ edge-distance tolerance 1e-8 m.
    pub fn contains_point(&self, p: [f64; 3]) -> Result<bool> {
        validate(p)?;
        let offset = std::array::from_fn::<_, 3, _>(|i| p[i] - self.points[0][i]);
        if offset
            .iter()
            .zip(self.normal)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            .abs()
            > 1e-7
        {
            return Ok(false);
        }
        let [a, b, c] = self.points;
        let area = (b[0] - a[0]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[0] - a[0]);
        let sign = area.signum();
        for (a, b) in [(a, b), (b, c), (c, a)] {
            let dx = b[0] - a[0];
            let dz = b[2] - a[2];
            let distance = sign * (dx * (p[2] - a[2]) - dz * (p[0] - a[0])) / dx.hypot(dz);
            if distance < -1e-8 {
                return Ok(false);
            }
        }
        Ok(true)
    }
}
fn validate(p: [f64; 3]) -> Result<()> {
    if p.iter().any(|v| !v.is_finite() || v.abs() > MAX_COORDINATE) {
        return Err(Error::new(
            "ground_face_range",
            "<ground face query>",
            0,
            "Expected finite coordinates within +/-10000000 m",
        ));
    }
    Ok(())
}
impl Ground {
    /// AABB overlap candidates only, sorted by source triangle index. At most
    /// 4096 candidates are retained; exceeding the cap is an error, not truncation.
    pub fn faces_in_box(&self, center: [f64; 3], half_extent: [f64; 3]) -> Result<Vec<Face<'_>>> {
        validate(center)?;
        if half_extent
            .iter()
            .any(|v| !v.is_finite() || !(0. ..=20.).contains(v))
        {
            return Err(Error::new(
                "ground_face_range",
                "<ground face query>",
                0,
                "Face query half extents must lie within 0..20 m",
            ));
        }
        let lo: [f64; 3] = std::array::from_fn(|i| center[i] - half_extent[i]);
        let hi: [f64; 3] = std::array::from_fn(|i| center[i] + half_extent[i]);
        let indices = spatial::candidates(
            &self.triangles,
            &self.order,
            &self.nodes,
            Bounds { lo, hi },
            || {
                Error::new(
                    "ground_face_limit",
                    "<ground face query>",
                    0,
                    "Local query exceeds 4096 face candidates",
                )
            },
        )?;
        Ok(indices
            .into_iter()
            .map(|index| {
                let triangle = &self.triangles[index];
                let [a, b, c] = triangle.points;
                let u: [f64; 3] = std::array::from_fn(|i| b[i] - a[i]);
                let v: [f64; 3] = std::array::from_fn(|i| c[i] - a[i]);
                let normal = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let length = normal[0].hypot(normal[1]).hypot(normal[2]);
                let sign = normal[1].signum();
                let mesh = &self.meshes[triangle.mesh];
                Face {
                    points: triangle.points,
                    normal: normal.map(|v| v * sign / length),
                    triangle: index,
                    mesh: &mesh.name,
                    model: &mesh.model,
                    surface: &mesh.surface,
                }
            })
            .collect())
    }
}
