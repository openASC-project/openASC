use crate::{Error, Result};
use glam::{DMat3, DVec3};

#[derive(Clone, Debug)]
pub struct MassProperties {
    pub(crate) mass: f64,
    pub(crate) inertia: DMat3,
    pub(crate) inverse: DMat3,
}
impl MassProperties {
    /// Full body-space tensor about the centre of mass, supplied in rows (kg m^2).
    pub fn new(mass_kg: f64, rows: [[f64; 3]; 3]) -> Result<Self> {
        let fail = || {
            Error::new(
                "physics_inertia",
                "Expected bounded positive mass and a symmetric, well-conditioned, physically realizable inertia tensor about the centre of mass.",
            )
        };
        if !mass_kg.is_finite()
            || !(1e-9..=1e12).contains(&mass_kg)
            || rows.iter().flatten().any(|v| !v.is_finite())
        {
            return Err(fail());
        }
        let scale = rows.iter().flatten().map(|v| v.abs()).fold(0f64, f64::max);
        if !(1e-12..=1e18).contains(&scale) {
            return Err(fail());
        }
        let mut a = [[0.; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                if (rows[i][j] / scale - rows[j][i] / scale).abs() > 1e-12 {
                    return Err(fail());
                }
                a[i][j] = (rows[i][j] / scale + rows[j][i] / scale) * 0.5;
            }
        }
        // Cholesky on the scaled tensor, including non-diagonal products of inertia.
        let mut l = [[0.; 3]; 3];
        for i in 0..3 {
            for j in 0..=i {
                let value = a[i][j] - (0..j).map(|k| l[i][k] * l[j][k]).sum::<f64>();
                if i == j {
                    if value <= 1e-12 {
                        return Err(fail());
                    }
                    l[i][j] = value.sqrt();
                } else {
                    l[i][j] = value / l[j][j];
                }
            }
        }
        let normalized = DMat3::from_cols_array(&[
            a[0][0], a[1][0], a[2][0], a[0][1], a[1][1], a[2][1], a[0][2], a[1][2], a[2][2],
        ]);
        let inverse = normalized.inverse();
        let norm = |m: DMat3| m.to_cols_array().iter().map(|v| v * v).sum::<f64>().sqrt();
        if !inverse.is_finite() || norm(normalized) * norm(inverse) > 1e12 {
            return Err(fail());
        }
        // C = trace(I)/2 * identity - I must be positive semidefinite:
        // this enforces principal-moment triangle inequalities for a mass distribution.
        let half_trace = (a[0][0] + a[1][1] + a[2][2]) * 0.5;
        let c = DMat3::from_diagonal(DVec3::splat(half_trace)) - normalized;
        let mut covariance = [[0.; 3]; 3];
        for (i, row) in covariance.iter_mut().enumerate() {
            for (j, value) in row.iter_mut().enumerate() {
                *value = c.col(j)[i] + if i == j { 1e-12 } else { 0. };
            }
        }
        let mut factor = [[0.; 3]; 3];
        for i in 0..3 {
            for j in 0..=i {
                let value =
                    covariance[i][j] - (0..j).map(|k| factor[i][k] * factor[j][k]).sum::<f64>();
                if i == j {
                    if value <= 0. {
                        return Err(fail());
                    }
                    factor[i][j] = value.sqrt();
                } else {
                    factor[i][j] = value / factor[j][j];
                }
            }
        }
        Ok(Self {
            mass: mass_kg,
            inertia: normalized * scale,
            inverse: inverse / scale,
        })
    }
    pub fn mass_kg(&self) -> f64 {
        self.mass
    }
    pub fn inertia_rows_kg_m2(&self) -> [[f64; 3]; 3] {
        std::array::from_fn(|i| std::array::from_fn(|j| self.inertia.col(j)[i]))
    }
}
