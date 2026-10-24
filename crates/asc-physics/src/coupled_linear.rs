//! Diagonally normalized positive-definite solve for at most fourteen coordinates.
use crate::{Error, Result};
fn failure() -> Error {
    Error::new(
        "physics_chassis_matrix",
        "Nonfinite, nonpositive or poorly conditioned coupled mass matrix",
    )
}
pub(super) struct Factor {
    matrix: [[f64; 14]; 14],
    lower: [[f64; 14]; 14],
    scale: [f64; 14],
    n: usize,
}
pub(super) fn factor(matrix: [[f64; 14]; 14], n: usize) -> Result<Factor> {
    if !(7..=14).contains(&n) {
        return Err(failure());
    }
    let mut scale = [0.; 14];
    for i in 0..n {
        if !matrix[i][i].is_finite() || !(1e-12..=1e24).contains(&matrix[i][i]) {
            return Err(failure());
        }
        scale[i] = matrix[i][i].sqrt();
    }
    let mut lower = [[0.; 14]; 14];
    for i in 0..n {
        for j in 0..=i {
            let value = matrix[i][j] / scale[i] / scale[j];
            let transposed = matrix[j][i] / scale[i] / scale[j];
            if !value.is_finite() || !transposed.is_finite() || (value - transposed).abs() > 1e-10 {
                return Err(failure());
            }
            let value =
                0.5 * (value + transposed) - (0..j).map(|k| lower[i][k] * lower[j][k]).sum::<f64>();
            if i == j {
                if value <= 1e-10 || !value.is_finite() {
                    return Err(failure());
                }
                lower[i][j] = value.sqrt();
            } else {
                lower[i][j] = value / lower[j][j];
            }
        }
    }
    Ok(Factor {
        matrix,
        lower,
        scale,
        n,
    })
}
impl Factor {
    pub(super) fn solve(&self, rhs: [f64; 14]) -> Result<[f64; 14]> {
        let n = self.n;
        let l = &self.lower;
        let scale = &self.scale;
        let mut y = [0.; 14];
        for i in 0..n {
            y[i] = (rhs[i] / scale[i] - (0..i).map(|j| l[i][j] * y[j]).sum::<f64>()) / l[i][i];
        }
        let mut z = [0.; 14];
        for i in (0..n).rev() {
            z[i] = (y[i] - (i + 1..n).map(|j| l[j][i] * z[j]).sum::<f64>()) / l[i][i];
        }
        let result = std::array::from_fn(|i| if i < n { z[i] / scale[i] } else { 0. });
        if result.iter().any(|v| !v.is_finite()) {
            return Err(failure());
        }
        for (i, load) in rhs.iter().enumerate().take(n) {
            let sum = (0..n).map(|j| self.matrix[i][j] * result[j]).sum::<f64>();
            let bound = load
                .abs()
                .max((0..n).map(|j| (self.matrix[i][j] * result[j]).abs()).sum())
                .max(1.);
            if !sum.is_finite() || (sum - load).abs() > 1e-8 * bound {
                return Err(failure());
            }
        }
        Ok(result)
    }
}
pub(super) fn solve(matrix: [[f64; 14]; 14], rhs: [f64; 14], n: usize) -> Result<[f64; 14]> {
    factor(matrix, n)?.solve(rhs)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn normalized_solve_handles_mixed_coordinate_scales_and_rejects_bad_systems() {
        for n in 7..=14 {
            let scale = [
                1e-5, 1e5, 10., 1e-3, 1e6, 0.3, 1e-4, 0.01, 1., 1e3, 0.2, 2., 20., 200.,
            ];
            let mut l = [[0.; 14]; 14];
            for (i, row) in l.iter_mut().enumerate().take(n) {
                for (j, value) in row.iter_mut().enumerate().take(i + 1) {
                    *value = if i == j { 1. } else { 0.02 * (i + j) as f64 };
                }
            }
            let matrix = std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    (0..n).map(|k| l[i][k] * l[j][k]).sum::<f64>() * scale[i] * scale[j]
                })
            });
            let target =
                std::array::from_fn::<_, 14, _>(|i| if i < n { 0.1 / scale[i] } else { 0. });
            let rhs = std::array::from_fn(|i| (0..n).map(|j| matrix[i][j] * target[j]).sum());
            let result = solve(matrix, rhs, n).unwrap();
            for i in 0..n {
                assert!((result[i] - target[i]).abs() < target[i].abs() * 1e-9);
            }
            let mut negative = matrix;
            negative[0][0] = -1.;
            assert!(solve(negative, rhs, n).is_err());
            let mut asymmetric = matrix;
            asymmetric[0][1] += scale[0] * scale[1] * 0.1;
            assert!(solve(asymmetric, rhs, n).is_err());
            let mut bad_rhs = rhs;
            bad_rhs[0] = f64::NAN;
            assert!(solve(matrix, bad_rhs, n).is_err());
        }
        let matrix = [[1.; 14]; 14];
        assert!(solve(matrix, [0.; 14], 7).is_err());
    }
}
