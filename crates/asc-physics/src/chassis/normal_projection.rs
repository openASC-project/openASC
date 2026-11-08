//! Bounded normal complementarity projection shared by impulse and force constraints.
use super::*;
fn fail(message: &str) -> Error {
    Error::new("physics_normal_projection", message)
}
pub(super) struct NormalProjection {
    matrix: [[f64; 64]; 64],
    scales: Vec<f64>,
}
impl NormalProjection {
    pub(super) fn new(rows: &[[f64; 6]], responses: &[[f64; 6]]) -> Result<Self> {
        if rows.len() != responses.len() || rows.len() > 64 {
            return Err(fail("Expected at most 64 matching normal rows"));
        }
        let n = rows.len();
        let mut matrix = [[0.; 64]; 64];
        let mut scales = Vec::new();
        for i in 0..n {
            let diagonal = dot(rows[i], responses[i]);
            if !diagonal.is_finite() || !(1e-24..=1e24).contains(&diagonal) {
                return Err(fail("Nonpositive normal mobility"));
            }
            scales.push(diagonal.sqrt());
        }
        for i in 0..n {
            for j in 0..=i {
                let a = dot(rows[i], responses[j]) / scales[i] / scales[j];
                let b = dot(rows[j], responses[i]) / scales[i] / scales[j];
                if !a.is_finite() || !b.is_finite() || (a - b).abs() > 1e-8 {
                    return Err(fail("Nonreciprocal normal mobility"));
                }
                matrix[i][j] = 0.5 * (a + b);
                matrix[j][i] = matrix[i][j];
            }
        }
        Ok(Self { matrix, scales })
    }
    pub(super) fn solve(&self, speed: &[f64], tolerance: f64) -> Result<Vec<f64>> {
        let n = self.scales.len();
        if speed.len() != n
            || !tolerance.is_finite()
            || tolerance <= 0.
            || speed.iter().any(|v| !v.is_finite())
        {
            return Err(fail("Invalid normal projection speeds or tolerance"));
        }
        let matrix = &self.matrix;
        let scales = &self.scales;
        let mut velocity: Vec<_> = speed.iter().zip(scales).map(|(v, s)| v / s).collect();
        let mut lambda = vec![0.; n];
        let mut active = Vec::new();
        let mut operations = 0;
        loop {
            let next = (0..n)
                .filter(|i| !active.contains(i) && velocity[*i] * scales[*i] < -tolerance * 0.1)
                .min_by(|&a, &b| velocity[a].total_cmp(&velocity[b]));
            let Some(next) = next else {
                return Ok(lambda.iter().zip(scales).map(|(l, s)| l / s).collect());
            };
            loop {
                operations += 1;
                if operations > 4096 {
                    return Err(fail(
                        "Normal active-set projection exceeded its work budget",
                    ));
                }
                let rhs: Vec<_> = active.iter().map(|&i| matrix[i][next]).collect();
                let direction = active_solve(matrix, &active, &rhs)?;
                let curvature = matrix[next][next]
                    - rhs.iter().zip(&direction).map(|(a, b)| a * b).sum::<f64>();
                if !curvature.is_finite() || curvature < -1e-10 {
                    return Err(fail("Normal projection has negative curvature"));
                }
                let primal = if curvature > 1e-12 {
                    (-velocity[next] / curvature).max(0.)
                } else {
                    f64::INFINITY
                };
                let bound = active
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| direction[*j] > 1e-14)
                    .map(|(j, &i)| (j, lambda[i] / direction[j]))
                    .min_by(|a, b| a.1.total_cmp(&b.1));
                let drop = bound.filter(|(_, limit)| *limit <= primal);
                let step = drop.map_or(primal, |(_, limit)| limit);
                if !step.is_finite() {
                    return Err(fail(
                        "Dependent normal constraint has no finite feasible update",
                    ));
                }
                let old_lambda = lambda.clone();
                let mut change = |index: usize, value: f64| -> Result<()> {
                    if !value.is_finite() || value / scales[index] > 1e18 {
                        return Err(fail("Normal multiplier exceeds numeric range"));
                    }
                    let delta = value - lambda[index];
                    lambda[index] = value;
                    for (i, v) in velocity.iter_mut().enumerate() {
                        *v += matrix[i][index] * delta;
                        if !v.is_finite() {
                            return Err(fail("Nonfinite normal iteration"));
                        }
                    }
                    Ok(())
                };
                for (j, &i) in active.iter().enumerate() {
                    change(i, (old_lambda[i] - step * direction[j]).max(0.))?;
                }
                change(next, old_lambda[next] + step)?;
                if let Some((j, _)) = drop {
                    active.remove(j);
                } else {
                    active.push(next);
                    break;
                }
            }
        }
    }
}
fn dot(a: [f64; 6], b: [f64; 6]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}
fn active_solve(matrix: &[[f64; 64]; 64], active: &[usize], rhs: &[f64]) -> Result<Vec<f64>> {
    let mut lower = [[0.; 6]; 6];
    let n = active.len();
    if n > 6 {
        return Err(fail("Normal active set exceeds chassis rank"));
    }
    for i in 0..n {
        for j in 0..=i {
            let v = matrix[active[i]][active[j]]
                - (0..j).map(|k| lower[i][k] * lower[j][k]).sum::<f64>();
            if i == j {
                if !v.is_finite() || v <= 1e-12 {
                    return Err(fail("Normal active block is numerically dependent"));
                }
                lower[i][j] = v.sqrt();
            } else {
                lower[i][j] = v / lower[j][j];
            }
        }
    }
    let mut x = vec![0.; n];
    for i in 0..n {
        x[i] = (rhs[i] - (0..i).map(|j| lower[i][j] * x[j]).sum::<f64>()) / lower[i][i];
    }
    for i in (0..n).rev() {
        x[i] = (x[i] - (i + 1..n).map(|j| lower[j][i] * x[j]).sum::<f64>()) / lower[i][i];
    }
    if x.iter().any(|v| !v.is_finite()) {
        return Err(fail("Nonfinite active normal solve"));
    }
    Ok(x)
}
