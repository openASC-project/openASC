//! Explicit source-frame rack paths; no automatic AC side/setup or steering-ratio mapping.
use asc_physics::{Error, RackMotion, Result};
use glam::DVec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct SteeringKey {
    pub time_s: f64,
    pub displacement_m: f64,
    pub velocity_m_s: f64,
}
struct Segment {
    start: f64,
    duration: f64,
    coefficients: [f64; 4],
}
impl Segment {
    fn evaluate(&self, u: f64) -> [f64; 3] {
        let [a, b, c, d] = self.coefficients;
        [
            a + u * (b + u * (c + u * d)),
            (b + u * (2. * c + 3. * u * d)) / self.duration,
            (2. * c + 6. * u * d) / (self.duration * self.duration),
        ]
    }
    fn validate(&self) -> Result<()> {
        let [_, b, c, d] = self.coefficients;
        let mut points = vec![0., 1.];
        if d != 0. {
            let disc = 4. * c * c - 12. * d * b;
            if disc >= 0. {
                let root = -c - 0.5 * disc.sqrt().copysign(c);
                if root != 0. {
                    points.extend([root / (3. * d), b / root]);
                } else {
                    points.push(-c / (3. * d));
                }
            }
            points.push(-c / (3. * d)); // Velocity extremum.
        } else if c != 0. {
            points.push(-b / (2. * c));
        }
        for u in points.into_iter().filter(|u| (0. ..=1.).contains(u)) {
            let [p, v, a] = self.evaluate(u);
            if !p.is_finite()
                || p.abs() > 0.25
                || !v.is_finite()
                || v.abs() > 1e3
                || !a.is_finite()
                || a.abs() > 1e6
            {
                return Err(fail());
            }
        }
        Ok(())
    }
}
fn fail() -> Error {
    Error::new(
        "chassis_steering_path",
        "Expected 1..64 ordered Hermite keys starting at 0, segment duration >=1e-6 s, endpoint velocities zero, displacement within +/-0.25 m, velocity +/-1000 m/s and acceleration +/-1e6 m/s^2 throughout each segment",
    )
}
pub(super) struct Steering {
    base: DVec3,
    axis: DVec3,
    keys: Vec<SteeringKey>,
    segments: Vec<Segment>,
}
impl Steering {
    pub fn new(base: [f64; 3], axis: [f64; 3], keys: &[SteeringKey]) -> Result<Self> {
        RackMotion::fixed(base)?;
        let axis = DVec3::from_array(axis);
        if !axis.is_finite() || (axis.length_squared() - 1.).abs() > 1e-8 {
            return Err(Error::new(
                "chassis_steering_axis",
                "Explicit source-frame rack axis must be unit length",
            ));
        }
        if !(1..=64).contains(&keys.len())
            || keys[0].time_s != 0.
            || keys[0].velocity_m_s != 0.
            || keys.last().unwrap().velocity_m_s != 0.
            || keys.iter().any(|k| {
                !k.time_s.is_finite()
                    || !(0. ..=1e6).contains(&k.time_s)
                    || !k.displacement_m.is_finite()
                    || k.displacement_m.abs() > 0.25
                    || !k.velocity_m_s.is_finite()
                    || k.velocity_m_s.abs() > 1e3
            })
            || keys.windows(2).any(|p| p[1].time_s - p[0].time_s < 1e-6)
        {
            return Err(fail());
        }
        let mut segments = Vec::new();
        for pair in keys.windows(2) {
            let [a, b] = [pair[0], pair[1]];
            let h = b.time_s - a.time_s;
            let delta = b.displacement_m - a.displacement_m;
            let segment = Segment {
                start: a.time_s,
                duration: h,
                coefficients: [
                    a.displacement_m,
                    h * a.velocity_m_s,
                    3. * delta - h * (2. * a.velocity_m_s + b.velocity_m_s),
                    -2. * delta + h * (a.velocity_m_s + b.velocity_m_s),
                ],
            };
            segment.validate()?;
            segments.push(segment);
        }
        Ok(Self {
            base: DVec3::from_array(base),
            axis: axis.normalize(),
            keys: keys.to_vec(),
            segments,
        })
    }
    pub fn sample(&self, time_s: f64) -> Result<RackMotion> {
        if !time_s.is_finite() || !(0. ..=1e6).contains(&time_s) {
            return Err(fail());
        }
        let index = self.keys.partition_point(|k| k.time_s <= time_s);
        let [p, v, a] = if index == self.keys.len() {
            [self.keys.last().unwrap().displacement_m, 0., 0.]
        } else {
            let s = &self.segments[index.saturating_sub(1)];
            s.evaluate((time_s - s.start) / s.duration)
        };
        RackMotion::new(
            (self.base + self.axis * p).to_array(),
            (self.axis * v).to_array(),
            (self.axis * a).to_array(),
        )
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn key(time_s: f64, displacement_m: f64, velocity_m_s: f64) -> SteeringKey {
        SteeringKey {
            time_s,
            displacement_m,
            velocity_m_s,
        }
    }
    #[test]
    fn hermite_racks_have_analytic_derivatives_and_continuous_velocity() {
        let path = Steering::new(
            [0.; 3],
            [1., 0., 0.],
            &[key(0., 0., 0.), key(1., 0.02, 0.01), key(2., 0., 0.)],
        )
        .unwrap();
        for t in [0.1, 0.5, 0.9, 1.1, 1.5, 1.9] {
            let e = 1e-5;
            let m = path.sample(t - e).unwrap();
            let c = path.sample(t).unwrap();
            let p = path.sample(t + e).unwrap();
            assert!(
                ((p.position_m[0] - m.position_m[0]) / (2. * e) - c.velocity_m_s[0]).abs() < 1e-8
            );
            assert!(
                ((p.velocity_m_s[0] - m.velocity_m_s[0]) / (2. * e) - c.acceleration_m_s2[0]).abs()
                    < 1e-8
            );
        }
        assert!(
            (path.sample(1. - 1e-8).unwrap().velocity_m_s[0]
                - path.sample(1. + 1e-8).unwrap().velocity_m_s[0])
                .abs()
                < 1e-8
        );
        assert_eq!(
            path.sample(3.).unwrap(),
            RackMotion::fixed([0.; 3]).unwrap()
        );
    }
    #[test]
    fn steering_rejects_bad_axes_keys_and_cubic_interior_overshoot() {
        assert!(Steering::new([0.; 3], [0.; 3], &[key(0., 0., 0.)]).is_err());
        for keys in [
            vec![],
            vec![key(1., 0., 0.)],
            vec![key(0., 0., 1.)],
            vec![key(0., 0., 0.), key(0., 0., 0.)],
            vec![key(0., 0., 0.), key(0.001, 0.25, 0.)],
            vec![
                key(0., 0., 0.),
                key(1., 0.1, 1.),
                key(2., 0.1, -2.),
                key(3., 0., 0.),
            ],
            vec![
                key(0., 0., 0.),
                key(1., 0.2, 2.),
                key(2., 0., -2.),
                key(3., 0., 0.),
            ],
        ] {
            assert!(Steering::new([0.; 3], [1., 0., 0.], &keys).is_err());
        }
    }
}
