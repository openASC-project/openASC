use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Control {
    pub time_s: f64,
    pub drive_fraction: f64,
    pub brake_fraction: f64,
}
pub(super) struct Timeline(Vec<Control>);
impl Timeline {
    pub fn new(keys: &[Control]) -> Result<Self, &'static str> {
        if !(1..=64).contains(&keys.len())
            || keys[0].time_s != 0.
            || keys.iter().any(|k| {
                !k.time_s.is_finite()
                    || !(0. ..=1e6).contains(&k.time_s)
                    || !k.drive_fraction.is_finite()
                    || !(-1. ..=1.).contains(&k.drive_fraction)
                    || !k.brake_fraction.is_finite()
                    || !(0. ..=1.).contains(&k.brake_fraction)
            })
            || keys.windows(2).any(|p| p[1].time_s <= p[0].time_s)
        {
            return Err(
                "Controls require 1..64 strictly increasing finite keys, starting at time 0, times within 0..1e6 s, drive_fraction in -1..1 and brake_fraction in 0..1",
            );
        }
        Ok(Self(keys.to_vec()))
    }
    pub fn sample(&self, time_s: f64) -> asc_physics::Result<Control> {
        if !time_s.is_finite() || !(0. ..=1e6).contains(&time_s) {
            return Err(asc_physics::Error::new(
                "chassis_control_time",
                "Control time must lie within 0..1e6 seconds",
            ));
        }
        let index = self.0.partition_point(|k| k.time_s <= time_s);
        let a = self.0[index.saturating_sub(1)];
        if index == self.0.len() {
            return Ok(Control { time_s, ..a });
        }
        let b = self.0[index];
        let t = (time_s - a.time_s) / (b.time_s - a.time_s);
        Ok(Control {
            time_s,
            drive_fraction: (a.drive_fraction + (b.drive_fraction - a.drive_fraction) * t)
                .clamp(-1., 1.),
            brake_fraction: (a.brake_fraction + (b.brake_fraction - a.brake_fraction) * t)
                .clamp(0., 1.),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn key(t: f64, d: f64, b: f64) -> Control {
        Control {
            time_s: t,
            drive_fraction: d,
            brake_fraction: b,
        }
    }
    #[test]
    fn controls_interpolate_stage_times_and_hold_endpoints_without_overshoot() {
        let t = Timeline::new(&[key(0., 1., 0.), key(1., 0., 1.), key(2., -1., 0.)]).unwrap();
        for (time, drive, brake) in [
            (0., 1., 0.),
            (0.5, 0.5, 0.5),
            (1., 0., 1.),
            (1.5, -0.5, 0.5),
            (2., -1., 0.),
            (3., -1., 0.),
        ] {
            let c = t.sample(time).unwrap();
            assert_eq!(c.time_s, time);
            assert_eq!(c.drive_fraction, drive);
            assert_eq!(c.brake_fraction, brake);
        }
        assert!(t.sample(f64::NAN).is_err());
        assert!(t.sample(-1.).is_err());
    }
    #[test]
    fn controls_reject_bad_counts_times_domains_and_duplicate_keys() {
        for keys in [
            vec![],
            vec![key(1., 0., 0.)],
            vec![key(0., 2., 0.)],
            vec![key(0., 0., -1.)],
            vec![key(0., f64::NAN, 0.)],
            vec![key(0., 0., 0.), key(0., 1., 0.)],
            vec![key(0., 0., 0.), key(f64::INFINITY, 0., 0.)],
            vec![key(0., 0., 0.); 65],
        ] {
            assert!(Timeline::new(&keys).is_err());
        }
    }
}
