use asc_physics::{Error, Result};
use serde::{Deserialize, Serialize};
use winit::keyboard::KeyCode;
#[derive(Default)]
pub(super) struct Keys {
    throttle: bool,
    brake_s: bool,
    brake_space: bool,
    left: bool,
    right: bool,
    pub reverse: bool,
}
impl Keys {
    pub fn event(&mut self, key: KeyCode, down: bool, repeat: bool, paused: bool) {
        if !down || (!paused && !repeat) {
            self.key(key, down);
        }
    }
    pub fn clear(&mut self) {
        self.throttle = false;
        self.brake_s = false;
        self.brake_space = false;
        self.left = false;
        self.right = false;
    }
    pub fn key(&mut self, key: KeyCode, down: bool) {
        match key {
            KeyCode::KeyW => self.throttle = down,
            KeyCode::KeyS => self.brake_s = down,
            KeyCode::Space => self.brake_space = down,
            KeyCode::KeyA => self.left = down,
            KeyCode::KeyD => self.right = down,
            _ => {}
        }
    }
    pub fn command(&self) -> Command {
        Command {
            drive: if self.throttle {
                if self.reverse { -1. } else { 1. }
            } else {
                0.
            },
            brake: f64::from(self.brake_s || self.brake_space),
            steering: f64::from(self.left) - f64::from(self.right),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize)]
pub(super) struct Command {
    pub drive: f64,
    pub brake: f64,
    pub steering: f64,
}
impl Command {
    pub fn validate(self) -> Result<()> {
        if !self.drive.is_finite()
            || self.drive.abs() > 1.
            || !self.brake.is_finite()
            || !(0. ..=1.).contains(&self.brake)
            || !self.steering.is_finite()
            || self.steering.abs() > 1.
        {
            return Err(Error::new(
                "native_drive_input",
                "Drive/steering must lie in -1..1 and brake in 0..1",
            ));
        }
        Ok(())
    }
}
#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct SteeringActuator {
    pub displacement_limit_m: f64,
    pub frequency_rad_s: f64,
    pub acceleration_limit_m_s2: f64,
    pub rate_limit_m_s: f64,
}
impl SteeringActuator {
    pub fn validate(self) -> Result<()> {
        for (v, lo, hi) in [
            (self.displacement_limit_m, 0., 0.1),
            (self.frequency_rad_s, 0.1, 100.),
            (self.acceleration_limit_m_s2, 1e-4, 10.),
            (self.rate_limit_m_s, 1e-4, 1.),
        ] {
            if !v.is_finite() || !(lo..=hi).contains(&v) {
                return Err(Error::new(
                    "native_steering_actuator",
                    "Invalid explicit steering target, frequency, acceleration or rate limit",
                ));
            }
        }
        Ok(())
    }
    pub fn acceleration(self, position: f64, velocity: f64, input: f64, dt: f64) -> Result<f64> {
        self.validate()?;
        if !position.is_finite()
            || !velocity.is_finite()
            || !input.is_finite()
            || input.abs() > 1.
            || !dt.is_finite()
            || !(1e-6..=1. / 30.).contains(&dt)
        {
            return Err(Error::new(
                "native_steering_state",
                "Invalid steering actuator state or step",
            ));
        }
        let a = (self.frequency_rad_s.powi(2) * (input * self.displacement_limit_m - position)
            - 2. * self.frequency_rad_s * velocity)
            .clamp(-self.acceleration_limit_m_s2, self.acceleration_limit_m_s2);
        let next = (velocity + a * dt).clamp(-self.rate_limit_m_s, self.rate_limit_m_s);
        Ok(((next - velocity) / dt)
            .clamp(-self.acceleration_limit_m_s2, self.acceleration_limit_m_s2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn key_release_focus_clear_and_brake_aliases_do_not_leave_stuck_commands() {
        let mut k = Keys::default();
        k.key(KeyCode::KeyW, true);
        k.key(KeyCode::KeyA, true);
        assert_eq!(k.command().drive, 1.);
        assert_eq!(k.command().steering, 1.);
        k.key(KeyCode::KeyD, true);
        assert_eq!(k.command().steering, 0.);
        k.key(KeyCode::KeyS, true);
        k.key(KeyCode::Space, true);
        k.key(KeyCode::KeyS, false);
        assert_eq!(k.command().brake, 1.);
        k.clear();
        k.event(KeyCode::KeyW, true, true, false);
        assert_eq!(k.command().drive, 0.);
        k.event(KeyCode::KeyW, true, false, true);
        assert_eq!(k.command().drive, 0.);
        assert_eq!(k.command().drive, 0.);
        assert_eq!(k.command().brake, 0.);
        assert_eq!(k.command().steering, 0.);
        k.reverse = true;
        k.key(KeyCode::KeyW, true);
        assert_eq!(k.command().drive, -1.);
        k.key(KeyCode::KeyW, false);
        assert_eq!(k.command().drive, 0.);
    }
    #[test]
    fn steering_response_is_continuous_rate_bounded_and_rejects_invalid_commands() {
        let a = SteeringActuator {
            displacement_limit_m: 0.01,
            frequency_rad_s: 8.,
            acceleration_limit_m_s2: 0.2,
            rate_limit_m_s: 0.08,
        };
        let mut p = 0.;
        let mut v = 0.;
        let dt = 0.001;
        for _ in 0..1000 {
            let acceleration = a.acceleration(p, v, 1., dt).unwrap();
            assert!(acceleration.abs() <= 0.2);
            p += v * dt + 0.5 * acceleration * dt * dt;
            v += acceleration * dt;
            assert!(v.abs() <= 0.08 + 1e-12);
        }
        assert!((p - 0.01).abs() < 0.0001);
        assert!(a.acceleration(p, v, f64::NAN, dt).is_err());
        assert!(
            Command {
                drive: 1.01,
                ..Command::default()
            }
            .validate()
            .is_err()
        );
        assert!(
            Command {
                brake: -0.01,
                ..Command::default()
            }
            .validate()
            .is_err()
        );
    }
}
