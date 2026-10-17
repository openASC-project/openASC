//! Bounded orbit/pan/zoom state, independent of the window event system.
use crate::gpu::scene::{Result, View};
use glam::{DMat4, DVec3};

#[derive(Clone)]
pub struct Orbit {
    target: DVec3,
    radius: f64,
    world_radius: f64,
    yaw: f64,
    pitch: f64,
    zoom: f64,
    pub perspective: bool,
}
impl Orbit {
    pub fn new(bounds: [[f32; 3]; 2], world: [[f32; 3]; 2]) -> Result<Self> {
        let decode = |b: [[f32; 3]; 2]| -> Result<(DVec3, f64)> {
            let lo = DVec3::from_array(b[0].map(f64::from));
            let hi = DVec3::from_array(b[1].map(f64::from));
            if !lo.is_finite()
                || !hi.is_finite()
                || (hi - lo).min_element() < 0.
                || lo.abs().max_element().max(hi.abs().max_element()) > 1e7
            {
                return Err(
                    "Native orbit bounds must be finite, ordered and within +/-10000000 metres."
                        .into(),
                );
            }
            Ok(((lo + hi) * 0.5, ((hi - lo) * 0.5).length().max(0.1)))
        };
        let (target, radius) = decode(bounds)?;
        let (_, world_radius) = decode(world)?;
        Ok(Self {
            target,
            radius,
            world_radius,
            yaw: 0.7_f64.atan2(0.8),
            pitch: (1. / 2.13_f64.sqrt()).asin(),
            zoom: 1.,
            perspective: true,
        })
    }
    pub fn direction(&self) -> [f64; 3] {
        let (s, c) = self.yaw.sin_cos();
        let (p, q) = self.pitch.sin_cos();
        [q * s, p, q * c]
    }
    fn basis(&self) -> (DVec3, DVec3, DVec3) {
        let z = DVec3::from_array(self.direction());
        let x = DVec3::Y.cross(z).normalize();
        (x, z.cross(x), z)
    }
    fn distance(&self, aspect: f64) -> f64 {
        let half = 26_f64.to_radians();
        let angle = half.min((half.tan() * aspect).atan());
        self.radius / angle.sin() * 1.1 * self.zoom
    }
    pub fn orbit(&mut self, dx: f64, dy: f64) -> bool {
        if !dx.is_finite() || !dy.is_finite() {
            return false;
        }
        self.yaw = (self.yaw - dx.clamp(-4096., 4096.) * 0.006).rem_euclid(std::f64::consts::TAU);
        self.pitch = (self.pitch + dy.clamp(-4096., 4096.) * 0.006).clamp(-1.55, 1.55);
        true
    }
    pub fn pan(&mut self, dx: f64, dy: f64, width: u32, height: u32) -> bool {
        if !dx.is_finite() || !dy.is_finite() || width == 0 || height == 0 {
            return false;
        }
        let aspect = width as f64 / height as f64;
        let half = if self.perspective {
            self.distance(aspect) * 26_f64.to_radians().tan()
        } else {
            self.radius * 1.1 * self.zoom / aspect.min(1.)
        };
        let (right, up, _) = self.basis();
        self.target += (up * dy.clamp(-4096., 4096.) - right * dx.clamp(-4096., 4096.)) * half * 2.
            / height as f64;
        self.target = self.target.clamp(DVec3::splat(-1e7), DVec3::splat(1e7));
        true
    }
    pub fn zoom(&mut self, lines: f64) -> bool {
        if !lines.is_finite() {
            return false;
        }
        self.zoom = (self.zoom * (-lines.clamp(-20., 20.) * 0.12).exp()).clamp(0.01, 100.);
        true
    }
    pub fn top(&mut self) {
        self.yaw = 0.;
        self.pitch = 1.55;
        self.perspective = false;
    }
    pub fn view(&self, width: u32, height: u32, textures: bool) -> Result<View> {
        if width == 0 || height == 0 || width > 4096 || height > 4096 {
            return Err("Native viewport dimensions must be in 1..=4096.".into());
        }
        let aspect = width as f64 / height as f64;
        let distance = self.distance(aspect);
        let (right, up, z) = self.basis();
        let eye = self.target + z * distance;
        let view = DMat4::from_cols_array(&[
            right.x,
            up.x,
            z.x,
            0.,
            right.y,
            up.y,
            z.y,
            0.,
            right.z,
            up.z,
            z.z,
            0.,
            -right.dot(eye),
            -up.dot(eye),
            -z.dot(eye),
            1.,
        ]);
        let near = (distance * 0.0001).clamp(0.01, 1.);
        let far = distance + self.world_radius.max(self.radius) * 4. + 100.;
        let projection = if self.perspective {
            let f = 1. / 26_f64.to_radians().tan();
            DMat4::from_cols_array(&[
                f / aspect,
                0.,
                0.,
                0.,
                0.,
                f,
                0.,
                0.,
                0.,
                0.,
                far / (near - far),
                -1.,
                0.,
                0.,
                near * far / (near - far),
                0.,
            ])
        } else {
            let half = self.radius * 1.1 * self.zoom / aspect.min(1.);
            DMat4::from_cols_array(&[
                1. / (half * aspect),
                0.,
                0.,
                0.,
                0.,
                1. / half,
                0.,
                0.,
                0.,
                0.,
                1. / (near - far),
                0.,
                0.,
                0.,
                near / (near - far),
                1.,
            ])
        };
        let matrix = (projection * view).to_cols_array().map(|v| v as f32);
        if matrix.iter().any(|v| !v.is_finite()) {
            return Err("Native camera matrix exceeds float32 range.".into());
        }
        Ok(View {
            matrix,
            direction: self.direction(),
            textures,
        })
    }
}
