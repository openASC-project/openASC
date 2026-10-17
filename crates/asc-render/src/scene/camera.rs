//! Camera projection independent of GPU resources.
use glam::{DMat4, DVec3};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub fn camera(bounds: [[f32; 3]; 2], width: u32, height: u32) -> Result<[f32; 16]> {
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err("Image dimensions must be in 1..=4096.".into());
    }
    let lo = DVec3::from_array(bounds[0].map(f64::from));
    let hi = DVec3::from_array(bounds[1].map(f64::from));
    if !lo.is_finite() || !hi.is_finite() || (hi - lo).min_element() < 0. {
        return Err("Invalid scene bounds.".into());
    }
    let center = (lo + hi) * 0.5;
    let radius = ((hi - lo) * 0.5).length().max(0.1);
    let z = DVec3::new(0.7, 1., 0.8).normalize();
    let eye = center + z * radius * 3.;
    let right = DVec3::Y.cross(z).normalize();
    let up = z.cross(right);
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
    let (mut x, mut y) = (0.01_f64, 0.01_f64);
    for i in 0..8 {
        let p = DVec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let v = view.transform_point3(p);
        x = x.max(v.x.abs());
        y = y.max(v.y.abs());
    }
    let aspect = width as f64 / height as f64;
    let half_height = y.max(x / aspect) * 1.1;
    let half_width = half_height * aspect;
    // Right-handed orthographic projection with WebGPU depth in [0, 1].
    let near = radius;
    let far = radius * 5.;
    let projection = DMat4::from_cols_array(&[
        1. / half_width,
        0.,
        0.,
        0.,
        0.,
        1. / half_height,
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
    ]);
    let matrix = (projection * view).to_cols_array().map(|v| v as f32);
    if matrix.iter().any(|v| !v.is_finite()) {
        return Err("Camera exceeds float32 range.".into());
    }
    Ok(matrix)
}
