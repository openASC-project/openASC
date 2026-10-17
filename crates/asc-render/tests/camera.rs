use asc_render::camera;
use glam::{Mat4, Vec3};

#[test]
fn camera_fits_all_corners_in_webgpu_clip_space_including_portrait() {
    let bounds = [[-123., -12., -23.], [25., 4., 100.]];
    for (w, h) in [(1240, 900), (319, 177), (177, 319)] {
        let m = Mat4::from_cols_array(&camera(bounds, w, h).unwrap());
        for i in 0..8 {
            let p = Vec3::new(
                bounds[i & 1][0],
                bounds[(i >> 1) & 1][1],
                bounds[(i >> 2) & 1][2],
            );
            let clip = m * p.extend(1.);
            assert!(
                clip.x.abs() < 1. && clip.y.abs() < 1. && clip.z > 0. && clip.z < 1.,
                "{clip:?}"
            );
        }
    }
    assert!(camera(bounds, 0, 1).is_err());
    assert!(camera(bounds, 4097, 1).is_err());
    assert!(camera([[1.; 3], [0.; 3]], 100, 100).is_err());
}
