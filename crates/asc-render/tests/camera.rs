use asc_render::camera;
use asc_render::scene::orbit::Orbit;
use glam::{Mat4, Vec3};

#[test]
fn camera_follow_translates_the_target_without_changing_view_direction_and_rejects_bad_updates() {
    let mut camera = Orbit::new([[-1.; 3], [1.; 3]], [[-20.; 3], [20.; 3]]).unwrap();
    let before = camera.view(800, 600, true).unwrap();
    let delta = glam::DVec3::new(2., 3., -4.);
    camera.translate_target(delta.to_array()).unwrap();
    let after = camera.view(800, 600, true).unwrap();
    assert_eq!(before.direction, after.direction);
    let p = Vec3::new(0.2, 0.3, -0.4);
    let a = Mat4::from_cols_array(&before.matrix) * p.extend(1.);
    let b = Mat4::from_cols_array(&after.matrix) * (p + delta.as_vec3()).extend(1.);
    assert!((a - b).length() < 1e-5);
    assert!(camera.translate_target([f64::NAN; 3]).is_err());
    assert_eq!(camera.view(800, 600, true).unwrap().matrix, after.matrix);
}

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

#[test]
fn orbit_fits_landscape_and_portrait_bounds_in_both_projections() {
    let bounds = [[-12., -2., -7.], [15., 4., 20.]];
    for perspective in [true, false] {
        for (w, h) in [(1240, 900), (180, 350)] {
            let mut orbit = Orbit::new(bounds, bounds).unwrap();
            orbit.perspective = perspective;
            let view = orbit.view(w, h, true).unwrap();
            let m = Mat4::from_cols_array(&view.matrix);
            for i in 0..8 {
                let p = Vec3::new(
                    bounds[i & 1][0],
                    bounds[(i >> 1) & 1][1],
                    bounds[(i >> 2) & 1][2],
                );
                let clip = m * p.extend(1.);
                let ndc = clip / clip.w;
                assert!(
                    ndc.x.abs() < 1. && ndc.y.abs() < 1. && ndc.z > 0. && ndc.z < 1.,
                    "Orbit clip {ndc:?}"
                );
            }
        }
    }
}
#[test]
fn orbit_pan_zoom_and_projection_produce_finite_changes_and_home_is_replayable() {
    let bounds = [[-20., -1., -20.], [20., 1., 20.]];
    let home = Orbit::new(bounds, bounds).unwrap();
    let original = home.view(800, 600, true).unwrap().matrix;
    let mut orbit = home.clone();
    orbit.orbit(50., 25.);
    assert_ne!(orbit.view(800, 600, true).unwrap().matrix, original);
    let direction = orbit.direction();
    orbit.pan(12., -4., 800, 600);
    assert_eq!(orbit.direction(), direction);
    orbit.zoom(2.);
    let zoomed = orbit.view(800, 600, false).unwrap();
    assert!(!zoomed.textures);
    assert!(zoomed.matrix.iter().all(|v| v.is_finite()));
    orbit.top();
    assert!(!orbit.perspective);
    assert!(orbit.direction()[1] > 0.99);
    assert_eq!(home.view(800, 600, true).unwrap().matrix, original);
}
#[test]
fn orbit_rejects_bad_bounds_and_ignores_nonfinite_input_without_damaging_state() {
    let bounds = [[-1.; 3], [1.; 3]];
    let mut orbit = Orbit::new(bounds, bounds).unwrap();
    let before = orbit.view(500, 400, true).unwrap().matrix;
    assert!(!orbit.orbit(f64::NAN, 0.));
    assert!(!orbit.pan(0., f64::INFINITY, 500, 400));
    assert!(!orbit.zoom(f64::NAN));
    assert_eq!(before, orbit.view(500, 400, true).unwrap().matrix);
    for _ in 0..100 {
        orbit.orbit(1e20, -1e20);
        orbit.zoom(1e20);
        orbit.pan(1e20, 1e20, 500, 400);
    }
    assert!(
        orbit
            .view(500, 400, true)
            .unwrap()
            .matrix
            .iter()
            .all(|v| v.is_finite())
    );
    assert!(Orbit::new([[f32::NAN; 3], [1.; 3]], bounds).is_err());
    assert!(Orbit::new([[0.; 3], [1e8; 3]], bounds).is_err());
    assert!(orbit.view(0, 0, true).is_err());
}
