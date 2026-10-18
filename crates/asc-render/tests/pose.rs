use asc_render::scene::pose::RigidPose;
use glam::DQuat;
#[test]
fn rigid_composition_matches_explicit_local_graphics_then_body_transform() {
    let body =
        RigidPose::from_basis([15., 5., 0.], [0., 0., -1.], [0., 1., 0.], [1., 0., 0.]).unwrap();
    let graphics = RigidPose::new(
        [0., -0.3, 0.1],
        DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2).to_array(),
    )
    .unwrap();
    let root = body.compose(graphics).unwrap();
    let p = root.point([0., 1., 0.]).unwrap();
    for (a, b) in p.into_iter().zip([16.1, 4.7, 0.]) {
        assert!((a - b).abs() < 1e-5);
    }
    let bounds = root.bounds([[0.; 3], [1.; 3]]).unwrap();
    assert!((bounds[0][0] - 15.1).abs() < 1e-5);
    assert!((bounds[1][0] - 16.1).abs() < 1e-5);
    assert!((bounds[0][2] + 1.).abs() < 1e-5);
}
#[test]
fn invalid_rigid_roots_and_out_of_domain_bounds_are_rejected() {
    for q in [[0.; 4], [f64::NAN, 0., 0., 1.], [2., 0., 0., 0.]] {
        assert!(RigidPose::new([0.; 3], q).is_err());
    }
    assert!(RigidPose::from_basis([0.; 3], [1., 0.1, 0.], [0., 1., 0.], [0., 0., 1.]).is_err());
    assert!(RigidPose::from_basis([0.; 3], [-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]).is_err());
    let pose = RigidPose::new([1e7, 0., 0.], [0., 0., 0., 1.]).unwrap();
    assert!(pose.bounds([[0.; 3], [1.; 3]]).is_err());
    assert!(RigidPose::default().bounds([[1.; 3], [0.; 3]]).is_err());
    assert!(RigidPose::default().point([f32::NAN, 0., 0.]).is_err());
}
