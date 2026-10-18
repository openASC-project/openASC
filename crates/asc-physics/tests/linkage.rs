use asc_physics::{FiveLink, LinkPose};
use glam::{DQuat, DVec3};

fn points() -> ([[f64; 3]; 5], [[f64; 3]; 5]) {
    (
        [
            [-0.3, 0.2, 0.12],
            [-0.3, 0.2, -0.12],
            [-0.3, -0.2, 0.12],
            [-0.3, -0.2, -0.12],
            [-0.3, 0., 0.15],
        ],
        [
            [0., 0.2, 0.],
            [0., 0.2, 0.],
            [0., -0.2, 0.],
            [0., -0.2, 0.],
            [0., 0., 0.15],
        ],
    )
}
fn model() -> FiveLink {
    let (a, b) = points();
    FiveLink::new(
        a,
        b,
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        [0., 1., 0.],
    )
    .unwrap()
}
#[test]
fn parallel_wishbone_sweep_matches_closed_form_circle() {
    let model = model();
    let mut seed = model.reference_pose();
    for height in [-0.08_f64, -0.04, 0., 0.04, 0.08, 0.04, 0., -0.04, -0.08] {
        let result = model.solve(height, [0.; 3], Some(seed)).unwrap();
        let expected_x = -0.3 + (0.09 - height * height).sqrt();
        assert!((result.position_m[0] - expected_x).abs() < 1e-9);
        assert!((result.position_m[1] - height).abs() < 1e-10);
        assert!(result.position_m[2].abs() < 1e-9);
        assert!(DQuat::from_array(result.orientation_xyzw).angle_between(DQuat::IDENTITY) < 1e-7);
        assert!(result.link_length_errors_m.iter().all(|v| v.abs() <= 1e-10));
        seed = result.pose().unwrap();
    }
}
#[test]
fn steering_changes_upright_rotation_while_preserving_all_five_lengths() {
    let model = model();
    let result = model.solve(0.025, [0.006, 0., 0.], None).unwrap();
    let q = DQuat::from_array(result.orientation_xyzw);
    assert!((q * DVec3::Z).x.abs() > 0.02);
    let pose = result.pose().unwrap();
    let (a, b) = points();
    for i in 0..5 {
        let anchor = DVec3::from_array(a[i])
            + if i == 4 {
                DVec3::new(0.006, 0., 0.)
            } else {
                DVec3::ZERO
            };
        let end = DVec3::from_array(pose.transform(b[i]).unwrap());
        assert!(((end - anchor).length() - model.rest_lengths_m()[i]).abs() < 1e-10);
    }
}
#[test]
fn global_rigid_frame_change_preserves_the_solved_mechanism() {
    let base = model().solve(0.04, [0.004, 0., 0.], None).unwrap();
    let rotation = DQuat::from_scaled_axis(DVec3::new(0.2, -0.3, 0.1));
    let offset = DVec3::new(1.5, 2.5, -3.);
    let (anchors, attachments) = points();
    let moved = FiveLink::new(
        anchors.map(|v| (rotation * DVec3::from_array(v) + offset).to_array()),
        attachments,
        LinkPose::new(offset.to_array(), rotation.to_array()).unwrap(),
        (rotation * DVec3::Y).to_array(),
    )
    .unwrap();
    let result = moved
        .solve(
            0.04,
            (rotation * DVec3::new(0.004, 0., 0.)).to_array(),
            None,
        )
        .unwrap();
    assert!(
        (DVec3::from_array(result.position_m)
            - (rotation * DVec3::from_array(base.position_m) + offset))
            .length()
            < 1e-9
    );
    assert!(
        DQuat::from_array(result.orientation_xyzw)
            .angle_between(rotation * DQuat::from_array(base.orientation_xyzw))
            < 1e-7
    );
}
#[test]
fn invalid_singular_and_unreachable_geometries_fail_without_mutation() {
    let (a, b) = points();
    let identity = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
    assert!(FiveLink::new(a, [[0.; 3]; 5], identity, [0., 1., 0.]).is_err());
    assert!(FiveLink::new(a, b, identity, [0.; 3]).is_err());
    assert!(LinkPose::new([f64::NAN, 0., 0.], [0., 0., 0., 1.]).is_err());
    let model = model();
    let before = model.reference_pose().position_m();
    for height in [f64::NAN, f64::INFINITY, 0.251, -0.251] {
        assert!(model.solve(height, [0.; 3], None).is_err());
    }
    assert!(model.solve(0., [0.26, 0., 0.], None).is_err());
    assert!(model.solve(0., [0.2, 0.2, 0.2], None).is_err());
    let bad_seed = LinkPose::new([1., 0., 0.], [0., 0., 0., 1.]).unwrap();
    assert!(model.solve(0., [0.; 3], Some(bad_seed)).is_err());
    assert_eq!(model.reference_pose().position_m(), before);
    assert_eq!(model.solve(0., [0.; 3], None).unwrap().iterations, 0);
    let (mut a, b) = points();
    for anchor in &mut a {
        anchor[0] = -0.1;
    }
    let short = FiveLink::new(a, b, identity, [0., 1., 0.]).unwrap();
    // The average upper/lower Y displacement cannot exceed either arm length;
    // 0.2 m is unreachable even though it passes the generic input range gate.
    assert!(short.solve(0.2, [0.; 3], None).is_err());
}
