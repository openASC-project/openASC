use asc_physics::{LinkPose, StrutLink};
use glam::{DQuat, DVec3};
fn points() -> ([[f64; 3]; 3], [[f64; 3]; 3]) {
    (
        [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]],
        [[0.; 3], [0.; 3], [0., 0., 0.15]],
    )
}
fn model() -> StrutLink {
    let (a, b) = points();
    StrutLink::new(
        a,
        b,
        [0., 0.6, 0.],
        [0.; 3],
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        [0., 1., 0.],
    )
    .unwrap()
}
#[test]
fn macpherson_sweep_matches_closed_form_lower_arm_and_strut_axis() {
    let model = model();
    let mut seed = model.reference_pose();
    for height in [-0.08_f64, -0.04, 0., 0.04, 0.08, 0.04, 0., -0.04, -0.08] {
        let result = model.solve(height, [0.; 3], Some(seed)).unwrap();
        let x = -0.3 + (0.09 - height * height).sqrt();
        assert!((result.position_m[0] - x).abs() < 1e-9);
        assert!((result.position_m[1] - height).abs() < 1e-10);
        assert!(result.position_m[2].abs() < 1e-9);
        let expected = DQuat::from_rotation_z(x.atan2(0.6 - height));
        assert!(DQuat::from_array(result.orientation_xyzw).angle_between(expected) < 1e-7);
        assert!((result.strut_length_m - x.hypot(0.6 - height)).abs() < 1e-9);
        assert!((result.strut_length_change_m - (result.strut_length_m - 0.6)).abs() < 1e-12);
        assert!(result.coaxial_errors_m.iter().all(|v| v.abs() <= 1e-10));
        seed = result.pose().unwrap();
    }
}
#[test]
fn steering_preserves_three_distances_and_top_lies_on_the_upright_axis() {
    let model = model();
    let (anchors, attachments) = points();
    for rack in [-0.008_f64, 0.008] {
        let result = model.solve(0.03, [rack, 0., 0.], None).unwrap();
        let pose = result.pose().unwrap();
        let q = DQuat::from_array(result.orientation_xyzw);
        assert!((q * DVec3::Z).x.abs() > 0.03);
        for i in 0..3 {
            let anchor = DVec3::from_array(anchors[i])
                + if i == 2 {
                    DVec3::new(rack, 0., 0.)
                } else {
                    DVec3::ZERO
                };
            let end = DVec3::from_array(pose.transform(attachments[i]).unwrap());
            assert!(((anchor - end).length() - model.rest_lengths_m()[i]).abs() < 1e-10);
        }
        let axis = q * DVec3::Y;
        let offset = DVec3::new(0., 0.6, 0.) - DVec3::from_array(result.position_m);
        assert!(axis.cross(offset).length() < 1.5e-10);
        assert!(axis.dot(offset) > 0.);
    }
}
#[test]
fn frame_rotation_and_translation_preserve_strut_geometry_and_extension() {
    let base = model().solve(0.04, [0.004, 0., 0.], None).unwrap();
    let q = DQuat::from_scaled_axis(DVec3::new(0.2, -0.3, 0.1));
    let t = DVec3::new(1., 2., 3.);
    let (a, b) = points();
    let moved = StrutLink::new(
        a.map(|v| (q * DVec3::from_array(v) + t).to_array()),
        b,
        (q * DVec3::new(0., 0.6, 0.) + t).to_array(),
        [0.; 3],
        LinkPose::new(t.to_array(), q.to_array()).unwrap(),
        (q * DVec3::Y).to_array(),
    )
    .unwrap();
    let result = moved
        .solve(0.04, (q * DVec3::new(0.004, 0., 0.)).to_array(), None)
        .unwrap();
    assert!(
        (DVec3::from_array(result.position_m) - (q * DVec3::from_array(base.position_m) + t))
            .length()
            < 1e-9
    );
    assert!(
        DQuat::from_array(result.orientation_xyzw)
            .angle_between(q * DQuat::from_array(base.orientation_xyzw))
            < 1e-7
    );
    assert!((result.strut_length_m - base.strut_length_m).abs() < 1e-9);
}
#[test]
fn collapsed_singular_reversed_and_unreachable_struts_fail_without_mutation() {
    let (mut a, b) = points();
    let reference = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
    assert!(StrutLink::new(a, b, [0.; 3], [0.; 3], reference, [0., 1., 0.]).is_err());
    assert!(
        StrutLink::new(
            a,
            [[0.; 3]; 3],
            [0., 0.6, 0.],
            [0.; 3],
            reference,
            [0., 1., 0.]
        )
        .is_err()
    );
    assert!(StrutLink::new(a, b, [f64::NAN, 0., 0.], [0.; 3], reference, [0., 1., 0.]).is_err());
    for anchor in &mut a {
        anchor[0] = -0.1;
    }
    let short = StrutLink::new(a, b, [0., 0.6, 0.], [0.; 3], reference, [0., 1., 0.]).unwrap();
    assert!(short.solve(0.2, [0.; 3], None).is_err());
    let model = model();
    for height in [f64::NAN, 0.251, -0.251] {
        assert!(model.solve(height, [0.; 3], None).is_err());
    }
    let reversed = LinkPose::new(
        [0.; 3],
        DQuat::from_rotation_x(std::f64::consts::PI).to_array(),
    )
    .unwrap();
    assert!(model.solve(0., [0.; 3], Some(reversed)).is_err());
    assert_eq!(model.reference_pose().position_m(), [0.; 3]);
    assert_eq!(model.solve(0., [0.; 3], None).unwrap().iterations, 0);
}
