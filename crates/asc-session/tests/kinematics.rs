use asc_session::{
    dwb_geometry, dwb_geometry_with_reflection, strut_geometry, strut_geometry_with_reflection,
};
use asc_vehicle::suspension::Axle;
use std::collections::BTreeMap;

fn axle() -> Axle {
    Axle {
        kind: "DWB".into(),
        track_m: 1.5,
        base_y_m: -0.2,
        rod_length_m: 0.1,
        hub_mass_kg: 30.,
        hardpoints: [
            ("WBCAR_TOP_FRONT", [-0.3, 0.2, 0.12]),
            ("WBCAR_TOP_REAR", [-0.3, 0.2, -0.12]),
            ("WBCAR_BOTTOM_FRONT", [-0.3, -0.2, 0.12]),
            ("WBCAR_BOTTOM_REAR", [-0.3, -0.2, -0.12]),
            ("WBTYRE_TOP", [0., 0.2, 0.]),
            ("WBTYRE_BOTTOM", [0., -0.2, 0.]),
            ("WBCAR_STEER", [-0.3, 0., 0.15]),
            ("WBTYRE_STEER", [0., 0., 0.15]),
        ]
        .into_iter()
        .map(|(k, v)| (k.into(), v))
        .collect(),
        source_parameters: BTreeMap::new(),
    }
}

fn strut_axle() -> Axle {
    let mut source = axle();
    source.kind = "STRUT".into();
    source.hardpoints.insert("STRUT_CAR".into(), [0., 0.6, 0.]);
    source.hardpoints.insert("STRUT_TYRE".into(), [0.; 3]);
    source.hardpoints.insert("WBTYRE_BOTTOM".into(), [0.; 3]);
    source
        .hardpoints
        .insert("WBCAR_BOTTOM_FRONT".into(), [-0.3, 0., 0.12]);
    source
        .hardpoints
        .insert("WBCAR_BOTTOM_REAR".into(), [-0.3, 0., -0.12]);
    source
}
#[test]
fn strut_adapter_preserves_named_geometry_and_ignores_unverified_setup_corrections() {
    let mut source = strut_axle();
    let result = strut_geometry(&source, [0., 1., 0.])
        .unwrap()
        .solve(0.05, [0.; 3], None)
        .unwrap();
    assert!((result.position_m[0] - (-0.3 + (0.09_f64 - 0.0025).sqrt())).abs() < 1e-9);
    assert!((result.strut_length_m - result.position_m[0].hypot(0.55)).abs() < 1e-9);
    source.base_y_m = -0.8;
    source.rod_length_m = 0.3;
    let same = strut_geometry(&source, [0., 1., 0.])
        .unwrap()
        .solve(0.05, [0.; 3], None)
        .unwrap();
    assert_eq!(result.position_m, same.position_m);
    assert_eq!(result.orientation_xyzw, same.orientation_xyzw);
}
#[test]
fn strut_adapter_rejects_wrong_type_and_each_missing_required_point() {
    assert!(strut_geometry(&axle(), [0., 1., 0.]).is_err());
    for key in [
        "WBCAR_BOTTOM_FRONT",
        "WBCAR_BOTTOM_REAR",
        "WBTYRE_BOTTOM",
        "WBCAR_STEER",
        "WBTYRE_STEER",
        "STRUT_CAR",
        "STRUT_TYRE",
    ] {
        let mut source = strut_axle();
        source.hardpoints.remove(key);
        assert!(strut_geometry(&source, [0., 1., 0.]).is_err(), "{key}");
    }
}
#[test]
fn source_adapter_preserves_design_frame_and_selects_each_hardpoint() {
    let source = axle();
    let mechanism = dwb_geometry(&source, [0., 1., 0.]).unwrap();
    let result = mechanism.solve(0.05, [0.; 3], None).unwrap();
    assert!((result.position_m[0] - (-0.3 + (0.09_f64 - 0.0025).sqrt())).abs() < 1e-9);
    assert!((result.position_m[1] - 0.05).abs() < 1e-10);
    let mut different_metadata = axle();
    different_metadata.base_y_m = -1.;
    different_metadata.rod_length_m = -0.2;
    let same = dwb_geometry(&different_metadata, [0., 1., 0.])
        .unwrap()
        .solve(0.05, [0.; 3], None)
        .unwrap();
    assert_eq!(result.position_m, same.position_m);
    assert_eq!(result.orientation_xyzw, same.orientation_xyzw);
}
#[test]
fn unsupported_and_incomplete_source_geometry_is_rejected() {
    let mut source = axle();
    source.kind = "STRUT".into();
    assert!(dwb_geometry(&source, [0., 1., 0.]).is_err());
    source.kind = "DWB".into();
    for key in source.hardpoints.keys().cloned().collect::<Vec<_>>() {
        let mut missing = axle();
        missing.hardpoints.remove(&key);
        assert!(dwb_geometry(&missing, [0., 1., 0.]).is_err(), "{key}");
        assert!(dwb_geometry_with_reflection(&missing, [0., 1., 0.], true).is_err());
    }
}

fn polar(v: [f64; 3]) -> [f64; 3] {
    [-v[0], v[1], v[2]]
}
fn axial(v: [f64; 3]) -> [f64; 3] {
    [v[0], -v[1], -v[2]]
}
fn near(a: [f64; 3], b: [f64; 3]) {
    assert!(
        (glam::DVec3::from_array(a) - glam::DVec3::from_array(b)).length() < 2e-7,
        "{a:?} != {b:?}"
    );
}
fn reflected_sample(
    strut: bool,
    reflected: bool,
    travel: f64,
    rack: [f64; 3],
) -> (
    asc_physics::LinkPose,
    asc_physics::LinkDifferential,
    asc_physics::RelativeAcceleration,
) {
    let mut source = if strut { strut_axle() } else { axle() };
    source.hardpoints.get_mut("WBCAR_BOTTOM_FRONT").unwrap()[2] += 0.013;
    source.hardpoints.get_mut("WBCAR_STEER").unwrap()[1] += 0.009;
    if strut {
        source.hardpoints.get_mut("STRUT_CAR").unwrap()[0] -= 0.07;
    }
    let rack = if reflected { polar(rack) } else { rack };
    let sign = if reflected { -1. } else { 1. };
    let rates = [0.12, sign * 0.03, -0.01, 0.02];
    let accelerations = [-0.4, sign * 0.07, 0.03, -0.02];
    macro_rules! sample {
        ($mechanism:expr) => {{
            let mechanism = $mechanism.unwrap();
            let pose = mechanism.solve(travel, rack, None).unwrap().pose().unwrap();
            let differential = mechanism.differential(travel, rack, pose).unwrap();
            let acceleration = mechanism
                .acceleration(travel, rack, pose, rates, accelerations)
                .unwrap();
            (pose, differential, acceleration)
        }};
    }
    if strut {
        sample!(strut_geometry_with_reflection(
            &source,
            [0., 1., 0.],
            reflected
        ))
    } else {
        sample!(dwb_geometry_with_reflection(
            &source,
            [0., 1., 0.],
            reflected
        ))
    }
}
#[test]
fn reflected_dwb_and_strut_preserve_polar_axial_kinematics_and_acceleration() {
    use glam::{DMat3, DQuat, DVec3};
    let reflection = DMat3::from_diagonal(DVec3::new(-1., 1., 1.));
    for strut in [false, true] {
        for travel in [-0.03, 0., 0.025] {
            let (lp, l, la) = reflected_sample(strut, false, travel, [0.002, 0.001, -0.001]);
            let (rp, r, ra) = reflected_sample(strut, true, travel, [0.002, 0.001, -0.001]);
            near(rp.position_m(), polar(lp.position_m()));
            let expected = reflection
                * DMat3::from_quat(DQuat::from_array(lp.orientation_xyzw()))
                * reflection;
            let actual = DMat3::from_quat(DQuat::from_array(rp.orientation_xyzw()));
            assert!(
                expected
                    .to_cols_array()
                    .iter()
                    .zip(actual.to_cols_array())
                    .all(|(a, b)| (a - b).abs() < 1e-8)
            );
            assert!((actual.determinant() - 1.).abs() < 1e-12);
            let lv = l.velocity(0.12, [0.03, -0.01, 0.02]).unwrap();
            let rv = r.velocity(0.12, [-0.03, -0.01, 0.02]).unwrap();
            near(rv.velocity_m_s(), polar(lv.velocity_m_s()));
            near(
                rv.angular_velocity_rad_s(),
                axial(lv.angular_velocity_rad_s()),
            );
            near(ra.acceleration_m_s2, polar(la.acceleration_m_s2));
            near(
                ra.angular_acceleration_rad_s2,
                axial(la.angular_acceleration_rad_s2),
            );
            let point = [0.04, -0.08, 0.11];
            near(
                rp.transform(polar(point)).unwrap(),
                polar(lp.transform(point).unwrap()),
            );
            near(
                rv.point_velocity_m_s(polar(point)).unwrap(),
                polar(lv.point_velocity_m_s(point).unwrap()),
            );
            near(
                ra.point_acceleration_m_s2(polar(point)).unwrap(),
                polar(la.point_acceleration_m_s2(point).unwrap()),
            );
        }
    }
}
#[test]
fn reflected_generalized_loads_and_constraint_wrenches_obey_virtual_work_and_chirality() {
    for strut in [false, true] {
        let (_, l, _) = reflected_sample(strut, false, 0.02, [0.002, 0.001, -0.001]);
        let (_, r, _) = reflected_sample(strut, true, 0.02, [0.002, 0.001, -0.001]);
        let force = [103., -217., 59.];
        let torque = [7., 11., -13.];
        let left = asc_physics::LinkLoad::new(force, torque).unwrap();
        let right = asc_physics::LinkLoad::new(polar(force), axial(torque)).unwrap();
        let lg = l.project_load(left).unwrap();
        let rg = r.project_load(right).unwrap();
        assert!((lg.travel_n - rg.travel_n).abs() < 2e-7);
        near(rg.steering_anchor_n, polar(lg.steering_anchor_n));
        let lb = l.hold_load(left).unwrap();
        let rb = r.hold_load(right).unwrap();
        near(
            rb.support_force_on_frame_n,
            polar(lb.support_force_on_frame_n),
        );
        near(
            rb.support_torque_about_frame_origin_nm,
            axial(lb.support_torque_about_frame_origin_nm),
        );
        near(
            rb.steering_anchor_force_n,
            polar(lb.steering_anchor_force_n),
        );
        let power = |g: asc_physics::GeneralizedLoads, rates: [f64; 3]| {
            g.travel_n * 0.12
                + glam::DVec3::from_array(g.steering_anchor_n).dot(glam::DVec3::from_array(rates))
        };
        assert!((power(lg, [0.03, -0.01, 0.02]) - power(rg, [-0.03, -0.01, 0.02])).abs() < 1e-7);
    }
}
