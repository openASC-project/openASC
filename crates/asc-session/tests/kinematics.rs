use asc_session::{dwb_geometry, strut_geometry};
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
    }
}
