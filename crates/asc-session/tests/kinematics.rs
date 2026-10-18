use asc_session::dwb_geometry;
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
