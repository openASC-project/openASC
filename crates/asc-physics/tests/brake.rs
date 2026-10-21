use asc_physics::{WheelBrake, WheelLoad, Wrench};

#[test]
fn brake_is_bounded_passive_symmetric_continuous_and_zero_at_rest() {
    let b = WheelBrake::new(100., 5.).unwrap();
    for input in [0., 0.25, 1.] {
        for rate in [
            -100., -5.000001, -5., -4.999999, -1., 0., 1., 4.999999, 5., 5.000001, 100.,
        ] {
            let s = b.sample(rate, input).unwrap();
            let expected = -input * 100. * (rate / 5.).clamp(-1., 1.);
            assert!((s.torque_on_wheel_nm - expected).abs() < 1e-12);
            assert_eq!(s.torque_on_upright_nm, -s.torque_on_wheel_nm);
            assert!(s.mechanical_power_w <= 0.);
            assert_eq!(s.mechanical_power_w, s.torque_on_wheel_nm * rate);
            assert_eq!(s.dissipated_power_w, -s.mechanical_power_w);
            assert!(s.torque_on_wheel_nm.abs() <= s.torque_limit_nm);
        }
    }
    assert_eq!(
        WheelBrake::new(0., 1.)
            .unwrap()
            .sample(100., 1.)
            .unwrap()
            .torque_on_wheel_nm,
        0.
    );
}

#[test]
fn invalid_brakes_and_load_updates_fail_without_overwriting_valid_actuation() {
    for (torque, rate) in [(-1., 1.), (1e7, 1.), (1., 0.), (1., 1001.), (f64::NAN, 1.)] {
        assert!(WheelBrake::new(torque, rate).is_err());
    }
    let b = WheelBrake::new(100., 5.).unwrap();
    for (rate, input) in [
        (1., -0.01),
        (1., 1.01),
        (1e6, 1.),
        (f64::NAN, 1.),
        (1., f64::INFINITY),
    ] {
        assert!(b.sample(rate, input).is_err());
    }
    let mut load = WheelLoad::new(Wrench::default(), 50.).unwrap();
    load.add_internal_torque(-20.).unwrap();
    assert_eq!(load.internal_torque_nm(), 30.);
    assert!(load.add_internal_torque(f64::INFINITY).is_err());
    assert_eq!(load.internal_torque_nm(), 30.);
    assert!(load.set_internal_torque(1e19).is_err());
    assert_eq!(load.internal_torque_nm(), 30.);
    load.set_internal_torque(-1e18).unwrap();
    assert!(load.add_internal_torque(-1e18).is_err());
    assert_eq!(load.internal_torque_nm(), -1e18);
}
