use asc_physics::{Body, MassProperties, Wrench};
use glam::{DQuat, DVec3};

fn properties(diagonal: [f64; 3]) -> MassProperties {
    MassProperties::new(
        2.,
        [
            [diagonal[0], 0., 0.],
            [0., diagonal[1], 0.],
            [0., 0., diagonal[2]],
        ],
    )
    .unwrap()
}
fn body(diagonal: [f64; 3], q: DQuat, velocity: [f64; 3], omega: [f64; 3]) -> Body {
    Body::new(properties(diagonal), [0.; 3], q.to_array(), velocity, omega).unwrap()
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} != {b}, tolerance {tol}");
}
fn vector_near(a: [f64; 3], b: [f64; 3], tol: f64) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b, tol);
    }
}
fn angle(a: [f64; 4], b: DQuat) -> f64 {
    let relative = (DQuat::from_array(a) * b.conjugate()).to_array();
    2. * relative[0]
        .hypot(relative[1])
        .hypot(relative[2])
        .atan2(relative[3].abs())
}

#[test]
fn full_tensor_and_rotated_principal_axes_give_expected_momenta_and_energy() {
    let props = MassProperties::new(2., [[4., 1., 0.], [1., 4., 0.], [0., 0., 4.]]).unwrap();
    assert_eq!(props.mass_kg(), 2.);
    assert_eq!(
        props.inertia_rows_kg_m2(),
        [[4., 1., 0.], [1., 4., 0.], [0., 0., 4.]]
    );
    let b = Body::new(
        props,
        [0.; 3],
        DQuat::IDENTITY.to_array(),
        [3., 0., 0.],
        [1., 2., 0.],
    )
    .unwrap();
    let s = b.state();
    vector_near(s.angular_momentum_world_kg_m2_s, [6., 9., 0.], 1e-12);
    near(s.kinetic_energy_j, 21., 1e-12);
    let b = body(
        [2., 3., 4.],
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        [0.; 3],
        [1., 2., 3.],
    );
    let s = b.state();
    vector_near(s.angular_momentum_world_kg_m2_s, [3., 4., 12.], 1e-12);
    vector_near(s.angular_velocity_world_rad_s, [1., 2., 3.], 1e-12);
    near(s.kinetic_energy_j, 23.5, 1e-12);
}

#[test]
fn invalid_or_unrealizable_tensors_are_rejected_before_state_creation() {
    for mass in [0., -1., f64::NAN, f64::INFINITY, 1e13] {
        assert_eq!(
            MassProperties::new(mass, [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
                .unwrap_err()
                .code,
            "physics_inertia"
        );
    }
    for rows in [
        [[1., 0., 0.], [0., 1., 0.], [0., 0., 3.]], // SPD, but impossible principal-moment triangle.
        [[1., 2., 0.], [2., 1., 0.], [0., 0., 1.]],
        [[1., 0.1, 0.], [0., 1., 0.], [0., 0., 1.]],
        [[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        [[1e-15, 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        [
            [1.000000005, 1.5e-8, 0.],
            [1.5e-8, 1.000000005, 0.],
            [0., 0., 1e-8],
        ],
        [[1., 0., 0.], [0., f64::INFINITY, 0.], [0., 0., 1.]],
    ] {
        assert_eq!(
            MassProperties::new(2., rows).unwrap_err().code,
            "physics_inertia"
        );
    }
    // A planar mass distribution is allowed: its largest moment equals the other two.
    assert!(MassProperties::new(2., [[1., 0., 0.], [0., 1., 0.], [0., 0., 2.]]).is_ok());
}

#[test]
fn constant_force_matches_analytic_translation_and_zero_load_does_not_add_gravity() {
    let mut b = body([2., 3., 4.], DQuat::IDENTITY, [1., 2., 3.], [0.; 3]);
    let load = Wrench::new([4., -19.62, 6.], [0.; 3]).unwrap();
    for _ in 0..120 {
        b.step(1. / 120., load).unwrap();
    }
    let s = b.state();
    vector_near(s.position_m, [2., 2. - 4.905, 4.5], 1e-11);
    vector_near(s.velocity_world_m_s, [3., 2. - 9.81, 6.], 1e-11);
    vector_near(s.linear_momentum_world_kg_m_s, [6., 4. - 19.62, 12.], 1e-11);
    assert_eq!(s.orientation_xyzw, DQuat::IDENTITY.to_array());
    let mut rest = body([2., 3., 4.], DQuat::IDENTITY, [0.; 3], [0.; 3]);
    let initial = rest.state();
    for _ in 0..120 {
        rest.step(1. / 120., Wrench::default()).unwrap();
    }
    assert_eq!(rest.state(), initial);
    let speed = s.velocity_world_m_s;
    b.step(1. / 120., Wrench::default()).unwrap();
    assert_eq!(b.state().velocity_world_m_s, speed);
}

#[test]
fn spherical_body_under_constant_torque_has_exact_axis_angle_and_angular_impulse() {
    let mut b = body([3.; 3], DQuat::IDENTITY, [0.; 3], [0.; 3]);
    let torque = DVec3::new(1., 2., 3.);
    let load = Wrench::new([0.; 3], torque.to_array()).unwrap();
    for _ in 0..240 {
        b.step(1. / 240., load).unwrap();
    }
    let s = b.state();
    vector_near(s.angular_momentum_world_kg_m2_s, torque.to_array(), 1e-12);
    let exact = DQuat::from_scaled_axis(torque / 6.);
    assert!(angle(s.orientation_xyzw, exact) < 1e-7);
    vector_near(
        s.angular_velocity_world_rad_s,
        (torque / 3.).to_array(),
        1e-12,
    );
}

#[test]
fn torque_free_symmetric_top_converges_to_closed_form_precession() {
    let momentum = DVec3::new(2., 0., 8.);
    let duration = 2.;
    let exact = DQuat::from_axis_angle(momentum.normalize(), momentum.length() / 2. * duration)
        * DQuat::from_rotation_z(-2. * duration);
    let mut errors = Vec::new();
    for hz in [120, 240, 480] {
        let mut b = body([2., 2., 4.], DQuat::IDENTITY, [0.; 3], [1., 0., 2.]);
        let initial = b.state();
        for _ in 0..hz * 2 {
            b.step(1. / f64::from(hz), Wrench::default()).unwrap();
        }
        let state = b.state();
        assert_eq!(
            state.angular_momentum_world_kg_m2_s,
            initial.angular_momentum_world_kg_m2_s
        );
        near(
            DQuat::from_array(state.orientation_xyzw).length(),
            1.,
            1e-14,
        );
        errors.push(angle(state.orientation_xyzw, exact));
    }
    assert!(errors[1] < errors[0] * 0.35, "{errors:?}");
    assert!(errors[2] < errors[1] * 0.35, "{errors:?}");
    assert!(errors[2] < 1e-4, "{errors:?}");
    eprintln!("symmetric top orientation errors at 120/240/480 Hz: {errors:?} rad");
}

#[test]
fn asymmetric_free_rotation_preserves_world_momenta_and_bounds_energy_drift() {
    let mut b = body(
        [2., 3., 4.],
        DQuat::from_rotation_x(0.4),
        [1., 2., 3.],
        [1.2, 0.7, 1.8],
    );
    let initial = b.state();
    let rotational_energy = initial.kinetic_energy_j - 14.;
    let mut max_drift = 0f64;
    for _ in 0..2400 {
        b.step(1. / 240., Wrench::default()).unwrap();
        let state = b.state();
        assert_eq!(
            state.angular_momentum_world_kg_m2_s,
            initial.angular_momentum_world_kg_m2_s
        );
        assert_eq!(
            state.linear_momentum_world_kg_m_s,
            initial.linear_momentum_world_kg_m_s
        );
        max_drift = max_drift
            .max((state.kinetic_energy_j - initial.kinetic_energy_j).abs() / rotational_energy);
    }
    assert!(max_drift < 1e-4, "relative energy drift {max_drift}");
    eprintln!("asymmetric free-body maximum relative rotational energy drift: {max_drift}");
    vector_near(b.state().position_m, [10., 20., 30.], 1e-10);
}

#[test]
fn lever_arms_force_couples_impulses_and_point_velocity_use_world_com_conventions() {
    let mut b = Body::new(
        properties([2., 3., 4.]),
        [5., 4., 3.],
        DQuat::IDENTITY.to_array(),
        [0.; 3],
        [0.; 3],
    )
    .unwrap();
    let mut couple = b.force_at_world_point([0., 0., 3.], [7., 4., 3.]).unwrap();
    couple
        .add(b.force_at_world_point([0., 0., -3.], [3., 4., 3.]).unwrap())
        .unwrap();
    assert_eq!(couple.force_world_n(), [0.; 3]);
    assert_eq!(couple.torque_world_nm(), [0., -12., 0.]);
    b.step(1. / 120., couple).unwrap();
    assert_eq!(b.state().position_m, [5., 4., 3.]);
    near(b.state().angular_momentum_world_kg_m2_s[1], -0.1, 1e-14);
    let mut b = Body::new(
        properties([2., 3., 4.]),
        [5., 4., 3.],
        DQuat::IDENTITY.to_array(),
        [0.; 3],
        [0.; 3],
    )
    .unwrap();
    b.impulse_at_world_point([0., 0., 6.], [6., 4., 3.])
        .unwrap();
    let state = b.state();
    assert_eq!(state.linear_momentum_world_kg_m_s, [0., 0., 6.]);
    assert_eq!(state.angular_momentum_world_kg_m2_s, [0., -6., 0.]);
    assert_eq!(state.orientation_xyzw, DQuat::IDENTITY.to_array());
    vector_near(
        b.point_velocity_world([6., 4., 3.]).unwrap(),
        [0., 0., 5.],
        1e-12,
    );
    near(state.kinetic_energy_j, 15., 1e-12);
    let b = body(
        [2., 3., 4.],
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
        [0.; 3],
        [0.; 3],
    );
    let local = b.force_at_body_point([0., 0., 6.], [1., 0., 0.]).unwrap();
    vector_near(local.force_world_n(), [0., 0., 6.], 1e-12);
    vector_near(local.torque_world_nm(), [6., 0., 0.], 1e-12);
}

#[test]
fn rejected_steps_impulses_load_sums_and_bad_initial_states_are_transactional() {
    for q in [[0.; 4], [2., 0., 0., 0.], [f64::NAN, 0., 0., 1.]] {
        assert_eq!(
            Body::new(properties([2., 3., 4.]), [0.; 3], q, [0.; 3], [0.; 3])
                .unwrap_err()
                .code,
            "physics_orientation"
        );
    }
    let mut b = body([2., 3., 4.], DQuat::IDENTITY, [0.; 3], [0.; 3]);
    let initial = b.state();
    for dt in [0., -0.1, f64::NAN, 0.1] {
        assert_eq!(
            b.step(dt, Wrench::default()).unwrap_err().code,
            "physics_step_range"
        );
        assert_eq!(b.state(), initial);
    }
    assert!(
        b.step(1. / 30., Wrench::new([1e18, 0., 0.], [0.; 3]).unwrap())
            .is_err()
    );
    assert_eq!(b.state(), initial);
    assert!(b.impulse_at_world_point([1e18, 0., 0.], [0.; 3]).is_err());
    assert_eq!(b.state(), initial);
    let mut load = Wrench::new([1e18, 0., 0.], [0.; 3]).unwrap();
    let before = load.force_world_n();
    assert!(load.add(load).is_err());
    assert_eq!(load.force_world_n(), before);
    let mut fast = body([2., 3., 4.], DQuat::IDENTITY, [0.; 3], [1000., 0., 0.]);
    let before = fast.state();
    assert_eq!(
        fast.step(1. / 120., Wrench::default()).unwrap_err().code,
        "physics_rotation_step"
    );
    assert_eq!(fast.state(), before);
}
