use asc_physics::{
    Body, Damper, DamperBranch, MassProperties, SpringConnection, SpringLink, Wrench, step_springs,
};
use glam::{DQuat, DVec3};
fn body(mass: f64, position: [f64; 3], velocity: [f64; 3], omega: [f64; 3]) -> Body {
    Body::new(
        MassProperties::new(mass, [[2., 0., 0.], [0., 2., 0.], [0., 0., 2.]]).unwrap(),
        position,
        [0., 0., 0., 1.],
        velocity,
        omega,
    )
    .unwrap()
}
fn linear(c: f64) -> Damper {
    let branch = DamperBranch::new(c, c, 1.).unwrap();
    Damper::new(branch, branch)
}
fn link(c: f64) -> SpringLink {
    SpringLink::new([0.; 3], [0.; 3], 1., 60., linear(c)).unwrap()
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} != {b}, tolerance {tol}");
}

#[test]
fn asymmetric_damper_is_continuous_at_both_knees_and_always_passive() {
    let d = Damper::new(
        DamperBranch::new(20., 5., 0.1).unwrap(),
        DamperBranch::new(30., 7., 0.2).unwrap(),
    );
    for (v, f) in [
        (-0.05, -1.),
        (-0.1, -2.),
        (-0.3, -3.),
        (0., 0.),
        (0.1, 3.),
        (0.2, 6.),
        (0.4, 7.4),
    ] {
        near(d.force_n(v).unwrap(), f, 1e-12);
    }
    for knee in [-0.1, 0.2] {
        near(
            d.force_n(knee - 1e-10).unwrap(),
            d.force_n(knee + 1e-10).unwrap(),
            1e-8,
        );
    }
    for i in -1000..=1000 {
        let v = f64::from(i) * 0.003;
        assert!(d.force_n(v).unwrap() * v >= 0.);
    }
    let d = Damper::new(
        DamperBranch::new(100., 4., 0.).unwrap(),
        DamperBranch::new(100., 6., 0.).unwrap(),
    );
    near(d.force_n(-2.).unwrap(), -8., 1e-12);
    near(d.force_n(2.).unwrap(), 12., 1e-12);
}

#[test]
fn endpoint_wrenches_obey_action_reaction_energy_and_point_velocity_conventions() {
    let rotated = Body::new(
        MassProperties::new(2., [[2., 0., 0.], [0., 2., 0.], [0., 0., 2.]]).unwrap(),
        [1., 2., 3.],
        DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2).to_array(),
        [0.; 3],
        [0.; 3],
    )
    .unwrap();
    for (a, b) in rotated
        .world_point([1., 0., 0.])
        .unwrap()
        .into_iter()
        .zip([1., 3., 3.])
    {
        near(a, b, 1e-12);
    }
    let a = body(2., [0.; 3], [0.; 3], [0., 0., 1.]);
    let b = body(3., [0., 2., 0.], [0.; 3], [0.; 3]);
    let link = SpringLink::new([1., 0., 0.], [-1., 0., 0.], 2., 10., linear(4.)).unwrap();
    let (fa, fb, s) = link.loads(&a, &b).unwrap();
    near(s.extension_m, 8f64.sqrt() - 2., 1e-12);
    near(s.extension_rate_m_s, -0.5f64.sqrt(), 1e-12);
    near(s.potential_energy_j, 5. * s.extension_m.powi(2), 1e-12);
    near(s.dissipated_power_w, 2., 1e-12);
    let force_a = DVec3::from_array(fa.force_world_n());
    let force_b = DVec3::from_array(fb.force_world_n());
    assert_eq!((force_a + force_b).to_array(), [0.; 3]);
    let total_torque = DVec3::from_array(fa.torque_world_nm())
        + DVec3::from_array(fb.torque_world_nm())
        + DVec3::new(0., 2., 0.).cross(force_b);
    assert!(total_torque.length() < 1e-12);
    let va = DVec3::from_array(a.point_velocity_world(s.point_a_world_m).unwrap());
    let vb = DVec3::from_array(b.point_velocity_world(s.point_b_world_m).unwrap());
    near(
        force_a.dot(va) + force_b.dot(vb) + s.elastic_tension_n * s.extension_rate_m_s,
        -s.dissipated_power_w,
        1e-12,
    );
}

#[test]
fn coupled_bodies_match_closed_form_damped_oscillation_with_second_order_convergence() {
    let omega = 46f64.sqrt();
    let exact = 0.2 * (-2f64).exp() * (omega.cos() + 2. / omega * omega.sin());
    let mut errors = Vec::new();
    for hz in [120, 240, 480] {
        let mut a = body(2., [0.; 3], [0.; 3], [0.; 3]);
        let mut b = body(3., [1.2, 0., 0.], [0.; 3], [0.; 3]);
        for _ in 0..hz {
            link(4.8)
                .step_pair(
                    &mut a,
                    &mut b,
                    1. / f64::from(hz),
                    Wrench::default(),
                    Wrench::default(),
                )
                .unwrap();
        }
        let x = a.state().position_m[0];
        let y = b.state().position_m[0];
        let relative = y - x - 1.;
        errors.push((relative - exact).abs());
        near((2. * x + 3. * y) / 5., 0.72, 1e-12);
        near(
            a.state().linear_momentum_world_kg_m_s[0] + b.state().linear_momentum_world_kg_m_s[0],
            0.,
            1e-12,
        );
    }
    assert!(errors[1] < errors[0] * 0.35, "{errors:?}");
    assert!(errors[2] < errors[1] * 0.35, "{errors:?}");
    assert!(errors[2] < 1e-5, "{errors:?}");
    eprintln!("coupled oscillator extension errors at 120/240/480 Hz: {errors:?} m");
}

#[test]
fn damped_energy_decreases_and_undamped_energy_stays_bounded_on_the_fixture() {
    for c in [0., 4.8] {
        let link = link(c);
        let mut a = body(2., [0.; 3], [0.; 3], [0.; 3]);
        let mut b = body(3., [1.2, 0., 0.], [0.; 3], [0.; 3]);
        let initial = link.loads(&a, &b).unwrap().2.potential_energy_j;
        let mut previous = initial;
        let mut drift = 0f64;
        for _ in 0..1200 {
            let s = link
                .step_pair(
                    &mut a,
                    &mut b,
                    1. / 240.,
                    Wrench::default(),
                    Wrench::default(),
                )
                .unwrap();
            assert!(s.dissipated_power_w >= 0.);
            let energy = a.state().kinetic_energy_j
                + b.state().kinetic_energy_j
                + link.loads(&a, &b).unwrap().2.potential_energy_j;
            if c > 0. {
                assert!(energy <= previous + 1e-12, "{energy} > {previous}");
            }
            drift = drift.max((energy - initial).abs() / initial);
            previous = energy;
        }
        if c == 0. {
            assert!(drift < 1e-3, "{drift}");
            eprintln!("undamped pair maximum relative energy error at 240 Hz: {drift}");
        } else {
            assert!(previous < 1e-7);
        }
    }
}

#[test]
fn pair_failures_leave_both_bodies_unchanged_including_second_body_final_rejection() {
    let link = link(0.);
    let mut a = body(2., [0.; 3], [0.; 3], [0.; 3]);
    let mut b = body(3., [1.2, 0., 0.], [0.; 3], [0.; 3]);
    let initial = (a.state(), b.state());
    for dt in [0., f64::NAN, 0.1, 1e-6] {
        assert_eq!(
            link.step_pair(&mut a, &mut b, dt, Wrench::default(), Wrench::default())
                .unwrap_err()
                .code,
            "physics_spring_step"
        );
        assert_eq!((a.state(), b.state()), initial);
    }
    let error = link
        .step_pair(
            &mut a,
            &mut b,
            1. / 120.,
            Wrench::default(),
            Wrench::new([540_000_000., 0., 0.], [0.; 3]).unwrap(),
        )
        .unwrap_err();
    assert_eq!(error.code, "physics_velocity");
    assert_eq!((a.state(), b.state()), initial);
    let coincident = body(3., [0.; 3], [0.; 3], [0.; 3]);
    assert_eq!(
        link.loads(&a, &coincident).unwrap_err().code,
        "physics_spring_geometry"
    );
}

#[test]
fn disabled_elements_and_invalid_numeric_inputs_are_explicit() {
    for (slow, fast, knee) in [(-1., 1., 1.), (1., f64::NAN, 1.), (1., 1., -1.)] {
        assert!(DamperBranch::new(slow, fast, knee).is_err());
    }
    assert!(linear(1.).force_n(f64::INFINITY).is_err());
    assert!(linear(1.).force_n(1_000_001.).is_err());
    for (rest, k) in [(0., 1.), (1., -1.), (1., f64::NAN)] {
        assert!(SpringLink::new([0.; 3], [0.; 3], rest, k, linear(0.)).is_err());
    }
    let a = body(2., [0.; 3], [0.; 3], [0.; 3]);
    let b = body(3., [1.2, 0., 0.], [0.; 3], [0.; 3]);
    let s = SpringLink::new([0.; 3], [0.; 3], 1., 0., linear(0.))
        .unwrap()
        .loads(&a, &b)
        .unwrap()
        .2;
    assert_eq!(s.force_on_a_world_n, [0.; 3]);
    assert_eq!(s.dissipated_power_w, 0.);
}

#[test]
fn shared_body_links_step_once_with_common_midpoint_states_and_atomic_topology_errors() {
    let link = SpringLink::new([0.; 3], [0.; 3], 1., 60., linear(0.)).unwrap();
    let links = [
        SpringConnection::new(0, 1, link.clone()).unwrap(),
        SpringConnection::new(0, 2, link.clone()).unwrap(),
    ];
    let mut bodies = [
        body(2., [0., 10., 0.], [0.; 3], [0.; 3]),
        body(1., [-1.2, 10., 0.], [0.; 3], [0.; 3]),
        body(1., [1.2, 10., 0.], [0.; 3], [0.; 3]),
    ];
    let gravity = [
        Wrench::new([0., -19.62, 0.], [0.; 3]).unwrap(),
        Wrench::new([0., -9.81, 0.], [0.; 3]).unwrap(),
        Wrench::new([0., -9.81, 0.], [0.; 3]).unwrap(),
    ];
    for _ in 0..240 {
        assert_eq!(
            step_springs(&mut bodies, &links, 1. / 240., &gravity)
                .unwrap()
                .len(),
            2
        );
    }
    near(bodies[0].state().position_m[0], 0., 1e-12);
    near(bodies[0].state().position_m[1], 5.095, 1e-11);
    let exact = 1. + 0.2 * 60f64.sqrt().cos();
    near(bodies[2].state().position_m[0], exact, 1e-4);
    near(bodies[1].state().position_m[0], -exact, 1e-4);
    for b in &bodies {
        near(b.state().position_m[1], 5.095, 1e-11);
    }
    let before = bodies.iter().map(Body::state).collect::<Vec<_>>();
    let invalid = [SpringConnection::new(0, 99, link.clone()).unwrap()];
    assert_eq!(
        step_springs(&mut bodies, &invalid, 1. / 240., &gravity)
            .unwrap_err()
            .code,
        "physics_spring_graph"
    );
    assert_eq!(bodies.iter().map(Body::state).collect::<Vec<_>>(), before);
    assert!(SpringConnection::new(0, 0, link).is_err());
    assert!(step_springs(&mut bodies, &links, 1. / 240., &gravity[..2]).is_err());
}
