use asc_physics::{
    Damper, DamperBranch, FiveLink, LinkLoad, LinkPose, LinkSpring, MassProperties, StrutLink,
    TravelMechanism, TravelState, TravelSystem,
};
use glam::{DQuat, DVec3};

fn geometries(t: DVec3, r: DQuat) -> [TravelMechanism; 2] {
    let transform = |p: [f64; 3]| (t + r * DVec3::from_array(p)).to_array();
    let reference = LinkPose::new(t.to_array(), r.to_array()).unwrap();
    let axis = (r * DVec3::Y).to_array();
    [
        TravelMechanism::Dwb(
            FiveLink::new(
                [
                    [-0.3, 0.2, 0.12],
                    [-0.3, 0.2, -0.12],
                    [-0.3, -0.2, 0.12],
                    [-0.3, -0.2, -0.12],
                    [-0.3, 0., 0.15],
                ]
                .map(transform),
                [
                    [0., 0.2, 0.],
                    [0., 0.2, 0.],
                    [0., -0.2, 0.],
                    [0., -0.2, 0.],
                    [0., 0., 0.15],
                ],
                reference,
                axis,
            )
            .unwrap(),
        ),
        TravelMechanism::Strut(
            StrutLink::new(
                [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]].map(transform),
                [[0.; 3], [0.; 3], [0., 0., 0.15]],
                transform([0., 0.6, 0.]),
                [0.; 3],
                reference,
                axis,
            )
            .unwrap(),
        ),
    ]
}
fn pose(m: &TravelMechanism, h: f64, rack: [f64; 3]) -> LinkPose {
    match m {
        TravelMechanism::Dwb(m) => m.solve(h, rack, None).unwrap().pose().unwrap(),
        TravelMechanism::Strut(m) => m.solve(h, rack, None).unwrap().pose().unwrap(),
    }
}
fn acceleration(
    m: &TravelMechanism,
    h: f64,
    rack: [f64; 3],
    rates: [f64; 4],
    a: [f64; 4],
) -> asc_physics::Result<asc_physics::RelativeAcceleration> {
    let p = pose(m, h, rack);
    match m {
        TravelMechanism::Dwb(m) => m.acceleration(h, rack, p, rates, a),
        TravelMechanism::Strut(m) => m.acceleration(h, rack, p, rates, a),
    }
}
fn properties() -> MassProperties {
    MassProperties::new(
        35.,
        [[0.4, 0.03, 0.02], [0.03, 0.5, -0.01], [0.02, -0.01, 0.6]],
    )
    .unwrap()
}
fn zero() -> LinkLoad {
    LinkLoad::new([0.; 3], [0.; 3]).unwrap()
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() < tol, "{a} != {b}; tolerance {tol}");
}
fn vector(a: [f64; 3]) -> DVec3 {
    DVec3::from_array(a)
}
fn close(a: [f64; 3], b: DVec3, tol: f64) {
    assert!((vector(a) - b).length() < tol, "{a:?} != {b:?}");
}
fn system(m: TravelMechanism, h: f64, v: f64) -> TravelSystem {
    TravelSystem::new(
        m,
        properties(),
        [0.04, -0.03, 0.02],
        [0.; 3],
        [0.; 3],
        TravelState {
            travel_m: h,
            rate_m_s: v,
        },
    )
    .unwrap()
}
fn spring(anchor: [f64; 3]) -> LinkSpring {
    let b = DamperBranch::new(100., 100., 0.).unwrap();
    LinkSpring::new(anchor, [0.; 3], 0.6, 10000., Damper::new(b, b)).unwrap()
}

#[test]
fn accelerations_match_closed_form_circle_and_telescoping_orientation() {
    let h = 0.04_f64;
    let speed = 0.2;
    let a = -0.7;
    let root = (0.09 - h * h).sqrt();
    let x = root - 0.3;
    let dx = -h / root;
    let ddx = -0.09 / root.powi(3);
    let numerator = (0.6 - h) * dx + x;
    let denominator = (0.6 - h).powi(2) + x * x;
    let theta_h = numerator / denominator;
    let theta_hh = ((0.6 - h) * ddx * denominator - numerator * (-2. * (0.6 - h) + 2. * x * dx))
        / denominator.powi(2);
    for m in geometries(DVec3::ZERO, DQuat::IDENTITY) {
        let result = acceleration(&m, h, [0.; 3], [speed, 0., 0., 0.], [a, 0., 0., 0.]).unwrap();
        close(
            result.acceleration_m_s2,
            DVec3::new(dx * a + ddx * speed * speed, a, 0.),
            1e-8,
        );
        let angular = if matches!(m, TravelMechanism::Strut(_)) {
            theta_h * a + theta_hh * speed * speed
        } else {
            0.
        };
        close(
            result.angular_acceleration_rad_s2,
            DVec3::new(0., 0., angular),
            1e-8,
        );
    }
}

#[test]
fn full_control_accelerations_and_local_points_match_independent_pose_differences() {
    let rates = [0.17, 0.025, -0.012, 0.009];
    let input_accel = [-0.3, 0.04, 0.02, -0.01];
    let coordinates = [0.03, 0.003, 0.001, -0.002];
    let eps = 0.001;
    for m in geometries(DVec3::ZERO, DQuat::IDENTITY) {
        let result = acceleration(
            &m,
            coordinates[0],
            coordinates[1..].try_into().unwrap(),
            rates,
            input_accel,
        )
        .unwrap();
        let at = |time: f64| {
            let c: [f64; 4] = std::array::from_fn(|i| {
                coordinates[i] + rates[i] * time + 0.5 * input_accel[i] * time * time
            });
            pose(&m, c[0], c[1..].try_into().unwrap())
        };
        let low = at(-eps);
        let middle = at(0.);
        let high = at(eps);
        close(
            result.acceleration_m_s2,
            (vector(high.position_m()) - 2. * vector(middle.position_m())
                + vector(low.position_m()))
                / (eps * eps),
            2e-4,
        );
        let mid_q = DQuat::from_array(middle.orientation_xyzw());
        let dq = |p: LinkPose| {
            (DQuat::from_array(p.orientation_xyzw()) * mid_q.conjugate()).to_scaled_axis()
        };
        close(
            result.angular_acceleration_rad_s2,
            (dq(high) + dq(low)) / (eps * eps),
            2e-4,
        );
        let point = [0.07, -0.08, 0.1];
        close(
            result.point_acceleration_m_s2(point).unwrap(),
            (vector(high.transform(point).unwrap())
                - 2. * vector(middle.transform(point).unwrap())
                + vector(low.transform(point).unwrap()))
                / (eps * eps),
            2e-4,
        );
        assert!(
            acceleration(
                &m,
                coordinates[0],
                coordinates[1..].try_into().unwrap(),
                [f64::NAN, 0., 0., 0.],
                input_accel
            )
            .is_err()
        );
        assert!(
            acceleration(
                &m,
                coordinates[0],
                coordinates[1..].try_into().unwrap(),
                rates,
                [1e7, 0., 0., 0.]
            )
            .is_err()
        );
    }
}

#[test]
fn free_circular_motion_has_correct_metric_bias_reactions_and_second_order_trajectory() {
    let geometry = geometries(DVec3::ZERO, DQuat::IDENTITY)[0].clone();
    let initial = system(geometry.clone(), 0.04, 0.1).sample(zero()).unwrap();
    let h = 0.04_f64;
    let rate = 0.1;
    near(initial.effective_mass_kg, 35. * 0.09 / (0.09 - h * h), 1e-8);
    near(
        initial.velocity_bias_n,
        35. * 0.09 * h * rate * rate / (0.09 - h * h).powi(2),
        1e-8,
    );
    near(
        initial.acceleration_m_s2,
        -h * rate * rate / (0.09 - h * h),
        1e-8,
    );
    close(
        initial.constraint_balance.support_force_on_upright_n,
        35. * vector(initial.center_of_mass_acceleration_m_s2),
        1e-8,
    );
    assert!(initial.constraint_balance.travel_actuator_n.abs() < 1e-9);
    let duration = 0.4;
    let angular_rate = rate / (0.09 - h * h).sqrt();
    let theta = (h / 0.3).asin() + angular_rate * duration;
    let exact_h = 0.3 * theta.sin();
    let exact_rate = 0.3 * theta.cos() * angular_rate;
    let mut errors = Vec::new();
    for count in [40, 80, 160] {
        let mut dynamic = system(geometry.clone(), h, rate);
        let mut sample = dynamic.sample(zero()).unwrap();
        for _ in 0..count {
            sample = dynamic.step(duration / f64::from(count), zero()).unwrap();
        }
        errors.push(
            (sample.state.travel_m - exact_h).abs() + (sample.state.rate_m_s - exact_rate).abs(),
        );
        near(sample.kinetic_energy_j, initial.kinetic_energy_j, 2e-6);
    }
    assert!(
        errors[0] / errors[1] > 3.7 && errors[1] / errors[2] > 3.7,
        "{errors:?}"
    );
    assert!(errors[2] < 2e-8, "{errors:?}");
}

#[test]
fn gravity_com_tensor_and_velocity_bias_obey_lagrangian_energy_identity() {
    for m in geometries(DVec3::ZERO, DQuat::IDENTITY) {
        let h = 0.03;
        let rate = 0.17;
        let eps = 1e-5;
        let make = |h| {
            TravelSystem::new(
                m.clone(),
                properties(),
                [0.04, -0.03, 0.02],
                [0.003, 0.002, -0.001],
                [0., -9.81, 0.],
                TravelState {
                    travel_m: h,
                    rate_m_s: rate,
                },
            )
            .unwrap()
        };
        let sample = make(h).sample(zero()).unwrap();
        let low = make(h - eps).sample(zero()).unwrap();
        let high = make(h + eps).sample(zero()).unwrap();
        let derivative = (high.effective_mass_kg - low.effective_mass_kg) / (2. * eps);
        near(sample.velocity_bias_n, 0.5 * derivative * rate * rate, 1e-5);
        let potential_derivative = (high.potential_energy_j - low.potential_energy_j) / (2. * eps);
        near(sample.generalized_load_n, -potential_derivative, 1e-5);
        near(
            sample.kinetic_energy_j,
            0.5 * sample.effective_mass_kg * rate * rate,
            1e-9,
        );
        near(
            sample.effective_mass_kg * sample.acceleration_m_s2 + sample.velocity_bias_n,
            sample.generalized_load_n,
            1e-8,
        );
        let pose =
            LinkPose::new(sample.upright_position_m, sample.upright_orientation_xyzw).unwrap();
        let rotation = DQuat::from_array(pose.orientation_xyzw());
        let lever = rotation * DVec3::new(0.04, -0.03, 0.02);
        let rows = properties().inertia_rows_kg_m2();
        let inertia = |v: DVec3| {
            let local = rotation.conjugate() * v;
            rotation * DVec3::from_array(rows.map(|row| vector(row).dot(local)))
        };
        let w = vector(sample.angular_velocity_rad_s);
        let alpha = vector(sample.angular_acceleration_rad_s2);
        let gravity = DVec3::new(0., -35. * 9.81, 0.);
        let inertial_force = 35. * vector(sample.center_of_mass_acceleration_m_s2);
        close(
            sample.constraint_balance.support_force_on_upright_n,
            inertial_force - gravity,
            1e-7,
        );
        close(
            sample.constraint_balance.support_torque_on_upright_nm,
            inertia(alpha) + w.cross(inertia(w)) + lever.cross(inertial_force - gravity),
            1e-7,
        );
    }
}

#[test]
fn damped_spring_motion_loses_energy_and_preserves_constraints_for_both_topologies() {
    for m in geometries(DVec3::ZERO, DQuat::IDENTITY) {
        let mut dynamic = system(m.clone(), 0.03, 0.);
        dynamic.add_spring(spring([0., 0.6, 0.])).unwrap();
        let initial = dynamic.sample(zero()).unwrap();
        let initial_energy = initial.kinetic_energy_j + initial.potential_energy_j;
        let mut previous = initial_energy;
        for _ in 0..1000 {
            let sample = dynamic.step(0.0005, zero()).unwrap();
            let energy = sample.kinetic_energy_j + sample.potential_energy_j;
            assert!(energy <= previous + 1e-7, "{energy} > {previous}");
            let independent_pose = pose(&m, sample.state.travel_m, [0.; 3]);
            close(
                sample.upright_position_m,
                vector(independent_pose.position_m()),
                1e-9,
            );
            assert!(sample.constraint_balance.travel_actuator_n.abs() < 1e-7);
            previous = energy;
        }
        assert!(
            previous < initial_energy * 0.4,
            "{previous} / {initial_energy}"
        );
    }
}

#[test]
fn dynamic_steps_and_support_moments_are_covariant_under_fixed_frame_changes() {
    let r = DQuat::from_euler(glam::EulerRot::XYZ, 0.3, -0.2, 0.4);
    let t = DVec3::new(3., -2., 4.);
    for (base, moved) in geometries(DVec3::ZERO, DQuat::IDENTITY)
        .into_iter()
        .zip(geometries(t, r))
    {
        let make = |m, gravity| {
            TravelSystem::new(
                m,
                properties(),
                [0.04, -0.03, 0.02],
                [0.; 3],
                gravity,
                TravelState {
                    travel_m: 0.03,
                    rate_m_s: 0.1,
                },
            )
            .unwrap()
        };
        let mut a = make(base, [0., -9.81, 0.]);
        let mut b = make(moved, (r * DVec3::new(0., -9.81, 0.)).to_array());
        a.add_spring(spring([0., 0.6, 0.])).unwrap();
        b.add_spring(spring((t + r * DVec3::new(0., 0.6, 0.)).to_array()))
            .unwrap();
        let f = DVec3::new(3., 5., 7.);
        let torque = DVec3::new(0.1, 0.2, -0.3);
        for _ in 0..40 {
            let first = a
                .step(
                    0.001,
                    LinkLoad::new(f.to_array(), torque.to_array()).unwrap(),
                )
                .unwrap();
            let second = b
                .step(
                    0.001,
                    LinkLoad::new((r * f).to_array(), (r * torque).to_array()).unwrap(),
                )
                .unwrap();
            near(first.state.travel_m, second.state.travel_m, 1e-9);
            near(first.state.rate_m_s, second.state.rate_m_s, 1e-8);
            near(first.kinetic_energy_j, second.kinetic_energy_j, 1e-7);
            close(
                second.center_of_mass_position_m,
                t + r * vector(first.center_of_mass_position_m),
                1e-8,
            );
            close(
                second.center_of_mass_acceleration_m_s2,
                r * vector(first.center_of_mass_acceleration_m_s2),
                1e-7,
            );
            close(
                second.constraint_balance.support_force_on_frame_n,
                r * vector(first.constraint_balance.support_force_on_frame_n),
                1e-6,
            );
            close(
                second
                    .constraint_balance
                    .support_torque_about_frame_origin_nm,
                r * vector(
                    first
                        .constraint_balance
                        .support_torque_about_frame_origin_nm,
                ) + t.cross(r * vector(first.constraint_balance.support_force_on_frame_n)),
                1e-5,
            );
        }
    }
}

#[test]
fn invalid_inputs_failed_final_stage_and_failed_spring_add_leave_state_unchanged() {
    for m in geometries(DVec3::ZERO, DQuat::IDENTITY) {
        assert!(
            TravelSystem::new(
                m.clone(),
                properties(),
                [101., 0., 0.],
                [0.; 3],
                [0.; 3],
                TravelState {
                    travel_m: 0.,
                    rate_m_s: 0.
                }
            )
            .is_err()
        );
        assert!(
            TravelSystem::new(
                m.clone(),
                properties(),
                [0.; 3],
                [0.; 3],
                [f64::NAN, 0., 0.],
                TravelState {
                    travel_m: 0.,
                    rate_m_s: 0.
                }
            )
            .is_err()
        );
        let mut dynamic = system(m, 0., 0.);
        let before = serde_json::to_string(&dynamic.sample(zero()).unwrap()).unwrap();
        for dt in [f64::NAN, 0., 0.1] {
            assert!(dynamic.step(dt, zero()).is_err());
        }
        let force = LinkLoad::new([0., 100000., 0.], [0.; 3]).unwrap();
        assert!(dynamic.step(0.02, force).is_err());
        let collapsed = spring([0.; 3]);
        assert!(dynamic.add_spring(collapsed).is_err());
        assert_eq!(
            serde_json::to_string(&dynamic.sample(zero()).unwrap()).unwrap(),
            before
        );
        dynamic.step(1e-4, force).unwrap();
        assert_ne!(dynamic.state().travel_m, 0.);
    }
}
