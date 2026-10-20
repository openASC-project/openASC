use asc_physics::{
    Body, ChassisSample, ChassisSystem, Damper, DamperBranch, FiveLink, LinkPose, LinkSpring,
    MassProperties, MountedUpright, StrutLink, TravelMechanism, TravelState, Wheel, WheelLoad,
    WheelState, Wrench,
};
use glam::{DQuat, DVec3};
fn geometry(strut: bool) -> TravelMechanism {
    let reference = LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap();
    if strut {
        TravelMechanism::Strut(
            StrutLink::new(
                [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]],
                [[0.; 3], [0.; 3], [0., 0., 0.15]],
                [0., 0.6, 0.],
                [0.; 3],
                reference,
                [0., 1., 0.],
            )
            .unwrap(),
        )
    } else {
        TravelMechanism::Dwb(
            FiveLink::new(
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
                reference,
                [0., 1., 0.],
            )
            .unwrap(),
        )
    }
}
fn chassis_properties() -> MassProperties {
    MassProperties::new(300., [[220., 3., 5.], [3., 400., 7.], [5., 7., 350.]]).unwrap()
}
fn upright_properties() -> MassProperties {
    MassProperties::new(
        35.,
        [[0.4, 0.03, 0.02], [0.03, 0.5, -0.01], [0.02, -0.01, 0.6]],
    )
    .unwrap()
}
fn body(position: DVec3, q: DQuat, v: DVec3, w: DVec3) -> Body {
    Body::new(
        chassis_properties(),
        position.to_array(),
        q.to_array(),
        v.to_array(),
        w.to_array(),
    )
    .unwrap()
}
fn spring(rest: f64, damping: f64) -> LinkSpring {
    let b = DamperBranch::new(damping, damping, 0.).unwrap();
    LinkSpring::new([0., 0.6, 0.], [0.; 3], rest, 1000., Damper::new(b, b)).unwrap()
}
fn corner(strut: bool, frame: LinkPose, state: TravelState, rest: Option<f64>) -> MountedUpright {
    let mut corner = MountedUpright::new(
        geometry(strut),
        upright_properties(),
        [0.; 3],
        [0.; 3],
        frame,
        state,
    )
    .unwrap();
    if let Some(rest) = rest {
        corner.add_spring(spring(rest, 20.)).unwrap();
    }
    corner
}
fn one(strut: bool, rest: Option<f64>) -> ChassisSystem {
    ChassisSystem::new(
        body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
        vec![corner(
            strut,
            LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
            TravelState {
                travel_m: 0.,
                rate_m_s: 0.,
            },
            rest,
        )],
        [0.; 3],
    )
    .unwrap()
}
fn four(
    chassis: Body,
    states: [TravelState; 4],
    gravity: [f64; 3],
    springs: bool,
) -> ChassisSystem {
    let mounts = [
        [0.8, 0., 1.2],
        [-0.8, 0., 1.2],
        [0.8, 0., -1.2],
        [-0.8, 0., -1.2],
    ];
    let corners = mounts
        .into_iter()
        .enumerate()
        .map(|(i, p)| {
            let rotation = if i % 2 == 0 {
                DQuat::IDENTITY
            } else {
                DQuat::from_rotation_y(std::f64::consts::PI)
            };
            corner(
                i < 2,
                LinkPose::new(p, rotation.to_array()).unwrap(),
                states[i],
                if springs { Some(0.6) } else { None },
            )
        })
        .collect();
    ChassisSystem::new(chassis, corners, gravity).unwrap()
}
fn vector(a: [f64; 3]) -> DVec3 {
    DVec3::from_array(a)
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() < tol, "{a} != {b}; tolerance {tol}");
}
fn close(a: [f64; 3], b: DVec3, tol: f64) {
    assert!((vector(a) - b).length() < tol, "{a:?} != {b:?}");
}
fn total_energy(s: &ChassisSample) -> f64 {
    s.kinetic_energy_j + s.potential_energy_j
}
fn inertia(p: &MassProperties, q: DQuat, v: DVec3) -> DVec3 {
    let local = q.conjugate() * v;
    q * DVec3::from_array(p.inertia_rows_kg_m2().map(|r| vector(r).dot(local)))
}

#[test]
fn internal_spring_recoil_matches_two_mass_acceleration_and_preserves_momenta() {
    for strut in [false, true] {
        let mut system = one(strut, Some(0.58));
        let first = system
            .sample(Wrench::default(), &[Wrench::default()])
            .unwrap();
        close(
            first.acceleration_world_m_s2,
            DVec3::new(0., -20. / 300., 0.),
            1e-9,
        );
        close(
            first.uprights[0].acceleration_world_m_s2,
            DVec3::new(0., 20. / 35., 0.),
            1e-9,
        );
        near(
            first.uprights[0].travel_acceleration_m_s2,
            20. * (1. / 35. + 1. / 300.),
            1e-9,
        );
        close(first.angular_acceleration_world_rad_s2, DVec3::ZERO, 1e-9);
        let center = vector(first.center_of_mass_world_m);
        let initial_energy = total_energy(&first);
        let mut previous = initial_energy;
        for _ in 0..200 {
            let sample = system
                .step(0.0005, Wrench::default(), &[Wrench::default()])
                .unwrap();
            close(sample.linear_momentum_world_kg_m_s, DVec3::ZERO, 2e-6);
            close(
                sample.angular_momentum_about_world_origin_kg_m2_s,
                DVec3::ZERO,
                2e-6,
            );
            close(sample.center_of_mass_world_m, center, 1e-8);
            assert!(total_energy(&sample) <= previous + 1e-8);
            previous = total_energy(&sample);
        }
        assert!(previous < initial_energy);
    }
}

#[test]
fn all_four_corners_share_exact_free_fall_without_internal_constraint_loads() {
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let p = DVec3::new(3., 4., 5.);
    let v = DVec3::new(1., 2., -0.5);
    let g = DVec3::new(0., -9.81, 0.);
    let states = [TravelState {
        travel_m: 0.02,
        rate_m_s: 0.,
    }; 4];
    let mut system = four(body(p, q, v, DVec3::ZERO), states, g.to_array(), false);
    for step in 1..=100 {
        let sample = system
            .step(0.001, Wrench::default(), &[Wrench::default(); 4])
            .unwrap();
        let t = f64::from(step) * 0.001;
        close(
            sample.chassis.position_m,
            p + v * t + 0.5 * g * t * t,
            1e-10,
        );
        close(sample.chassis.velocity_world_m_s, v + g * t, 1e-9);
        close(sample.acceleration_world_m_s2, g, 1e-8);
        close(sample.angular_acceleration_world_rad_s2, DVec3::ZERO, 1e-8);
        close(
            sample.linear_momentum_world_kg_m_s,
            sample.total_mass_kg * (v + g * t),
            1e-7,
        );
        for upright in &sample.uprights {
            near(upright.state.travel_m, 0.02, 1e-10);
            near(upright.state.rate_m_s, 0., 1e-9);
            close(upright.acceleration_world_m_s2, g, 1e-8);
            close(upright.support_force_on_chassis_world_n, DVec3::ZERO, 1e-7);
        }
    }
}

fn diverse_states() -> [TravelState; 4] {
    [
        TravelState {
            travel_m: 0.03,
            rate_m_s: 0.12,
        },
        TravelState {
            travel_m: 0.02,
            rate_m_s: -0.08,
        },
        TravelState {
            travel_m: -0.01,
            rate_m_s: 0.04,
        },
        TravelState {
            travel_m: 0.015,
            rate_m_s: -0.02,
        },
    ]
}
#[test]
fn spinning_translating_four_corner_system_obeys_global_newton_euler_balance() {
    let g = DVec3::new(0.3, -9.81, 0.1);
    let chassis = body(
        DVec3::new(3., 4., 5.),
        DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1),
        DVec3::new(1., -0.2, 0.3),
        DVec3::new(0.2, -0.15, 0.3),
    );
    let system = four(chassis, diverse_states(), g.to_array(), true);
    let chassis_load = Wrench::new([13., -4., 7.], [1., 2., -3.]).unwrap();
    let loads = [Wrench::new([3., 5., -2.], [0.1, -0.2, 0.3]).unwrap(); 4];
    let sample = system.sample(chassis_load, &loads).unwrap();
    let mut force = 300. * vector(sample.acceleration_world_m_s2);
    let q = DQuat::from_array(sample.chassis.orientation_xyzw);
    let omega = vector(sample.chassis.angular_velocity_world_rad_s);
    let mut angular = vector(sample.chassis.position_m).cross(force)
        + inertia(
            &chassis_properties(),
            q,
            vector(sample.angular_acceleration_world_rad_s2),
        )
        + omega.cross(inertia(&chassis_properties(), q, omega));
    let mut expected_force = vector(chassis_load.force_world_n()) + 300. * g;
    let mut expected_torque = vector(sample.chassis.position_m).cross(expected_force)
        + vector(chassis_load.torque_world_nm());
    for (upright, load) in sample.uprights.iter().zip(loads) {
        let a = 35. * vector(upright.acceleration_world_m_s2);
        force += a;
        let q = DQuat::from_array(upright.body.orientation_xyzw);
        let w = vector(upright.body.angular_velocity_world_rad_s);
        angular += vector(upright.body.position_m).cross(a)
            + inertia(
                &upright_properties(),
                q,
                vector(upright.angular_acceleration_world_rad_s2),
            )
            + w.cross(inertia(&upright_properties(), q, w));
        let f = vector(load.force_world_n()) + 35. * g;
        expected_force += f;
        expected_torque +=
            vector(upright.body.position_m).cross(f) + vector(load.torque_world_nm());
        assert!(
            upright
                .constraint_balance_in_source_frame
                .travel_actuator_n
                .abs()
                < 1e-7
        );
    }
    assert!(
        (force - expected_force).length() < 1e-7,
        "{force:?} != {expected_force:?}"
    );
    assert!(
        (angular - expected_torque).length() < 1e-6,
        "{angular:?} != {expected_torque:?}"
    );
}

#[test]
fn kinetic_potential_energy_derivative_matches_external_power_minus_dissipation() {
    let p = DVec3::new(3., 4., 5.);
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let v = DVec3::new(1., -0.2, 0.3);
    let w = DVec3::new(0.2, -0.15, 0.3);
    let gravity = [0.3, -9.81, 0.1];
    let initial = diverse_states();
    let load = Wrench::new([13., -4., 7.], [1., 2., -3.]).unwrap();
    let loads = [Wrench::new([3., 5., -2.], [0.1, -0.2, 0.3]).unwrap(); 4];
    let system = four(body(p, q, v, w), initial, gravity, true);
    let sample = system.sample(load, &loads).unwrap();
    let mut power = vector(load.force_world_n()).dot(v) + vector(load.torque_world_nm()).dot(w)
        - sample.dissipated_power_w;
    for (s, load) in sample.uprights.iter().zip(loads) {
        power += vector(load.force_world_n()).dot(vector(s.body.velocity_world_m_s))
            + vector(load.torque_world_nm()).dot(vector(s.body.angular_velocity_world_rad_s));
    }
    let eps = 1e-4;
    let mut energies = Vec::new();
    for dt in [-eps, eps] {
        let chassis = body(
            p + v * dt,
            DQuat::from_scaled_axis(w * dt) * q,
            v + vector(sample.acceleration_world_m_s2) * dt,
            w + vector(sample.angular_acceleration_world_rad_s2) * dt,
        );
        let states = std::array::from_fn(|i| TravelState {
            travel_m: initial[i].travel_m + dt * initial[i].rate_m_s,
            rate_m_s: initial[i].rate_m_s + dt * sample.uprights[i].travel_acceleration_m_s2,
        });
        energies.push(total_energy(
            &four(chassis, states, gravity, true)
                .sample(load, &loads)
                .unwrap(),
        ));
    }
    near((energies[1] - energies[0]) / (2. * eps), power, 0.002);
}

#[test]
fn coupled_motion_and_reactions_are_covariant_under_world_rotation_translation() {
    let p = DVec3::new(3., 4., 5.);
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let v = DVec3::new(1., -0.2, 0.3);
    let w = DVec3::new(0.2, -0.15, 0.3);
    let g = DVec3::new(0., -9.81, 0.);
    let r = DQuat::from_euler(glam::EulerRot::XYZ, -0.3, 0.1, 0.4);
    let t = DVec3::new(-2., 3., 1.);
    let mut base = four(body(p, q, v, w), diverse_states(), g.to_array(), true);
    let mut moved = four(
        body(t + r * p, r * q, r * v, r * w),
        diverse_states(),
        (r * g).to_array(),
        true,
    );
    for _ in 0..30 {
        let a = base
            .step(0.001, Wrench::default(), &[Wrench::default(); 4])
            .unwrap();
        let b = moved
            .step(0.001, Wrench::default(), &[Wrench::default(); 4])
            .unwrap();
        close(
            b.chassis.position_m,
            t + r * vector(a.chassis.position_m),
            1e-8,
        );
        close(
            b.acceleration_world_m_s2,
            r * vector(a.acceleration_world_m_s2),
            1e-7,
        );
        close(
            b.angular_acceleration_world_rad_s2,
            r * vector(a.angular_acceleration_world_rad_s2),
            1e-7,
        );
        close(
            b.linear_momentum_world_kg_m_s,
            r * vector(a.linear_momentum_world_kg_m_s),
            1e-6,
        );
        close(
            b.angular_momentum_about_world_origin_kg_m2_s,
            r * vector(a.angular_momentum_about_world_origin_kg_m2_s)
                + t.cross(r * vector(a.linear_momentum_world_kg_m_s)),
            1e-5,
        );
        for (first, second) in a.uprights.iter().zip(&b.uprights) {
            near(first.state.travel_m, second.state.travel_m, 1e-9);
            near(first.state.rate_m_s, second.state.rate_m_s, 1e-8);
            close(
                second.support_force_on_chassis_world_n,
                r * vector(first.support_force_on_chassis_world_n),
                1e-6,
            );
            close(
                second.support_torque_about_chassis_com_world_nm,
                r * vector(first.support_torque_about_chassis_com_world_nm),
                1e-6,
            );
        }
    }
}

#[test]
fn midpoint_convergence_and_internal_recoil_hold_for_both_topologies() {
    for strut in [false, true] {
        let mut results = Vec::new();
        for count in [64, 128, 1024] {
            let mut system = one(strut, Some(0.58));
            let mut sample = system
                .sample(Wrench::default(), &[Wrench::default()])
                .unwrap();
            for _ in 0..count {
                sample = system
                    .step(
                        0.2 / f64::from(count),
                        Wrench::default(),
                        &[Wrench::default()],
                    )
                    .unwrap();
            }
            results.push(sample);
        }
        let error = |a: &ChassisSample, b: &ChassisSample| {
            (vector(a.chassis.position_m) - vector(b.chassis.position_m)).length()
                + (a.uprights[0].state.travel_m - b.uprights[0].state.travel_m).abs()
                + (a.uprights[0].state.rate_m_s - b.uprights[0].state.rate_m_s).abs()
        };
        let coarse = error(&results[0], &results[2]);
        let fine = error(&results[1], &results[2]);
        assert!(coarse / fine > 3.7, "{coarse} / {fine}");
        assert!(fine < 1e-5, "{fine}");
    }
}

#[test]
fn bad_counts_loads_matrix_and_failed_last_corner_leave_the_system_unchanged() {
    let base = body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO);
    assert!(ChassisSystem::new(base.clone(), vec![], [0.; 3]).is_err());
    let c = corner(
        false,
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
        None,
    );
    assert!(ChassisSystem::new(base.clone(), vec![c; 5], [0.; 3]).is_err());
    let states = [TravelState {
        travel_m: 0.,
        rate_m_s: 0.,
    }; 4];
    let mut system = four(base, states, [0.; 3], false);
    let before = serde_json::to_string(
        &system
            .sample(Wrench::default(), &[Wrench::default(); 4])
            .unwrap(),
    )
    .unwrap();
    assert!(
        system
            .sample(Wrench::default(), &[Wrench::default(); 3])
            .is_err()
    );
    for dt in [f64::NAN, 0., 0.1] {
        assert!(
            system
                .step(dt, Wrench::default(), &[Wrench::default(); 4])
                .is_err()
        );
    }
    let mut loads = [Wrench::default(); 4];
    loads[3] = Wrench::new([0., 100000., 0.], [0.; 3]).unwrap();
    assert!(system.step(0.02, Wrench::default(), &loads).is_err());
    assert_eq!(
        serde_json::to_string(
            &system
                .sample(Wrench::default(), &[Wrench::default(); 4])
                .unwrap()
        )
        .unwrap(),
        before
    );
    system.step(1e-4, Wrench::default(), &loads).unwrap();
    let tiny = Body::new(
        MassProperties::new(1e-9, [[1e-12, 0., 0.], [0., 1e-12, 0.], [0., 0., 1e-12]]).unwrap(),
        [0.; 3],
        [0., 0., 0., 1.],
        [0.; 3],
        [0.; 3],
    )
    .unwrap();
    let huge = MountedUpright::new(
        geometry(false),
        MassProperties::new(1e12, [[1e12, 0., 0.], [0., 1e12, 0.], [0., 0., 1e12]]).unwrap(),
        [0.; 3],
        [0.; 3],
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
    )
    .unwrap();
    assert!(ChassisSystem::new(tiny, vec![huge], [0.; 3]).is_err());
}

fn rotor(state: WheelState) -> Wheel {
    Wheel::new([0.02, -0.01, 0.03], [1., 0., 0.], 20., 1.2, 0.7, state).unwrap()
}
fn rotor_four(
    chassis: Body,
    states: [TravelState; 4],
    spins: [WheelState; 4],
    gravity: [f64; 3],
) -> ChassisSystem {
    let mounts = [
        [0.8, 0., 1.2],
        [-0.8, 0., 1.2],
        [0.8, 0., -1.2],
        [-0.8, 0., -1.2],
    ];
    let uprights = mounts
        .into_iter()
        .enumerate()
        .map(|(i, p)| {
            let q = if i % 2 == 0 {
                DQuat::IDENTITY
            } else {
                DQuat::from_rotation_y(std::f64::consts::PI)
            };
            let mut u = corner(
                i < 2,
                LinkPose::new(p, q.to_array()).unwrap(),
                states[i],
                Some(0.6),
            );
            u.attach_wheel(rotor(spins[i])).unwrap();
            u
        })
        .collect();
    ChassisSystem::new(chassis, uprights, gravity).unwrap()
}
fn spins() -> [WheelState; 4] {
    [
        WheelState {
            angle_rad: 0.2,
            rate_rad_s: 40.,
        },
        WheelState {
            angle_rad: -0.1,
            rate_rad_s: -30.,
        },
        WheelState {
            angle_rad: 0.5,
            rate_rad_s: 20.,
        },
        WheelState {
            angle_rad: 0.3,
            rate_rad_s: -10.,
        },
    ]
}
fn motor_loads() -> [WheelLoad; 4] {
    [8., -6., 4., -2.].map(|t| WheelLoad::new(Wrench::default(), t).unwrap())
}
fn coaxial_rotor(strut: bool, rate: f64) -> ChassisSystem {
    let chassis = Body::new(
        MassProperties::new(300., [[10., 0., 0.], [0., 12., 0.], [0., 0., 14.]]).unwrap(),
        [0.; 3],
        [0., 0., 0., 1.],
        [0.; 3],
        [0.; 3],
    )
    .unwrap();
    let mut u = MountedUpright::new(
        geometry(strut),
        MassProperties::new(35., [[0.4, 0., 0.], [0., 0.5, 0.], [0., 0., 0.6]]).unwrap(),
        [0.; 3],
        [0.; 3],
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
    )
    .unwrap();
    u.attach_wheel(
        Wheel::new(
            [0.; 3],
            [1., 0., 0.],
            20.,
            1.2,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: rate,
            },
        )
        .unwrap(),
    )
    .unwrap();
    ChassisSystem::new(chassis, vec![u], [0.; 3]).unwrap()
}

#[test]
fn wheel_mass_inertia_state_attachment_and_load_bounds_are_explicit() {
    for (axis, mass, axial, transverse, state) in [
        (
            [0.; 3],
            20.,
            1.2,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: 0.,
            },
        ),
        (
            [1., 0., 0.],
            -1.,
            1.2,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: 0.,
            },
        ),
        (
            [1., 0., 0.],
            20.,
            2.,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: 0.,
            },
        ),
        (
            [1., 0., 0.],
            20.,
            1.2,
            0.7,
            WheelState {
                angle_rad: f64::NAN,
                rate_rad_s: 0.,
            },
        ),
        (
            [1., 0., 0.],
            20.,
            1.2,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: 100001.,
            },
        ),
    ] {
        assert!(Wheel::new([0.; 3], axis, mass, axial, transverse, state).is_err());
    }
    let mut u = corner(
        false,
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
        None,
    );
    u.attach_wheel(rotor(spins()[0])).unwrap();
    assert!(u.attach_wheel(rotor(spins()[1])).is_err());
    assert!(WheelLoad::new(Wrench::default(), f64::INFINITY).is_err());
    assert!(
        one(false, None)
            .sample_with_wheels(Wrench::default(), &[Wrench::default()], &motor_loads()[..1])
            .is_err()
    );
}

#[test]
fn internal_drive_and_external_axle_torque_have_distinct_analytical_reactions() {
    for strut in [false, true] {
        let motor = WheelLoad::new(Wrench::default(), 8.).unwrap();
        let mut system = coaxial_rotor(strut, 0.);
        let sample = system
            .sample_with_wheels(Wrench::default(), &[Wrench::default()], &[motor])
            .unwrap();
        let wheel = sample.uprights[0].wheel.as_ref().unwrap();
        close(
            sample.angular_acceleration_world_rad_s2,
            DVec3::new(-8. / 10.4, 0., 0.),
            1e-9,
        );
        close(
            wheel.angular_acceleration_world_rad_s2,
            DVec3::new(8. / 1.2, 0., 0.),
            1e-9,
        );
        near(wheel.spin_acceleration_rad_s2, 8. / 1.2 + 8. / 10.4, 1e-9);
        close(
            wheel.motor_reaction_on_upright_world_nm,
            DVec3::new(-8., 0., 0.),
            1e-9,
        );
        close(
            wheel.bearing_torque_about_wheel_com_on_upright_world_nm,
            DVec3::ZERO,
            1e-9,
        );
        let external = WheelLoad::new(Wrench::new([0.; 3], [8., 0., 0.]).unwrap(), 0.).unwrap();
        let ext_sample = system
            .sample_with_wheels(Wrench::default(), &[Wrench::default()], &[external])
            .unwrap();
        close(
            ext_sample.angular_acceleration_world_rad_s2,
            DVec3::ZERO,
            1e-9,
        );
        near(
            ext_sample.uprights[0]
                .wheel
                .as_ref()
                .unwrap()
                .spin_acceleration_rad_s2,
            8. / 1.2,
            1e-9,
        );
        let mut last = sample;
        for _ in 0..500 {
            last = system
                .step_with_wheels(0.001, Wrench::default(), &[Wrench::default()], &[motor])
                .unwrap();
        }
        let wheel = last.uprights[0].wheel.as_ref().unwrap();
        let alpha = 8. / 1.2 + 8. / 10.4;
        near(wheel.state.rate_rad_s, alpha * 0.5, 1e-8);
        near(wheel.state.angle_rad, 0.5 * alpha * 0.25, 1e-8);
        near(
            last.chassis.angular_velocity_world_rad_s[0],
            -8. * 0.5 / 10.4,
            1e-8,
        );
        near(last.kinetic_energy_j, 8. * wheel.state.angle_rad, 1e-8);
        close(
            last.angular_momentum_about_world_origin_kg_m2_s,
            DVec3::ZERO,
            1e-8,
        );
        close(last.linear_momentum_world_kg_m_s, DVec3::ZERO, 1e-8);
    }
}

#[test]
fn spinning_wheel_masses_and_internal_motors_obey_global_newton_euler_balance() {
    let g = DVec3::new(0.3, -9.81, 0.1);
    let system = rotor_four(
        body(
            DVec3::new(3., 4., 5.),
            DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1),
            DVec3::new(1., -0.2, 0.3),
            DVec3::new(0.2, -0.15, 0.3),
        ),
        diverse_states(),
        spins(),
        g.to_array(),
    );
    let wheel_external = Wrench::new([2., -3., 1.], [0.3, 0.1, -0.2]).unwrap();
    let motors = [8., -6., 4., -2.].map(|t| WheelLoad::new(wheel_external, t).unwrap());
    let sample = system
        .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motors)
        .unwrap();
    let mut force = 300. * vector(sample.acceleration_world_m_s2);
    let q = DQuat::from_array(sample.chassis.orientation_xyzw);
    let w = vector(sample.chassis.angular_velocity_world_rad_s);
    let mut moment = vector(sample.chassis.position_m).cross(force)
        + inertia(
            &chassis_properties(),
            q,
            vector(sample.angular_acceleration_world_rad_s2),
        )
        + w.cross(inertia(&chassis_properties(), q, w));
    let mut expected_force = 300. * g;
    let mut expected_moment = vector(sample.chassis.position_m).cross(expected_force);
    let wheel_properties =
        MassProperties::new(20., [[1.2, 0., 0.], [0., 0.7, 0.], [0., 0., 0.7]]).unwrap();
    for u in &sample.uprights {
        let wheel = u.wheel.as_ref().unwrap();
        assert!(wheel.spin_balance_residual_nm.abs() < 1e-7);
        for (body, a, alpha, properties) in [
            (
                &u.body,
                u.acceleration_world_m_s2,
                u.angular_acceleration_world_rad_s2,
                upright_properties(),
            ),
            (
                &wheel.body,
                wheel.acceleration_world_m_s2,
                wheel.angular_acceleration_world_rad_s2,
                wheel_properties.clone(),
            ),
        ] {
            let ma = properties.mass_kg() * vector(a);
            force += ma;
            let q = DQuat::from_array(body.orientation_xyzw);
            let w = vector(body.angular_velocity_world_rad_s);
            moment += vector(body.position_m).cross(ma)
                + inertia(&properties, q, vector(alpha))
                + w.cross(inertia(&properties, q, w));
            let gravity = properties.mass_kg() * g;
            expected_force += gravity;
            expected_moment += vector(body.position_m).cross(gravity);
        }
        let f = vector(wheel_external.force_world_n());
        expected_force += f;
        expected_moment +=
            vector(wheel.body.position_m).cross(f) + vector(wheel_external.torque_world_nm());
    }
    assert!((force - expected_force).length() < 1e-7);
    assert!((moment - expected_moment).length() < 1e-6);
    near(sample.total_mass_kg, 520., 1e-9);
}

#[test]
fn wheel_drive_power_and_gyroscopic_transport_match_energy_derivative() {
    let p = DVec3::new(3., 4., 5.);
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let v = DVec3::new(1., -0.2, 0.3);
    let w = DVec3::new(0.2, -0.15, 0.3);
    let g = [0.3, -9.81, 0.1];
    let states = diverse_states();
    let spin = spins();
    let wheel_external = Wrench::new([2., -3., 1.], [0.3, 0.1, -0.2]).unwrap();
    let motors = [8., -6., 4., -2.].map(|t| WheelLoad::new(wheel_external, t).unwrap());
    let system = rotor_four(body(p, q, v, w), states, spin, g);
    let sample = system
        .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motors)
        .unwrap();
    let mut expected = [8., -6., 4., -2.]
        .into_iter()
        .zip(spin)
        .map(|(t, s)| t * s.rate_rad_s)
        .sum::<f64>()
        - sample.dissipated_power_w;
    for u in &sample.uprights {
        let wheel = u.wheel.as_ref().unwrap();
        expected += vector(wheel_external.force_world_n())
            .dot(vector(wheel.body.velocity_world_m_s))
            + vector(wheel_external.torque_world_nm())
                .dot(vector(wheel.body.angular_velocity_world_rad_s));
    }
    let eps = 1e-5;
    let mut energies = Vec::new();
    for dt in [-eps, eps] {
        let chassis = body(
            p + dt * v,
            DQuat::from_scaled_axis(dt * w) * q,
            v + dt * vector(sample.acceleration_world_m_s2),
            w + dt * vector(sample.angular_acceleration_world_rad_s2),
        );
        let hs = std::array::from_fn(|i| TravelState {
            travel_m: states[i].travel_m + dt * states[i].rate_m_s,
            rate_m_s: states[i].rate_m_s + dt * sample.uprights[i].travel_acceleration_m_s2,
        });
        let ss = std::array::from_fn(|i| WheelState {
            angle_rad: spin[i].angle_rad + dt * spin[i].rate_rad_s,
            rate_rad_s: spin[i].rate_rad_s
                + dt * sample.uprights[i]
                    .wheel
                    .as_ref()
                    .unwrap()
                    .spin_acceleration_rad_s2,
        });
        energies.push(total_energy(
            &rotor_four(chassis, hs, ss, g)
                .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motors)
                .unwrap(),
        ));
    }
    near((energies[1] - energies[0]) / (2. * eps), expected, 0.02);
}

#[test]
fn wheel_axes_spin_reactions_and_world_momenta_transform_covariantly() {
    let p = DVec3::new(3., 4., 5.);
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let v = DVec3::new(1., -0.2, 0.3);
    let w = DVec3::new(0.2, -0.15, 0.3);
    let g = DVec3::new(0., -9.81, 0.);
    let r = DQuat::from_euler(glam::EulerRot::XYZ, -0.3, 0.1, 0.4);
    let t = DVec3::new(-2., 3., 1.);
    let mut base = rotor_four(body(p, q, v, w), diverse_states(), spins(), g.to_array());
    let mut moved = rotor_four(
        body(t + r * p, r * q, r * v, r * w),
        diverse_states(),
        spins(),
        (r * g).to_array(),
    );
    for _ in 0..20 {
        let a = base
            .step_with_wheels(
                0.001,
                Wrench::default(),
                &[Wrench::default(); 4],
                &motor_loads(),
            )
            .unwrap();
        let b = moved
            .step_with_wheels(
                0.001,
                Wrench::default(),
                &[Wrench::default(); 4],
                &motor_loads(),
            )
            .unwrap();
        close(
            b.chassis.position_m,
            t + r * vector(a.chassis.position_m),
            1e-8,
        );
        close(
            b.linear_momentum_world_kg_m_s,
            r * vector(a.linear_momentum_world_kg_m_s),
            1e-6,
        );
        close(
            b.angular_momentum_about_world_origin_kg_m2_s,
            r * vector(a.angular_momentum_about_world_origin_kg_m2_s)
                + t.cross(r * vector(a.linear_momentum_world_kg_m_s)),
            1e-5,
        );
        for (first, second) in a.uprights.iter().zip(&b.uprights) {
            let x = first.wheel.as_ref().unwrap();
            let y = second.wheel.as_ref().unwrap();
            near(x.state.rate_rad_s, y.state.rate_rad_s, 1e-7);
            near(x.spin_acceleration_rad_s2, y.spin_acceleration_rad_s2, 1e-6);
            close(y.axis_world, r * vector(x.axis_world), 1e-8);
            close(
                y.force_on_upright_world_n,
                r * vector(x.force_on_upright_world_n),
                1e-6,
            );
            close(
                y.torque_about_upright_origin_world_nm,
                r * vector(x.torque_about_upright_origin_world_nm),
                1e-6,
            );
        }
    }
}

#[test]
fn rotating_wheel_midpoint_trajectory_converges_and_failed_final_spin_rolls_back() {
    let mut results = Vec::new();
    for count in [40, 80, 320] {
        let mut system = rotor_four(
            body(
                DVec3::ZERO,
                DQuat::IDENTITY,
                DVec3::ZERO,
                DVec3::new(0.2, -0.15, 0.3),
            ),
            diverse_states(),
            spins(),
            [0.; 3],
        );
        let mut sample = system
            .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motor_loads())
            .unwrap();
        for _ in 0..count {
            sample = system
                .step_with_wheels(
                    0.04 / f64::from(count),
                    Wrench::default(),
                    &[Wrench::default(); 4],
                    &motor_loads(),
                )
                .unwrap();
        }
        results.push(sample);
    }
    let error = |a: &ChassisSample, b: &ChassisSample| {
        a.uprights
            .iter()
            .zip(&b.uprights)
            .map(|(a, b)| {
                (a.state.travel_m - b.state.travel_m).abs()
                    + (a.wheel.as_ref().unwrap().state.rate_rad_s
                        - b.wheel.as_ref().unwrap().state.rate_rad_s)
                        .abs()
            })
            .sum::<f64>()
    };
    let coarse = error(&results[0], &results[2]);
    let fine = error(&results[1], &results[2]);
    assert!(coarse / fine > 3.7, "{coarse} / {fine}");
    let mut system = coaxial_rotor(false, 99999.);
    let before = serde_json::to_string(
        &system
            .sample(Wrench::default(), &[Wrench::default()])
            .unwrap(),
    )
    .unwrap();
    let loads = [WheelLoad::new(Wrench::default(), 72.).unwrap()];
    assert!(
        system
            .step_with_wheels(0.02, Wrench::default(), &[Wrench::default()], &loads)
            .is_err()
    );
    assert!(
        system
            .sample_with_wheels(Wrench::default(), &[Wrench::default()], &[])
            .is_err()
    );
    assert_eq!(
        serde_json::to_string(
            &system
                .sample(Wrench::default(), &[Wrench::default()])
                .unwrap()
        )
        .unwrap(),
        before
    );
    system
        .step_with_wheels(1e-4, Wrench::default(), &[Wrench::default()], &loads)
        .unwrap();
}

#[test]
fn tilted_axle_and_spin_phase_preserve_axisymmetric_inertia_and_motor_recoil() {
    for axis in [DVec3::X, DVec3::new(1., 2., -1.).normalize(), DVec3::Z] {
        for angle in [0., 1.234, 1e9] {
            let chassis = Body::new(
                MassProperties::new(300., [[10., 0., 0.], [0., 10., 0.], [0., 0., 10.]]).unwrap(),
                [0.; 3],
                [0., 0., 0., 1.],
                [0.; 3],
                [0.; 3],
            )
            .unwrap();
            let mut u = MountedUpright::new(
                geometry(false),
                MassProperties::new(35., [[0.4, 0., 0.], [0., 0.4, 0.], [0., 0., 0.4]]).unwrap(),
                [0.; 3],
                [0.; 3],
                LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
                TravelState {
                    travel_m: 0.,
                    rate_m_s: 0.,
                },
            )
            .unwrap();
            u.attach_wheel(
                Wheel::new(
                    [0.; 3],
                    axis.to_array(),
                    20.,
                    1.2,
                    0.7,
                    WheelState {
                        angle_rad: angle,
                        rate_rad_s: 3.,
                    },
                )
                .unwrap(),
            )
            .unwrap();
            let system = ChassisSystem::new(chassis, vec![u], [0.; 3]).unwrap();
            let sample = system
                .sample_with_wheels(
                    Wrench::default(),
                    &[Wrench::default()],
                    &[WheelLoad::new(Wrench::default(), 8.).unwrap()],
                )
                .unwrap();
            let wheel = sample.uprights[0].wheel.as_ref().unwrap();
            close(
                sample.angular_acceleration_world_rad_s2,
                -axis * (8. / 10.4),
                1e-9,
            );
            close(
                wheel.angular_acceleration_world_rad_s2,
                axis * (8. / 1.2),
                1e-9,
            );
            near(sample.kinetic_energy_j, 0.5 * 1.2 * 9., 1e-9);
            close(
                sample.angular_momentum_about_world_origin_kg_m2_s,
                axis * (1.2 * 3.),
                1e-9,
            );
        }
    }
}
