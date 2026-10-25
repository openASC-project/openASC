use asc_physics::{
    Body, ChassisKinematics, ChassisLoads, ChassisSample, ChassisSystem, Damper, DamperBranch,
    FiveLink, LinkPose, LinkSpring, MassProperties, MountedUpright, NormalContact, StrutLink,
    SupportPlane, TravelMechanism, TravelState, Wheel, WheelLoad, WheelState, Wrench,
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
fn contact_fixture(q: DQuat, shift: DVec3) -> ChassisSystem {
    let chassis = Body::new(
        MassProperties::new(300., [[220., 0., 0.], [0., 400., 0.], [0., 0., 350.]]).unwrap(),
        shift.to_array(),
        q.to_array(),
        (q * DVec3::new(-2., 2., 0.)).to_array(),
        [0.; 3],
    )
    .unwrap();
    let mut u = corner(
        false,
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
        None,
    );
    u.attach_wheel(
        Wheel::new(
            [0.; 3],
            [1., 0., 0.],
            20.,
            1.2,
            0.7,
            WheelState {
                angle_rad: 0.,
                rate_rad_s: 0.,
            },
        )
        .unwrap(),
    )
    .unwrap();
    ChassisSystem::new(chassis, vec![u], [0.; 3]).unwrap()
}
fn wall_contacts(mu: f64, q: DQuat, shift: DVec3) -> Vec<asc_physics::ChassisContact> {
    [-1., 1.]
        .into_iter()
        .map(|y| asc_physics::ChassisContact {
            point_world_m: (shift + q * DVec3::new(0., y, 0.)).to_array(),
            normal_world: (q * DVec3::X).to_array(),
            friction: mu,
        })
        .collect()
}
#[test]
fn simultaneous_chassis_contacts_match_analytic_sticking_and_sliding_with_free_suspension_mass() {
    for mu in [0., 0.2, 0.9] {
        let mut system = contact_fixture(DQuat::IDENTITY, DVec3::ZERO);
        let snapshot = system.kinematics().unwrap();
        let result = system
            .resolve_chassis_contacts(
                &wall_contacts(mu, DQuat::IDENTITY, DVec3::ZERO),
                Default::default(),
            )
            .unwrap();
        let after = system.kinematics().unwrap();
        near(after.chassis.velocity_world_m_s[0], 0., 1e-6);
        let tangential_impulse = (mu * 710.).min(600.);
        near(
            after.chassis.velocity_world_m_s[1],
            2. - tangential_impulse / 300.,
            1e-6,
        );
        near(result.net_impulse_world_ns[0], 710., 1e-5);
        near(result.net_impulse_world_ns[1], -tangential_impulse, 1e-5);
        close(
            after.chassis.angular_velocity_world_rad_s,
            DVec3::ZERO,
            1e-6,
        );
        assert_eq!(snapshot.chassis.position_m, after.chassis.position_m);
        assert_eq!(
            snapshot.chassis.orientation_xyzw,
            after.chassis.orientation_xyzw
        );
        near(result.work.energy_balance_residual_j, 0., 1e-7);
        assert!(result.work.external_impulse_work_j < 0.);
        for c in &result.contacts {
            let impulse = vector(c.tangent_impulse_world_ns);
            assert!(impulse.length() <= mu * c.normal_impulse_ns + 1e-7);
            assert!(
                vector(c.point_velocity_after_world_m_s).dot(vector(c.contact.normal_world))
                    >= -1e-7
            );
        }
        near(after.uprights[0].body.velocity_world_m_s[1], 2., 1e-6);
    }
}
#[test]
fn coupled_contact_friction_is_covariant_and_redundant_constraints_do_not_create_spin_or_energy() {
    let q = DQuat::from_rotation_y(0.7) * DQuat::from_rotation_z(-0.4);
    let shift = DVec3::new(10., 3., -4.);
    let mut reference = contact_fixture(DQuat::IDENTITY, DVec3::ZERO);
    let base = reference
        .resolve_chassis_contacts(
            &wall_contacts(0.2, DQuat::IDENTITY, DVec3::ZERO),
            Default::default(),
        )
        .unwrap();
    let mut rotated = contact_fixture(q, shift);
    let mut contacts = wall_contacts(0.2, q, shift);
    let original = contacts.clone();
    for _ in 0..31 {
        contacts.extend(original.clone());
    }
    contacts.reverse();
    let result = rotated
        .resolve_chassis_contacts(&contacts, Default::default())
        .unwrap();
    close(
        result.net_impulse_world_ns,
        q * vector(base.net_impulse_world_ns),
        1e-5,
    );
    let a = rotated.kinematics().unwrap();
    let b = reference.kinematics().unwrap();
    close(
        a.chassis.velocity_world_m_s,
        q * vector(b.chassis.velocity_world_m_s),
        1e-6,
    );
    close(
        a.chassis.angular_velocity_world_rad_s,
        q * vector(b.chassis.angular_velocity_world_rad_s),
        1e-6,
    );
    near(
        result.work.kinetic_energy_after_j,
        base.work.kinetic_energy_after_j,
        1e-5,
    );
    assert!(result.max_projected_velocity_residual_m_s <= 1e-7);
}
#[test]
fn oblique_sliding_friction_minimizes_tangent_energy_in_anisotropic_point_mobility() {
    let mut initial = contact_fixture(DQuat::IDENTITY, DVec3::ZERO);
    initial
        .impulse_at_world_point([0., 0., 355.], [0.; 3])
        .unwrap();
    let c = asc_physics::ChassisContact {
        point_world_m: [0., 1., 0.],
        normal_world: [1., 0., 0.],
        friction: 0.2,
    };
    let mut system = initial.clone();
    let result = system
        .resolve_chassis_contacts(&[c], Default::default())
        .unwrap();
    let contact = &result.contacts[0];
    let tangent = vector(contact.tangent_impulse_world_ns);
    let radius = c.friction * contact.normal_impulse_ns;
    near(tangent.length(), radius, 1e-8);
    let final_speed = vector(contact.point_velocity_after_world_m_s);
    let slip = DVec3::new(0., final_speed.y, final_speed.z);
    assert!(slip.length() > 0.1);
    close(
        (tangent / tangent.length()).to_array(),
        -slip.normalize(),
        1e-7,
    );
    for angle in [-0.2, -0.05, 0.05, 0.2] {
        let trial_tangent = DQuat::from_rotation_x(angle) * tangent;
        let mut alternative = initial.clone();
        let trial = alternative
            .impulse_at_world_point(
                (DVec3::X * contact.normal_impulse_ns + trial_tangent).to_array(),
                c.point_world_m,
            )
            .unwrap();
        assert!(trial.kinetic_energy_after_j >= result.work.kinetic_energy_after_j - 1e-8);
    }
}
#[test]
fn multi_contact_momentum_energy_and_moving_rack_work_balance_are_transactional() {
    for moving in [false, true] {
        let mut system = impulse_fixture(moving);
        let before = system
            .sample(Wrench::default(), &[Wrench::default(); 4])
            .unwrap();
        let state = system.kinematics().unwrap();
        let racks = system.rack_motions();
        let points = [
            DVec3::X,
            -DVec3::X,
            DVec3::Y,
            -DVec3::Y,
            DVec3::Z,
            -DVec3::Z,
        ]
        .map(|n| asc_physics::ChassisContact {
            point_world_m: state.chassis.position_m,
            normal_world: n.to_array(),
            friction: 0.,
        });
        let result = system
            .resolve_chassis_contacts(&points, Default::default())
            .unwrap();
        let after = system
            .sample(Wrench::default(), &[Wrench::default(); 4])
            .unwrap();
        close(after.chassis.velocity_world_m_s, DVec3::ZERO, 1e-7);
        close(
            after.linear_momentum_world_kg_m_s,
            vector(before.linear_momentum_world_kg_m_s) + vector(result.net_impulse_world_ns),
            1e-7,
        );
        close(
            after.angular_momentum_about_world_origin_kg_m2_s,
            vector(before.angular_momentum_about_world_origin_kg_m2_s)
                + vector(state.chassis.position_m).cross(vector(result.net_impulse_world_ns))
                + vector(result.net_moment_about_chassis_com_nms),
            1e-6,
        );
        near(result.work.energy_balance_residual_j, 0., 1e-7);
        assert!(result.work.external_impulse_work_j <= 1e-7);
        if moving {
            assert!(result.work.prescribed_rack_impulse_work_j.abs() > 1e-5);
        }
        assert_eq!(system.rack_motions(), racks);
        let final_state = system.kinematics().unwrap();
        assert_eq!(
            state.chassis.orientation_xyzw,
            final_state.chassis.orientation_xyzw
        );
        for (a, b) in state.uprights.iter().zip(final_state.uprights) {
            assert_eq!(a.state.travel_m, b.state.travel_m);
            assert_eq!(
                a.wheel.as_ref().unwrap().state.angle_rad,
                b.wheel.unwrap().state.angle_rad
            );
        }
    }
}
#[test]
fn separating_and_empty_manifolds_are_exact_noops_and_invalid_or_unconverged_solves_preserve_state()
{
    let mut system = contact_fixture(DQuat::IDENTITY, DVec3::ZERO);
    let snapshot = serde_json::to_value(system.kinematics().unwrap()).unwrap();
    system
        .resolve_chassis_contacts(&[], Default::default())
        .unwrap();
    let separating = [asc_physics::ChassisContact {
        point_world_m: [0.; 3],
        normal_world: [-1., 0., 0.],
        friction: 0.5,
    }];
    let result = system
        .resolve_chassis_contacts(&separating, Default::default())
        .unwrap();
    assert_eq!(result.net_impulse_world_ns, [0.; 3]);
    assert_eq!(
        serde_json::to_value(system.kinematics().unwrap()).unwrap(),
        snapshot
    );
    let contacts = wall_contacts(0.2, DQuat::IDENTITY, DVec3::ZERO);
    for settings in [
        asc_physics::ChassisContactSettings {
            max_iterations: 0,
            velocity_tolerance_m_s: 1e-7,
        },
        asc_physics::ChassisContactSettings {
            max_iterations: 1,
            velocity_tolerance_m_s: 1e-9,
        },
    ] {
        assert!(
            system
                .resolve_chassis_contacts(&contacts, settings)
                .is_err()
        );
        assert_eq!(
            serde_json::to_value(system.kinematics().unwrap()).unwrap(),
            snapshot
        );
    }
    for c in [
        asc_physics::ChassisContact {
            normal_world: [0.; 3],
            ..contacts[0]
        },
        asc_physics::ChassisContact {
            friction: 2.1,
            ..contacts[0]
        },
        asc_physics::ChassisContact {
            point_world_m: [f64::NAN, 0., 0.],
            ..contacts[0]
        },
    ] {
        assert!(
            system
                .resolve_chassis_contacts(&[c], Default::default())
                .is_err()
        );
        assert_eq!(
            serde_json::to_value(system.kinematics().unwrap()).unwrap(),
            snapshot
        );
    }
    assert!(
        system
            .resolve_chassis_contacts(&vec![contacts[0]; 65], Default::default())
            .is_err()
    );
}

#[test]
fn chassis_impulse_distinguishes_free_vertical_travel_from_carried_horizontal_mass() {
    for strut in [false, true] {
        for wheel in [false, true] {
            let make = || {
                let mut u = corner(
                    strut,
                    LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
                    TravelState {
                        travel_m: 0.,
                        rate_m_s: 0.,
                    },
                    None,
                );
                if wheel {
                    u.attach_wheel(
                        Wheel::new(
                            [0.; 3],
                            [1., 0., 0.],
                            20.,
                            1.2,
                            0.7,
                            WheelState {
                                angle_rad: 0.,
                                rate_rad_s: 0.,
                            },
                        )
                        .unwrap(),
                    )
                    .unwrap();
                }
                ChassisSystem::new(
                    body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
                    vec![u],
                    [0.; 3],
                )
                .unwrap()
            };
            let mut vertical = make();
            let response = vertical
                .chassis_point_response([0.; 3], [0., 1., 0.])
                .unwrap();
            near(response.inverse_effective_mass_kg_inv, 1. / 300., 1e-12);
            let result = vertical
                .impulse_at_world_point([0., 30., 0.], [0.; 3])
                .unwrap();
            let k = vertical.kinematics().unwrap();
            close(k.chassis.velocity_world_m_s, DVec3::Y * 0.1, 1e-10);
            close(k.uprights[0].body.velocity_world_m_s, DVec3::ZERO, 1e-10);
            near(k.uprights[0].state.rate_m_s, -0.1, 1e-10);
            near(result.kinetic_energy_after_j, 1.5, 1e-10);
            near(result.prescribed_rack_impulse_work_j, 0., 1e-12);
            near(result.energy_balance_residual_j, 0., 1e-10);
            let mut horizontal = make();
            let mass = 335. + if wheel { 20. } else { 0. };
            near(
                horizontal
                    .chassis_point_response([0.; 3], [1., 0., 0.])
                    .unwrap()
                    .inverse_effective_mass_kg_inv,
                1. / mass,
                1e-12,
            );
            horizontal
                .impulse_at_world_point([30., 0., 0.], [0.; 3])
                .unwrap();
            let k = horizontal.kinematics().unwrap();
            close(k.chassis.velocity_world_m_s, DVec3::X * (30. / mass), 1e-10);
            close(
                k.uprights[0].body.velocity_world_m_s,
                DVec3::X * (30. / mass),
                1e-10,
            );
            if let Some(w) = &k.uprights[0].wheel {
                close(w.body.velocity_world_m_s, DVec3::X * (30. / mass), 1e-10);
            }
        }
    }
}
fn impulse_fixture(moving_racks: bool) -> ChassisSystem {
    rotor_four_racks(
        body(
            DVec3::new(2., 3., 4.),
            DQuat::from_rotation_y(0.4) * DQuat::from_rotation_x(0.1),
            DVec3::new(1., -0.2, 0.3),
            DVec3::new(0.1, -0.04, 0.03),
        ),
        [TravelState {
            travel_m: 0.01,
            rate_m_s: 0.02,
        }; 4],
        [WheelState {
            angle_rad: 0.4,
            rate_rad_s: 20.,
        }; 4],
        [0.; 3],
        [asc_physics::RackMotion::new(
            [0.0002, 0., 0.0001],
            if moving_racks {
                [0.03, -0.02, 0.01]
            } else {
                [0.; 3]
            },
            [0.; 3],
        )
        .unwrap(); 4],
    )
}
#[test]
fn coupled_point_impulse_preserves_pose_and_changes_global_momenta_by_external_impulse_only() {
    for moving in [false, true] {
        let mut system = impulse_fixture(moving);
        let zero = [Wrench::default(); 4];
        let before = system.sample(Wrench::default(), &zero).unwrap();
        let initial = system.kinematics().unwrap();
        let racks = system.rack_motions();
        let point = DVec3::new(2.5, 3.2, 4.7);
        let impulse = DVec3::new(4., 8., -3.);
        let response = system
            .chassis_point_response(point.to_array(), impulse.normalize().to_array())
            .unwrap();
        assert_eq!(response.dof_count, 14);
        let result = system
            .impulse_at_world_point(impulse.to_array(), point.to_array())
            .unwrap();
        let after = system.sample(Wrench::default(), &zero).unwrap();
        close(
            after.linear_momentum_world_kg_m_s,
            vector(before.linear_momentum_world_kg_m_s) + impulse,
            2e-8,
        );
        close(
            after.angular_momentum_about_world_origin_kg_m2_s,
            vector(before.angular_momentum_about_world_origin_kg_m2_s) + point.cross(impulse),
            2e-7,
        );
        let final_state = system.kinematics().unwrap();
        assert_eq!(initial.chassis.position_m, final_state.chassis.position_m);
        assert_eq!(
            initial.chassis.orientation_xyzw,
            final_state.chassis.orientation_xyzw
        );
        assert_eq!(racks, system.rack_motions());
        for (b, a) in initial.uprights.iter().zip(&final_state.uprights) {
            assert_eq!(b.state.travel_m, a.state.travel_m);
            assert_eq!(b.source_position_m, a.source_position_m);
            assert_eq!(b.source_orientation_xyzw, a.source_orientation_xyzw);
            assert_eq!(
                b.wheel.as_ref().unwrap().state.angle_rad,
                a.wheel.as_ref().unwrap().state.angle_rad
            );
        }
        let dv = vector(result.point_velocity_after_world_m_s)
            - vector(result.point_velocity_before_world_m_s);
        close(
            dv.to_array(),
            vector(response.point_velocity_delta_world_per_ns) * impulse.length(),
            1e-10,
        );
        near(result.energy_balance_residual_j, 0., 1e-8);
        if moving {
            assert!(result.prescribed_rack_impulse_work_j.abs() > 1e-5);
        } else {
            near(result.prescribed_rack_impulse_work_j, 0., 1e-12);
        }
    }
}
#[test]
fn chassis_normal_restitution_impulse_is_passive_with_fixed_racks() {
    for restitution in [0., 0.3, 1.] {
        let mut system = impulse_fixture(false);
        let point = [2.5, 3.2, 4.7];
        let normal = -DVec3::X;
        let k = system.kinematics().unwrap();
        let velocity = vector(k.chassis.velocity_world_m_s)
            + vector(k.chassis.angular_velocity_world_rad_s)
                .cross(vector(point) - vector(k.chassis.position_m));
        let speed = normal.dot(velocity);
        assert!(speed < 0.);
        let response = system
            .chassis_point_response(point, normal.to_array())
            .unwrap();
        let magnitude = -(1. + restitution) * speed / response.inverse_effective_mass_kg_inv;
        let result = system
            .impulse_at_world_point((normal * magnitude).to_array(), point)
            .unwrap();
        near(
            normal.dot(vector(result.point_velocity_after_world_m_s)),
            -restitution * speed,
            1e-10,
        );
        let expected_loss = 0.5 * (1. - restitution * restitution) * speed * speed
            / response.inverse_effective_mass_kg_inv;
        near(
            result.kinetic_energy_before_j - result.kinetic_energy_after_j,
            expected_loss,
            1e-8,
        );
        assert!(result.external_impulse_work_j <= 1e-8);
    }
}
#[test]
fn impulsive_mass_operator_matches_instantaneous_force_response_without_bias_terms() {
    let mut system = impulse_fixture(true);
    let before = system.kinematics().unwrap();
    // Subtract the force-free acceleration so springs and geometry bias do not enter the impulse.
    let force = DVec3::new(3., 7., -2.);
    let point = DVec3::new(0.4, 0.2, 0.7);
    let baseline = system
        .sample(Wrench::default(), &[Wrench::default(); 4])
        .unwrap();
    let loaded = system
        .sample(
            Wrench::new(
                force.to_array(),
                (point - vector(before.chassis.position_m))
                    .cross(force)
                    .to_array(),
            )
            .unwrap(),
            &[Wrench::default(); 4],
        )
        .unwrap();
    system
        .impulse_at_world_point(force.to_array(), point.to_array())
        .unwrap();
    let after = system.kinematics().unwrap();
    close(
        (vector(after.chassis.velocity_world_m_s) - vector(before.chassis.velocity_world_m_s))
            .to_array(),
        vector(loaded.acceleration_world_m_s2) - vector(baseline.acceleration_world_m_s2),
        1e-9,
    );
    close(
        (vector(after.chassis.angular_velocity_world_rad_s)
            - vector(before.chassis.angular_velocity_world_rad_s))
        .to_array(),
        vector(loaded.angular_acceleration_world_rad_s2)
            - vector(baseline.angular_acceleration_world_rad_s2),
        1e-9,
    );
    for (i, u) in after.uprights.iter().enumerate() {
        near(
            u.state.rate_m_s - before.uprights[i].state.rate_m_s,
            loaded.uprights[i].travel_acceleration_m_s2
                - baseline.uprights[i].travel_acceleration_m_s2,
            1e-9,
        );
        near(
            u.wheel.as_ref().unwrap().state.rate_rad_s
                - before.uprights[i].wheel.as_ref().unwrap().state.rate_rad_s,
            loaded.uprights[i]
                .wheel
                .as_ref()
                .unwrap()
                .spin_acceleration_rad_s2
                - baseline.uprights[i]
                    .wheel
                    .as_ref()
                    .unwrap()
                    .spin_acceleration_rad_s2,
            1e-9,
        );
    }
}
#[test]
fn malformed_impulses_response_directions_and_post_impulse_bounds_leave_state_unchanged() {
    let mut system = impulse_fixture(true);
    let snapshot = serde_json::to_value(system.kinematics().unwrap()).unwrap();
    let racks = system.rack_motions();
    system
        .impulse_at_world_point([0.; 3], [2., 3., 4.])
        .unwrap();
    assert_eq!(
        serde_json::to_value(system.kinematics().unwrap()).unwrap(),
        snapshot
    );
    for (impulse, point) in [
        ([f64::NAN, 0., 0.], [0.; 3]),
        ([f64::INFINITY, 0., 0.], [0.; 3]),
        ([1e19, 0., 0.], [0.; 3]),
        ([0.; 3], [1e8, 0., 0.]),
        ([1e18, 0., 0.], [2., 3., 4.]),
    ] {
        assert!(system.impulse_at_world_point(impulse, point).is_err());
        assert_eq!(
            serde_json::to_value(system.kinematics().unwrap()).unwrap(),
            snapshot
        );
        assert_eq!(system.rack_motions(), racks);
    }
    for direction in [[0.; 3], [2., 0., 0.], [f64::NAN, 0., 0.]] {
        assert!(
            system
                .chassis_point_response([2., 3., 4.], direction)
                .is_err()
        );
    }
    assert_eq!(
        serde_json::to_value(system.kinematics().unwrap()).unwrap(),
        snapshot
    );
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
    rotor_four_racks(
        chassis,
        states,
        spins,
        gravity,
        [asc_physics::RackMotion::fixed([0.; 3]).unwrap(); 4],
    )
}
fn rotor_four_racks(
    chassis: Body,
    states: [TravelState; 4],
    spins: [WheelState; 4],
    gravity: [f64; 3],
    racks: [asc_physics::RackMotion; 4],
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
            u.set_rack_motion(racks[i]).unwrap();
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

#[test]
fn force_independent_snapshots_match_body_states_and_constant_callback_preserves_steps() {
    let mut base = rotor_four(
        body(
            DVec3::new(1., 2., 3.),
            DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1),
            DVec3::new(1., -0.2, 0.3),
            DVec3::new(0.2, -0.15, 0.3),
        ),
        diverse_states(),
        spins(),
        [0., -9.81, 0.],
    );
    let k = base.kinematics().unwrap();
    let sample = base
        .sample(Wrench::default(), &[Wrench::default(); 4])
        .unwrap();
    assert_eq!(k.chassis, sample.chassis);
    for (a, b) in k.uprights.iter().zip(&sample.uprights) {
        close(a.body.position_m, vector(b.body.position_m), 1e-12);
        close(
            a.body.velocity_world_m_s,
            vector(b.body.velocity_world_m_s),
            1e-12,
        );
        let a = a.wheel.as_ref().unwrap();
        let b = b.wheel.as_ref().unwrap();
        close(a.body.position_m, vector(b.body.position_m), 1e-12);
        close(
            a.body.velocity_world_m_s,
            vector(b.body.velocity_world_m_s),
            1e-12,
        );
        close(
            a.body.angular_velocity_world_rad_s,
            vector(b.body.angular_velocity_world_rad_s),
            1e-12,
        );
    }
    let mut callback = base.clone();
    let loads = motor_loads();
    let direct = base
        .step_with_wheels(0.001, Wrench::default(), &[Wrench::default(); 4], &loads)
        .unwrap();
    let via_callback = callback
        .step_with_forces(0.001, |_, k| {
            let mut result = ChassisLoads::zero(k.uprights.len())?;
            result.wheels = loads.to_vec();
            Ok(result)
        })
        .unwrap();
    assert_eq!(
        serde_json::to_string(&direct).unwrap(),
        serde_json::to_string(&via_callback).unwrap()
    );
}

#[test]
fn force_callbacks_use_actual_stage_times_and_fail_transactionally_at_each_stage() {
    let states = [TravelState {
        travel_m: 0.,
        rate_m_s: 0.,
    }; 4];
    let make = || {
        four(
            body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
            states,
            [0.; 3],
            false,
        )
    };
    let mut system = make();
    let mut offsets = Vec::new();
    let dt = 0.01;
    let sample = system
        .step_with_forces(dt, |offset, k| {
            offsets.push(offset);
            let a = 2. + 3. * offset;
            let mut loads = ChassisLoads::zero(k.uprights.len())?;
            loads.chassis = Wrench::new([0., 300. * a, 0.], [0.; 3])?;
            loads.uprights = vec![Wrench::new([0., 35. * a, 0.], [0.; 3])?; 4];
            Ok(loads)
        })
        .unwrap();
    assert_eq!(offsets, vec![0., dt * 0.5, dt]);
    near(
        sample.chassis.velocity_world_m_s[1],
        2. * dt + 1.5 * dt * dt,
        1e-10,
    );
    near(sample.chassis.position_m[1], dt * dt, 1e-10);
    near(sample.acceleration_world_m_s2[1], 2. + 3. * dt, 1e-9);
    for failed in [0, 1, 2] {
        let mut system = make();
        let before = serde_json::to_string(
            &system
                .sample(Wrench::default(), &[Wrench::default(); 4])
                .unwrap(),
        )
        .unwrap();
        let mut stage = 0;
        let result = system.step_with_forces(dt, |_, k| {
            let current = stage;
            stage += 1;
            if current == failed {
                return Err(asc_physics::Error::new(
                    "test_force_stage",
                    "Explicit callback failure",
                ));
            }
            ChassisLoads::zero(k.uprights.len())
        });
        assert!(result.is_err());
        assert_eq!(stage, failed + 1);
        assert_eq!(
            serde_json::to_string(
                &system
                    .sample(Wrench::default(), &[Wrench::default(); 4])
                    .unwrap()
            )
            .unwrap(),
            before
        );
    }
    assert!(
        make()
            .sample_with_forces(|_, _| ChassisLoads::zero(1))
            .is_err()
    );
    let chassis = body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO);
    let mut upright = corner(
        false,
        LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap(),
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
        None,
    );
    let b = DamperBranch::new(0., 0., 0.).unwrap();
    upright
        .add_spring(LinkSpring::new([0., 0.6, 0.], [0.; 3], 0.7, 1e9, Damper::new(b, b)).unwrap())
        .unwrap();
    assert!(ChassisSystem::new(chassis.clone(), vec![upright.clone()], [0.; 3]).is_err());
    let tension = 1e9 * (0.6 - 0.7);
    let applied = |_: f64, _: &ChassisKinematics| -> asc_physics::Result<ChassisLoads> {
        let mut loads = ChassisLoads::zero(1)?;
        loads.chassis = Wrench::new([0., tension, 0.], [0.; 3])?;
        loads.uprights[0] = Wrench::new([0., -tension, 0.], [0.; 3])?;
        Ok(loads)
    };
    let balanced =
        ChassisSystem::new_with_forces(chassis, vec![upright], [0.; 3], applied).unwrap();
    let sample = balanced.sample_with_forces(applied).unwrap();
    close(sample.acceleration_world_m_s2, DVec3::ZERO, 1e-7);
    near(sample.uprights[0].travel_acceleration_m_s2, 0., 1e-7);
}

fn normal_support_system(height_offset: f64) -> (ChassisSystem, f64) {
    let equilibrium = 0.3 - (520. * 9.81 / 4.) / 200000.;
    let preload = (300. * 9.81 / 4.) / 20000.;
    let mounts = [
        [0.8, 0., 1.2],
        [-0.8, 0., 1.2],
        [0.8, 0., -1.2],
        [-0.8, 0., -1.2],
    ];
    let b = DamperBranch::new(1000., 1000., 0.).unwrap();
    let uprights = mounts
        .into_iter()
        .enumerate()
        .map(|(i, p)| {
            let mut u = corner(
                i < 2,
                LinkPose::new(p, [0., 0., 0., 1.]).unwrap(),
                TravelState {
                    travel_m: 0.,
                    rate_m_s: 0.,
                },
                None,
            );
            u.add_spring(
                LinkSpring::new(
                    [0., 0.6, 0.],
                    [0.; 3],
                    0.6 + preload,
                    20000.,
                    Damper::new(b, b),
                )
                .unwrap(),
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
                        rate_rad_s: 0.,
                    },
                )
                .unwrap(),
            )
            .unwrap();
            u
        })
        .collect();
    (
        ChassisSystem::new(
            body(
                DVec3::new(0., equilibrium + height_offset, 0.),
                DQuat::IDENTITY,
                DVec3::ZERO,
                DVec3::ZERO,
            ),
            uprights,
            [0., -9.81, 0.],
        )
        .unwrap(),
        equilibrium,
    )
}

fn moving_racks(time: f64) -> [asc_physics::RackMotion; 4] {
    std::array::from_fn(|i| {
        let sign = if i % 2 == 0 { 1. } else { -1. };
        let p = DVec3::new(0.002 * sign, -0.001, 0.001);
        let v = DVec3::new(0.01 * sign, 0.005, -0.004);
        let a = DVec3::new(0.03 * sign, -0.01, 0.02);
        asc_physics::RackMotion::new(
            (p + v * time + 0.5 * a * time * time).to_array(),
            (v + a * time).to_array(),
            a.to_array(),
        )
        .unwrap()
    })
}

#[test]
fn prescribed_internal_rack_work_converges_and_fixed_prescriptions_preserve_existing_steps() {
    let zero = [TravelState {
        travel_m: 0.02,
        rate_m_s: 0.,
    }; 4];
    let spin = [WheelState {
        angle_rad: 0.1,
        rate_rad_s: 5.,
    }; 4];
    let mut residuals = Vec::new();
    for (steps, dt) in [(100, 0.001), (200, 0.0005)] {
        let mut system = rotor_four_racks(
            body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
            zero,
            spin,
            [0.; 3],
            moving_racks(0.),
        );
        let initial = system
            .sample_with_forces(|_, _| ChassisLoads::zero(4))
            .unwrap();
        let mut prev = initial.rack_actuator_power_w - initial.dissipated_power_w;
        let mut work = 0.;
        let mut final_sample = initial;
        let energy = total_energy(&final_sample);
        let momentum = vector(final_sample.linear_momentum_world_kg_m_s);
        let angular = vector(final_sample.angular_momentum_about_world_origin_kg_m2_s);
        for i in 0..steps {
            let start = f64::from(i) * dt;
            final_sample = system
                .step_with_racks_and_forces(
                    dt,
                    |offset| Ok(moving_racks(start + offset).to_vec()),
                    |_, _| ChassisLoads::zero(4),
                )
                .unwrap();
            let power = final_sample.rack_actuator_power_w - final_sample.dissipated_power_w;
            work += 0.5 * dt * (prev + power);
            prev = power;
        }
        let residual = (total_energy(&final_sample) - energy - work).abs();
        residuals.push(residual);
        close(final_sample.linear_momentum_world_kg_m_s, momentum, 1e-4);
        close(
            final_sample.angular_momentum_about_world_origin_kg_m2_s,
            angular,
            1e-4,
        );
    }
    assert!(
        residuals[1] < 1e-3 && residuals[1] < 0.4 * residuals[0],
        "{residuals:?}"
    );
    let mut legacy = rotor_four(
        body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
        zero,
        spin,
        [0.; 3],
    );
    let mut prescribed = legacy.clone();
    let racks = prescribed.rack_motions();
    let a = legacy
        .step_with_forces(0.001, |_, _| ChassisLoads::zero(4))
        .unwrap();
    let b = prescribed
        .step_with_racks_and_forces(0.001, |_| Ok(racks.clone()), |_, _| ChassisLoads::zero(4))
        .unwrap();
    close(b.chassis.position_m, vector(a.chassis.position_m), 1e-12);
    close(
        b.chassis.velocity_world_m_s,
        vector(a.chassis.velocity_world_m_s),
        1e-12,
    );
    near(total_energy(&a), total_energy(&b), 1e-10);
}
#[test]
fn moving_rack_velocities_power_and_global_momenta_match_independent_state_differences() {
    let p = DVec3::new(3., 4., 5.);
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.3, 0.1);
    let v = DVec3::new(1., -0.2, 0.3);
    let w = DVec3::new(0.2, -0.15, 0.3);
    let g = [0.3, -9.81, 0.1];
    let states = diverse_states();
    let spin = spins();
    let wheel_external = Wrench::new([2., -3., 1.], [0.3, 0.1, -0.2]).unwrap();
    let motors = [8., -6., 4., -2.].map(|t| WheelLoad::new(wheel_external, t).unwrap());
    let system = rotor_four_racks(body(p, q, v, w), states, spin, g, moving_racks(0.));
    let sample = system
        .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motors)
        .unwrap();
    let mut power = sample.rack_actuator_power_w - sample.dissipated_power_w;
    let mut force = DVec3::from_array(g) * sample.total_mass_kg;
    let mut moment = vector(sample.center_of_mass_world_m).cross(force);
    for (i, u) in sample.uprights.iter().enumerate() {
        let wheel = u.wheel.as_ref().unwrap();
        power += [8., -6., 4., -2.][i] * spin[i].rate_rad_s
            + vector(wheel_external.force_world_n()).dot(vector(wheel.body.velocity_world_m_s))
            + vector(wheel_external.torque_world_nm())
                .dot(vector(wheel.body.angular_velocity_world_rad_s));
        force += vector(wheel_external.force_world_n());
        moment += vector(wheel.body.position_m).cross(vector(wheel_external.force_world_n()))
            + vector(wheel_external.torque_world_nm());
    }
    let eps = 1e-5;
    let mut neighbors = Vec::new();
    for dt in [-eps, eps] {
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
        let b = body(
            p + v * dt,
            DQuat::from_scaled_axis(w * dt) * q,
            v + vector(sample.acceleration_world_m_s2) * dt,
            w + vector(sample.angular_acceleration_world_rad_s2) * dt,
        );
        neighbors.push(
            rotor_four_racks(b, hs, ss, g, moving_racks(dt))
                .sample_with_wheels(Wrench::default(), &[Wrench::default(); 4], &motors)
                .unwrap(),
        );
    }
    near(
        (total_energy(&neighbors[1]) - total_energy(&neighbors[0])) / (2. * eps),
        power,
        0.02,
    );
    for (i, u) in sample.uprights.iter().enumerate() {
        let bodies = [
            (
                &u.body,
                &neighbors[0].uprights[i].body,
                &neighbors[1].uprights[i].body,
            ),
            (
                &u.wheel.as_ref().unwrap().body,
                &neighbors[0].uprights[i].wheel.as_ref().unwrap().body,
                &neighbors[1].uprights[i].wheel.as_ref().unwrap().body,
            ),
        ];
        for (b, minus, plus) in bodies {
            close(
                ((vector(plus.position_m) - vector(minus.position_m)) / (2. * eps)).to_array(),
                vector(b.velocity_world_m_s),
                1e-6,
            );
            let dq = DQuat::from_array(plus.orientation_xyzw)
                * DQuat::from_array(minus.orientation_xyzw).conjugate();
            close(
                (dq.to_scaled_axis() / (2. * eps)).to_array(),
                vector(b.angular_velocity_world_rad_s),
                1e-5,
            );
        }
    }
    close(
        ((vector(neighbors[1].linear_momentum_world_kg_m_s)
            - vector(neighbors[0].linear_momentum_world_kg_m_s))
            / (2. * eps))
            .to_array(),
        force,
        0.01,
    );
    close(
        ((vector(neighbors[1].angular_momentum_about_world_origin_kg_m2_s)
            - vector(neighbors[0].angular_momentum_about_world_origin_kg_m2_s))
            / (2. * eps))
            .to_array(),
        moment,
        0.03,
    );
}

#[test]
fn prescribed_racks_are_sampled_before_forces_commit_final_state_and_rollback_each_failure() {
    let states = [TravelState {
        travel_m: 0.02,
        rate_m_s: 0.,
    }; 4];
    let spin = [WheelState {
        angle_rad: 0.,
        rate_rad_s: 1.,
    }; 4];
    let original = rotor_four_racks(
        body(DVec3::ZERO, DQuat::IDENTITY, DVec3::ZERO, DVec3::ZERO),
        states,
        spin,
        [0.; 3],
        moving_racks(0.),
    );
    let before = serde_json::to_value(original.kinematics().unwrap()).unwrap();
    let mut invalid = moving_racks(0.);
    invalid[3].acceleration_m_s2 = [f64::NAN; 3];
    assert!(
        original
            .sample_with_racks_and_forces(&invalid, |_, _| ChassisLoads::zero(4))
            .is_err()
    );
    assert_eq!(
        serde_json::to_value(original.kinematics().unwrap()).unwrap(),
        before
    );
    for failure in [0., 0.0005, 0.001] {
        let mut s = original.clone();
        let result = s.step_with_racks_and_forces(
            0.001,
            |time| {
                if time == failure {
                    return Err(asc_physics::Error::new(
                        "rack_fixture",
                        "Deliberate rack failure",
                    ));
                }
                Ok(moving_racks(time).to_vec())
            },
            |_, _| ChassisLoads::zero(4),
        );
        assert!(result.is_err());
        assert_eq!(
            serde_json::to_value(s.kinematics().unwrap()).unwrap(),
            before
        );
        let result = s.step_with_racks_and_forces(
            0.001,
            |t| Ok(moving_racks(t).to_vec()),
            |time, _| {
                if time == failure {
                    return Err(asc_physics::Error::new(
                        "force_fixture",
                        "Deliberate force failure",
                    ));
                }
                ChassisLoads::zero(4)
            },
        );
        assert!(result.is_err());
        assert_eq!(
            serde_json::to_value(s.kinematics().unwrap()).unwrap(),
            before
        );
    }
    let mut s = original.clone();
    assert!(
        s.step_with_racks_and_forces(
            0.001,
            |t| {
                if t == 0.001 {
                    Ok(vec![])
                } else {
                    Ok(moving_racks(t).to_vec())
                }
            },
            |_, _| ChassisLoads::zero(4)
        )
        .is_err()
    );
    assert_eq!(
        serde_json::to_value(s.kinematics().unwrap()).unwrap(),
        before
    );
    assert!(
        s.step_with_forces(0.001, |_, _| ChassisLoads::zero(4))
            .is_err()
    );
    let mut seen = Vec::new();
    s.step_with_racks_and_forces(
        0.001,
        |t| Ok(moving_racks(t).to_vec()),
        |t, k| {
            for (u, r) in k.uprights.iter().zip(moving_racks(t)) {
                assert_eq!(u.rack_motion, r);
            }
            seen.push(t);
            ChassisLoads::zero(4)
        },
    )
    .unwrap();
    assert_eq!(seen, vec![0., 0.0005, 0.001]);
    assert_eq!(s.rack_motions(), moving_racks(0.001));
    let result = s.step_with_racks_and_forces(
        0.001,
        |_| Ok(moving_racks(0.).to_vec()),
        |_, _| ChassisLoads::zero(4),
    );
    assert_eq!(result.unwrap_err().code, "physics_rack_discontinuity");
    assert!(asc_physics::RackMotion::new([f64::NAN; 3], [0.; 3], [0.; 3]).is_err());
}

#[test]
fn regularized_internal_brake_matches_two_rotor_solution_recoil_and_energy_loss() {
    let brake = asc_physics::WheelBrake::new(40., 5.).unwrap();
    let inertia = 1. / (1. / 1.2 + 1. / 10.4);
    for strut in [false, true] {
        for sign in [-1., 1.] {
            let mut errors = Vec::new();
            for (dt, steps) in [(0.002, 250), (0.001, 500)] {
                let mut system = coaxial_rotor(strut, 2. * sign);
                let loads = |_: f64, k: &ChassisKinematics| -> asc_physics::Result<ChassisLoads> {
                    let b =
                        brake.sample(k.uprights[0].wheel.as_ref().unwrap().state.rate_rad_s, 1.)?;
                    let mut loads = ChassisLoads::zero(1)?;
                    loads.wheels[0].add_internal_torque(b.torque_on_wheel_nm)?;
                    Ok(loads)
                };
                let initial = system.sample_with_forces(loads).unwrap();
                let momentum = initial.angular_momentum_about_world_origin_kg_m2_s;
                let mut previous = initial.kinetic_energy_j;
                for _ in 0..steps {
                    let s = system.step_with_forces(dt, loads).unwrap();
                    let rate = s.uprights[0].wheel.as_ref().unwrap().state.rate_rad_s;
                    assert!(rate * sign > 0.);
                    assert!(s.kinetic_energy_j <= previous + 1e-12);
                    previous = s.kinetic_energy_j;
                    close(
                        s.angular_momentum_about_world_origin_kg_m2_s,
                        vector(momentum),
                        1e-9,
                    );
                }
                let s = system.sample_with_forces(loads).unwrap();
                let relative = 2. * sign * (-8. / inertia * 0.5f64).exp();
                let chassis_rate = 1.2 / 11.6 * (2. * sign - relative);
                errors.push(
                    (s.uprights[0].wheel.as_ref().unwrap().state.rate_rad_s - relative).abs(),
                );
                near(
                    s.chassis.angular_velocity_world_rad_s[0],
                    chassis_rate,
                    1e-6,
                );
                let energy = 0.5 * inertia * relative.powi(2) + 0.5 * (2.4f64).powi(2) / 11.6;
                near(s.kinetic_energy_j, energy, 1e-6);
            }
            assert!(errors[1] < 0.3 * errors[0], "{errors:?}");
        }
    }
}
fn normal_loads(
    k: &ChassisKinematics,
    law: &NormalContact,
    plane: SupportPlane,
) -> asc_physics::Result<ChassisLoads> {
    let mut loads = ChassisLoads::zero(k.uprights.len())?;
    for (i, u) in k.uprights.iter().enumerate() {
        loads.wheels[i].add_external(law.sample(u.wheel.as_ref().unwrap(), plane)?.wrench()?)?;
    }
    Ok(loads)
}

#[test]
fn friction_transmits_motor_torque_to_vehicle_and_balances_work_and_dissipation() {
    let law = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let friction = asc_physics::TangentialContact::new(3000., 4000., 0.9, 0.9).unwrap();
    let plane = SupportPlane::new([0.; 3], [0., 1., 0.]).unwrap();
    let evaluate = |system: &ChassisSystem| {
        let k = system.kinematics().unwrap();
        let s = system
            .sample_with_forces(|_, k| {
                let mut loads = ChassisLoads::zero(4)?;
                for (i, u) in k.uprights.iter().enumerate() {
                    loads.wheels[i] = WheelLoad::new(
                        law.sample_with_friction(u.wheel.as_ref().unwrap(), plane, &friction, 1.)?
                            .wrench()?,
                        10.,
                    )?;
                }
                Ok(loads)
            })
            .unwrap();
        let mut energy = total_energy(&s);
        let mut dissipation = s.dissipated_power_w;
        let mut power = 0.;
        for u in k.uprights {
            let wheel = u.wheel.unwrap();
            let c = law
                .sample_with_friction(&wheel, plane, &friction, 1.)
                .unwrap();
            energy += c.normal.potential_energy_j;
            dissipation += c.normal.dissipated_power_w + c.tangential.dissipated_power_w;
            power += 10. * wheel.state.rate_rad_s;
        }
        (s, energy, power - dissipation)
    };
    let mut residuals = Vec::new();
    for (steps, dt) in [(500, 0.001), (1000, 0.0005)] {
        let (mut system, _) = normal_support_system(0.);
        let (_, initial, mut previous_power) = evaluate(&system);
        let mut work = 0.;
        for _ in 0..steps {
            system
                .step_with_forces(dt, |_, k| {
                    let mut loads = ChassisLoads::zero(4)?;
                    for (i, u) in k.uprights.iter().enumerate() {
                        loads.wheels[i] = WheelLoad::new(
                            law.sample_with_friction(
                                u.wheel.as_ref().unwrap(),
                                plane,
                                &friction,
                                1.,
                            )?
                            .wrench()?,
                            10.,
                        )?;
                    }
                    Ok(loads)
                })
                .unwrap();
            let (_, _, power) = evaluate(&system);
            work += 0.5 * dt * (previous_power + power);
            previous_power = power;
        }
        let (s, energy, _) = evaluate(&system);
        assert!(s.linear_momentum_world_kg_m_s[2] > 50.);
        assert!(
            s.uprights
                .iter()
                .all(|u| u.wheel.as_ref().unwrap().state.rate_rad_s > 0.)
        );
        residuals.push((energy - initial - work).abs());
    }
    assert!(residuals[1] < 0.01, "{residuals:?}");
    assert!(residuals[1] < 0.4 * residuals[0], "{residuals:?}");
}
#[test]
fn normal_support_has_analytical_static_equilibrium_and_damped_coupled_settling() {
    let law = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let plane = SupportPlane::new([0.; 3], [0., 1., 0.]).unwrap();
    let (mut equilibrium, height) = normal_support_system(0.);
    let initial = equilibrium
        .sample_with_forces(|_, k| normal_loads(k, &law, plane))
        .unwrap();
    close(initial.acceleration_world_m_s2, DVec3::ZERO, 1e-9);
    for u in &initial.uprights {
        near(u.travel_acceleration_m_s2, 0., 1e-9);
    }
    for _ in 0..50 {
        let s = equilibrium
            .step_with_forces(0.001, |_, k| normal_loads(k, &law, plane))
            .unwrap();
        near(s.chassis.position_m[1], height, 1e-10);
    }
    let energy = |system: &ChassisSystem| {
        let s = system
            .sample_with_forces(|_, k| normal_loads(k, &law, plane))
            .unwrap();
        let potential = system
            .kinematics()
            .unwrap()
            .uprights
            .iter()
            .map(|u| {
                law.sample(u.wheel.as_ref().unwrap(), plane)
                    .unwrap()
                    .potential_energy_j
            })
            .sum::<f64>();
        total_energy(&s) + potential
    };
    let base = energy(&equilibrium);
    let (mut falling, _) = normal_support_system(0.003);
    let initial = energy(&falling);
    near(initial - base, 3.6, 1e-7);
    let mut previous = initial;
    for _ in 0..1000 {
        falling
            .step_with_forces(0.001, |_, k| normal_loads(k, &law, plane))
            .unwrap();
        let current = energy(&falling);
        assert!(current <= previous + 1e-6, "{current} > {previous}");
        previous = current;
    }
    assert!(
        previous - base < (initial - base) * 0.01,
        "{} / {}",
        previous - base,
        initial - base
    );
    let final_state = falling.kinematics().unwrap();
    near(final_state.chassis.position_m[1], height, 5e-5);
}

#[test]
fn selected_midpoint_path_fraction_matches_retained_translation_rotation_and_full_step() {
    use std::cell::RefCell;
    for fraction in [0., 1e-8, 0.2, 0.7, 1.] {
        let mut system = impulse_fixture(true);
        let initial = system.kinematics().unwrap();
        let racks = system.rack_motions();
        let dt = 0.002;
        let offsets = RefCell::new(Vec::new());
        let force = |_: f64, _: &ChassisKinematics| {
            Ok(ChassisLoads {
                chassis: Wrench::new([170., 30., -50.], [2., -3., 1.]).unwrap(),
                uprights: vec![Wrench::default(); 4],
                wheels: vec![WheelLoad::default(); 4],
            })
        };
        let prescribe = |t: f64| {
            racks
                .iter()
                .map(|r| {
                    asc_physics::RackMotion::new(
                        (DVec3::from_array(r.position_m) + DVec3::from_array(r.velocity_m_s) * t)
                            .to_array(),
                        r.velocity_m_s,
                        [0.; 3],
                    )
                })
                .collect()
        };
        let mut ordinary = system.clone();
        let full = ordinary
            .step_with_racks_and_forces(dt, prescribe, force)
            .unwrap();
        let result = system
            .step_with_racks_and_forces_until(
                dt,
                |t| {
                    offsets.borrow_mut().push(t);
                    prescribe(t)
                },
                force,
                |path| {
                    assert_eq!(path.duration_s, dt);
                    assert_eq!(path.start.position_m, initial.chassis.position_m);
                    close(
                        path.end.position_m,
                        DVec3::from_array(full.chassis.position_m),
                        1e-12,
                    );
                    Ok(fraction)
                },
            )
            .unwrap();
        near(result.elapsed_s, dt * fraction, 1e-20);
        assert_eq!(*offsets.borrow(), vec![0., dt * 0.5, dt * fraction]);
        let expected = DVec3::from_array(result.proposal.start.position_m)
            .lerp(DVec3::from_array(result.proposal.end.position_m), fraction);
        close(result.sample.chassis.position_m, expected, 1e-12);
        let q = DQuat::from_scaled_axis(
            DVec3::from_array(result.proposal.rotation_world_rad) * fraction,
        ) * DQuat::from_array(initial.chassis.orientation_xyzw);
        assert!(
            (DQuat::from_array(result.sample.chassis.orientation_xyzw)
                .dot(q)
                .abs()
                - 1.)
                .abs()
                < 1e-12
        );
        for (u, r) in system
            .kinematics()
            .unwrap()
            .uprights
            .iter()
            .zip(prescribe(dt * fraction).unwrap())
        {
            assert_eq!(u.rack_motion, r);
        }
        if fraction == 1. {
            assert_eq!(
                serde_json::to_value(system.kinematics().unwrap()).unwrap(),
                serde_json::to_value(ordinary.kinematics().unwrap()).unwrap()
            );
        }
        if fraction == 0. {
            assert_eq!(
                serde_json::to_value(system.kinematics().unwrap()).unwrap(),
                serde_json::to_value(initial).unwrap()
            );
        }
    }
}
#[test]
fn fraction_selector_and_final_callbacks_fail_atomically_for_all_rack_and_mechanical_state() {
    use asc_physics::{Error, RackMotion};
    for case in 0..6 {
        let mut system = impulse_fixture(false);
        let before = serde_json::to_value(system.kinematics().unwrap()).unwrap();
        let racks = system.rack_motions();
        let result = system.step_with_racks_and_forces_until(
            0.002,
            |t| {
                if case == 4 && t == 0.0005 {
                    return Err(Error::new("test_final_rack", "rejected selected endpoint"));
                }
                Ok(racks.clone())
            },
            |t, _| {
                if case == 5 && t == 0.0005 {
                    return Err(Error::new("test_final_force", "rejected selected endpoint"));
                }
                Ok(ChassisLoads {
                    chassis: Wrench::default(),
                    uprights: vec![Wrench::default(); 4],
                    wheels: vec![WheelLoad::default(); 4],
                })
            },
            |_| match case {
                0 => Err(Error::new("test_selector", "rejected path")),
                1 => Ok(-0.1),
                2 => Ok(1.1),
                3 => Ok(f64::NAN),
                _ => Ok(0.25),
            },
        );
        assert!(result.is_err(), "case {case}");
        assert_eq!(
            serde_json::to_value(system.kinematics().unwrap()).unwrap(),
            before
        );
        assert!(
            system
                .rack_motions()
                .iter()
                .all(|r| r == &RackMotion::fixed(r.position_m).unwrap())
        );
    }
}
#[test]
fn partial_midpoint_impact_and_remaining_time_match_analytical_free_translation() {
    use asc_physics::{ChassisContact, ChassisContactSettings};
    let mut system = contact_fixture(DQuat::IDENTITY, DVec3::ZERO);
    let racks = system.rack_motions();
    let dt = 0.01;
    let force = |_: f64, _: &ChassisKinematics| {
        Ok(ChassisLoads {
            chassis: Wrench::default(),
            uprights: vec![Wrench::default()],
            wheels: vec![WheelLoad::default()],
        })
    };
    let result = system
        .step_with_racks_and_forces_until(
            dt,
            |_| Ok(racks.clone()),
            force,
            |path| {
                // A plane at x=-0.01 constrains this reference point. Initial x speed is -2 m/s.
                let x0 = path.start.position_m[0];
                let x1 = path.end.position_m[0];
                Ok((-0.01 - x0) / (x1 - x0))
            },
        )
        .unwrap();
    near(result.elapsed_s, 0.005, 1e-12);
    near(result.sample.chassis.position_m[0], -0.01, 1e-12);
    let impact = system
        .resolve_chassis_contacts(
            &[ChassisContact {
                point_world_m: result.sample.chassis.position_m,
                normal_world: [1., 0., 0.],
                friction: 0.,
            }],
            ChassisContactSettings::default(),
        )
        .unwrap();
    assert!(impact.net_impulse_world_ns[0] > 0.);
    let rest = system
        .step_with_racks_and_forces(dt - result.elapsed_s, |_| Ok(racks.clone()), force)
        .unwrap();
    near(rest.chassis.position_m[0], -0.01, 1e-12);
    near(rest.chassis.velocity_world_m_s[0], 0., 1e-12);
    near(rest.chassis.position_m[1], 0.02, 1e-12);
    assert!(impact.work.external_impulse_work_j < 0.);
}
