use asc_physics::{
    Body, Damper, DamperBranch, FiveLink, LinkDifferential, LinkPose, LinkSpring, MassProperties,
    SpringLink, StrutLink,
};
use glam::{DQuat, DVec3};

enum Mechanism {
    Dwb(FiveLink),
    Strut(StrutLink),
}
impl Mechanism {
    fn map(&self, h: f64, rack: [f64; 3]) -> LinkDifferential {
        match self {
            Self::Dwb(m) => {
                m.differential(h, rack, m.solve(h, rack, None).unwrap().pose().unwrap())
            }
            Self::Strut(m) => {
                m.differential(h, rack, m.solve(h, rack, None).unwrap().pose().unwrap())
            }
        }
        .unwrap()
    }
}
fn models(translation: DVec3, rotation: DQuat) -> [Mechanism; 2] {
    let anchors = [
        [-0.3, 0.2, 0.12],
        [-0.3, 0.2, -0.12],
        [-0.3, -0.2, 0.12],
        [-0.3, -0.2, -0.12],
        [-0.3, 0., 0.15],
    ];
    let attachments = [
        [0., 0.2, 0.],
        [0., 0.2, 0.],
        [0., -0.2, 0.],
        [0., -0.2, 0.],
        [0., 0., 0.15],
    ];
    let transform = |p: [f64; 3]| (translation + rotation * DVec3::from_array(p)).to_array();
    let pose = LinkPose::new(translation.to_array(), rotation.to_array()).unwrap();
    let axis = (rotation * DVec3::Y).to_array();
    [
        Mechanism::Dwb(FiveLink::new(anchors.map(transform), attachments, pose, axis).unwrap()),
        Mechanism::Strut(
            StrutLink::new(
                [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]].map(transform),
                [[0.; 3], [0.; 3], [0., 0., 0.15]],
                transform([0., 0.6, 0.]),
                [0.; 3],
                pose,
                axis,
            )
            .unwrap(),
        ),
    ]
}
fn linear(c: f64) -> Damper {
    let branch = DamperBranch::new(c, c, 0.).unwrap();
    Damper::new(branch, branch)
}
fn spring(c: f64) -> LinkSpring {
    LinkSpring::new([0., 0.6, 0.], [0.07, -0.08, 0.1], 0.65, 12000., linear(c)).unwrap()
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() < tol, "{a} != {b}, tolerance {tol}");
}
fn vector(a: [f64; 3]) -> DVec3 {
    DVec3::from_array(a)
}
fn close(a: [f64; 3], b: DVec3, tol: f64) {
    assert!((vector(a) - b).length() < tol, "{a:?} != {b:?}");
}

#[test]
fn axial_length_rate_and_travel_load_match_closed_form_for_both_topologies() {
    let h = 0.04_f64;
    let rate = 0.2;
    let x = -0.3 + (0.09 - h * h).sqrt();
    let dx = -h / (0.09 - h * h).sqrt();
    let length = (x * x + (h - 0.6).powi(2)).sqrt();
    let gradient = (x * dx + h - 0.6) / length;
    for model in models(DVec3::ZERO, DQuat::IDENTITY) {
        let map = model.map(h, [0.; 3]);
        let element = LinkSpring::new([0., 0.6, 0.], [0.; 3], 0.65, 12000., linear(80.)).unwrap();
        let sample = element.sample(&map, rate, [0.; 3]).unwrap();
        near(sample.length_m, length, 1e-10);
        near(sample.extension_rate_m_s, gradient * rate, 1e-10);
        near(sample.length_gradient[0], gradient, 1e-10);
        let tension = 12000. * (length - 0.65) + 80. * gradient * rate;
        near(sample.generalized_loads.travel_n, -tension * gradient, 1e-7);
        near(
            sample.potential_energy_j,
            6000. * (length - 0.65).powi(2),
            1e-9,
        );
        near(
            sample.dissipated_power_w,
            80. * (gradient * rate).powi(2),
            1e-9,
        );
    }
}

#[test]
fn all_motion_ratios_and_conservative_loads_match_independent_energy_differences() {
    for model in models(DVec3::ZERO, DQuat::IDENTITY) {
        let h = 0.035;
        let rack = [0.004, 0., 0.];
        let sample = spring(0.).sample(&model.map(h, rack), 0., [0.; 3]).unwrap();
        let generalized = [
            sample.generalized_loads.travel_n,
            sample.generalized_loads.steering_anchor_n[0],
            sample.generalized_loads.steering_anchor_n[1],
            sample.generalized_loads.steering_anchor_n[2],
        ];
        let eps = 1e-5;
        for i in 0..4 {
            let mut values = [h, rack[0], rack[1], rack[2]];
            let mut measurements = Vec::new();
            for sign in [-1., 1.] {
                values[i] += sign * eps;
                let pose = model.map(values[0], values[1..].try_into().unwrap()).pose();
                let point = vector(pose.transform([0.07, -0.08, 0.1]).unwrap());
                let length = (point - DVec3::new(0., 0.6, 0.)).length();
                measurements.push((length, 6000. * (length - 0.65).powi(2)));
                values[i] -= sign * eps;
            }
            near(
                sample.length_gradient[i],
                (measurements[1].0 - measurements[0].0) / (2. * eps),
                2e-6,
            );
            near(
                generalized[i],
                -(measurements[1].1 - measurements[0].1) / (2. * eps),
                0.003,
            );
        }
    }
}

#[test]
fn asymmetric_damping_power_point_torque_and_rigid_body_sampling_agree() {
    let damping = Damper::new(
        DamperBranch::new(80., 20., 0.05).unwrap(),
        DamperBranch::new(140., 30., 0.07).unwrap(),
    );
    for model in models(DVec3::ZERO, DQuat::IDENTITY) {
        let map = model.map(0.03, [0.003, 0., 0.]);
        for sign in [-1., 1.] {
            let rates = [sign * 0.17, sign * 0.025, sign * -0.012, sign * 0.009];
            let sample = LinkSpring::new([0., 0.6, 0.], [0.07, -0.08, 0.1], 0.65, 12000., damping)
                .unwrap()
                .sample(&map, rates[0], rates[1..].try_into().unwrap())
                .unwrap();
            let velocity = map
                .velocity(rates[0], rates[1..].try_into().unwrap())
                .unwrap();
            let point_power = vector(sample.upright_load.force_n())
                .dot(vector(sample.upright_point_velocity_m_s));
            let power = sample.generalized_loads.travel_n * rates[0]
                + vector(sample.generalized_loads.steering_anchor_n)
                    .dot(DVec3::new(rates[1], rates[2], rates[3]));
            near(power, point_power, 1e-8);
            near(
                power
                    + sample.elastic_tension_n * sample.extension_rate_m_s
                    + sample.dissipated_power_w,
                0.,
                1e-8,
            );
            assert!(sample.dissipated_power_w >= 0.);
            close(
                sample.force_on_frame_n,
                -vector(sample.upright_load.force_n()),
                1e-8,
            );
            close(
                sample.torque_about_frame_origin_nm,
                -vector(sample.upright_load.torque_about_upright_origin_nm())
                    - vector(map.pose().position_m()).cross(vector(sample.upright_load.force_n())),
                1e-8,
            );
            let properties =
                MassProperties::new(2., [[2., 0., 0.], [0., 2., 0.], [0., 0., 2.]]).unwrap();
            let a = Body::new(
                properties.clone(),
                [0.; 3],
                [0., 0., 0., 1.],
                [0.; 3],
                [0.; 3],
            )
            .unwrap();
            let b = Body::new(
                properties,
                map.pose().position_m(),
                map.pose().orientation_xyzw(),
                velocity.velocity_m_s(),
                velocity.angular_velocity_rad_s(),
            )
            .unwrap();
            let (_, _, body_sample) =
                SpringLink::new([0., 0.6, 0.], [0.07, -0.08, 0.1], 0.65, 12000., damping)
                    .unwrap()
                    .loads(&a, &b)
                    .unwrap();
            near(body_sample.length_m, sample.length_m, 1e-12);
            near(
                body_sample.extension_rate_m_s,
                sample.extension_rate_m_s,
                1e-12,
            );
            near(
                body_sample.damping_tension_n,
                sample.damping_tension_n,
                1e-10,
            );
            close(
                body_sample.force_on_b_world_n,
                vector(sample.upright_load.force_n()),
                1e-8,
            );
        }
    }
}

#[test]
fn mounting_forces_moments_and_motion_ratios_transform_with_the_fixed_frame() {
    let rotation = DQuat::from_euler(glam::EulerRot::XYZ, 0.3, -0.2, 0.4);
    let translation = DVec3::new(3., -2., 4.);
    for (base, moved) in models(DVec3::ZERO, DQuat::IDENTITY)
        .into_iter()
        .zip(models(translation, rotation))
    {
        let rack = DVec3::new(0.003, 0., 0.);
        let rack_velocity = DVec3::new(0.025, -0.012, 0.009);
        let first = spring(80.)
            .sample(
                &base.map(0.03, rack.to_array()),
                0.17,
                rack_velocity.to_array(),
            )
            .unwrap();
        let second = LinkSpring::new(
            (translation + rotation * DVec3::new(0., 0.6, 0.)).to_array(),
            [0.07, -0.08, 0.1],
            0.65,
            12000.,
            linear(80.),
        )
        .unwrap()
        .sample(
            &moved.map(0.03, (rotation * rack).to_array()),
            0.17,
            (rotation * rack_velocity).to_array(),
        )
        .unwrap();
        near(first.length_m, second.length_m, 1e-9);
        near(first.extension_rate_m_s, second.extension_rate_m_s, 1e-8);
        near(first.length_gradient[0], second.length_gradient[0], 1e-8);
        near(
            first.generalized_loads.travel_n,
            second.generalized_loads.travel_n,
            1e-5,
        );
        close(
            second.upright_load.force_n(),
            rotation * vector(first.upright_load.force_n()),
            1e-5,
        );
        close(
            second.upright_load.torque_about_upright_origin_nm(),
            rotation * vector(first.upright_load.torque_about_upright_origin_nm()),
            1e-5,
        );
        close(
            second.generalized_loads.steering_anchor_n,
            rotation * vector(first.generalized_loads.steering_anchor_n),
            1e-5,
        );
        close(
            second.torque_about_frame_origin_nm,
            rotation * vector(first.torque_about_frame_origin_nm)
                + translation.cross(rotation * vector(first.force_on_frame_n)),
            1e-5,
        );
    }
}

#[test]
fn invalid_mounts_laws_coincidence_and_rates_fail_without_mutation() {
    for (anchor, point, rest, k) in [
        ([101., 0., 0.], [0.; 3], 0.6, 1.),
        ([0.; 3], [f64::NAN, 0., 0.], 0.6, 1.),
        ([0.; 3], [0.; 3], 0., 1.),
        ([0.; 3], [0.; 3], 0.6, -1.),
    ] {
        assert!(LinkSpring::new(anchor, point, rest, k, linear(0.)).is_err());
    }
    for model in models(DVec3::ZERO, DQuat::IDENTITY) {
        let map = model.map(0., [0.; 3]);
        let before = serde_json::to_string(&map).unwrap();
        let collapsed = LinkSpring::new([0.; 3], [0.; 3], 0.6, 1., linear(0.)).unwrap();
        assert!(collapsed.sample(&map, 0., [0.; 3]).is_err());
        assert!(spring(0.).sample(&map, f64::NAN, [0.; 3]).is_err());
        assert!(spring(0.).sample(&map, 0., [1001., 0., 0.]).is_err());
        assert_eq!(serde_json::to_string(&map).unwrap(), before);
    }
}
