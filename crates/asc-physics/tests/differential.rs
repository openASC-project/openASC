use asc_physics::{FiveLink, LinkDifferential, LinkLoad, LinkPose, StrutLink};
use glam::{DQuat, DVec3};

enum Mechanism {
    Dwb(FiveLink),
    Strut(StrutLink),
}
impl Mechanism {
    fn solve(&self, h: f64, rack: [f64; 3], seed: Option<LinkPose>) -> LinkPose {
        match self {
            Self::Dwb(m) => m.solve(h, rack, seed).unwrap().pose().unwrap(),
            Self::Strut(m) => m.solve(h, rack, seed).unwrap().pose().unwrap(),
        }
    }
    fn differential(
        &self,
        h: f64,
        rack: [f64; 3],
        pose: LinkPose,
    ) -> asc_physics::Result<LinkDifferential> {
        match self {
            Self::Dwb(m) => m.differential(h, rack, pose),
            Self::Strut(m) => m.differential(h, rack, pose),
        }
    }
}
fn reference() -> LinkPose {
    LinkPose::new([0.; 3], [0., 0., 0., 1.]).unwrap()
}
fn dwb_points() -> ([[f64; 3]; 5], [[f64; 3]; 5]) {
    (
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
    )
}
fn dwb() -> Mechanism {
    let (a, b) = dwb_points();
    Mechanism::Dwb(FiveLink::new(a, b, reference(), [0., 1., 0.]).unwrap())
}
fn strut_points() -> ([[f64; 3]; 3], [[f64; 3]; 3]) {
    (
        [[-0.3, 0., 0.12], [-0.3, 0., -0.12], [-0.3, 0., 0.15]],
        [[0.; 3], [0.; 3], [0., 0., 0.15]],
    )
}
fn strut() -> Mechanism {
    let (a, b) = strut_points();
    Mechanism::Strut(
        StrutLink::new(a, b, [0., 0.6, 0.], [0.; 3], reference(), [0., 1., 0.]).unwrap(),
    )
}
fn vector(value: [f64; 3]) -> DVec3 {
    DVec3::from_array(value)
}
fn close(a: DVec3, b: DVec3, tol: f64) {
    assert!((a - b).length() < tol, "{a:?} != {b:?}");
}

#[test]
fn travel_velocity_matches_closed_form_and_strut_geometric_length_rate() {
    let h = 0.04_f64;
    let rate = 0.2;
    let root = (0.09 - h * h).sqrt();
    let slope = -h / root;
    let x = -0.3 + root;
    for model in [dwb(), strut()] {
        let pose = model.solve(h, [0.; 3], None);
        let map = model.differential(h, [0.; 3], pose).unwrap();
        let velocity = map.velocity(rate, [0.; 3]).unwrap();
        close(
            vector(velocity.velocity_m_s()),
            DVec3::new(slope * rate, rate, 0.),
            1e-9,
        );
        let expected_omega = match model {
            Mechanism::Dwb(_) => 0.,
            Mechanism::Strut(_) => ((0.6 - h) * slope + x) / ((0.6 - h).powi(2) + x * x) * rate,
        };
        close(
            vector(velocity.angular_velocity_rad_s()),
            DVec3::new(0., 0., expected_omega),
            1e-9,
        );
        if let Mechanism::Strut(_) = model {
            let axis = DQuat::from_array(pose.orientation_xyzw()) * DVec3::Y;
            let length_rate = -axis.dot(vector(velocity.point_velocity_m_s([0.; 3]).unwrap()));
            let expected = (x * slope - (0.6 - h)) / x.hypot(0.6 - h) * rate;
            assert!((length_rate - expected).abs() < 1e-9);
        }
    }
}

#[test]
fn all_four_tangent_columns_match_independent_pose_perturbations() {
    let h = 0.03;
    let rack = [0.003, -0.001, 0.002];
    let epsilon = 1e-5;
    for model in [dwb(), strut()] {
        let pose = model.solve(h, rack, None);
        let map = model.differential(h, rack, pose).unwrap();
        for (i, column) in map.columns().iter().enumerate() {
            let (mut low_h, mut high_h) = (h, h);
            let (mut low_rack, mut high_rack) = (rack, rack);
            if i == 0 {
                low_h -= epsilon;
                high_h += epsilon;
            } else {
                low_rack[i - 1] -= epsilon;
                high_rack[i - 1] += epsilon;
            }
            let low = model.solve(low_h, low_rack, Some(pose));
            let high = model.solve(high_h, high_rack, Some(pose));
            let v = (vector(high.position_m()) - vector(low.position_m())) / (2. * epsilon);
            let delta = DQuat::from_array(high.orientation_xyzw())
                * DQuat::from_array(low.orientation_xyzw()).conjugate();
            let omega = delta.to_scaled_axis() / (2. * epsilon);
            close(v, vector(column.origin_per_m), 3e-5);
            close(omega, vector(column.angular_rad_per_m), 3e-5);
        }
    }
}

#[test]
fn point_force_projection_preserves_virtual_power_and_correct_torque_origin() {
    let rates = [0.17, 0.025, -0.012, 0.009];
    let p = [0.03, -0.25, 0.08];
    let force = [120., 2300., -300.];
    for model in [dwb(), strut()] {
        let pose = model.solve(0.035, [0.004, 0., 0.], None);
        let map = model.differential(0.035, [0.004, 0., 0.], pose).unwrap();
        let velocity = map
            .velocity(rates[0], rates[1..].try_into().unwrap())
            .unwrap();
        let load = LinkLoad::at_point(force, p, pose).unwrap();
        let projected = map.project_load(load).unwrap();
        let point_power = vector(force).dot(vector(velocity.point_velocity_m_s(p).unwrap()));
        let wrench_power = vector(load.force_n()).dot(vector(velocity.velocity_m_s()))
            + vector(load.torque_about_upright_origin_nm())
                .dot(vector(velocity.angular_velocity_rad_s()));
        let actuator_power = projected.travel_n * rates[0]
            + vector(projected.steering_anchor_n).dot(DVec3::new(rates[1], rates[2], rates[3]));
        assert!((point_power - wrench_power).abs() < 1e-9);
        assert!((point_power - actuator_power).abs() < 1e-9);
    }
}

#[test]
fn held_reactions_balance_force_torque_and_projected_actuator_loads() {
    let h = 0.035;
    let rack = [0.004, 0., 0.];
    let load = LinkLoad::new([120., 2300., -300.], [40., -15., 25.]).unwrap();
    for model in [dwb(), strut()] {
        let pose = model.solve(h, rack, None);
        let map = model.differential(h, rack, pose).unwrap();
        let balance = map.hold_load(load).unwrap();
        let generalized = map.project_load(load).unwrap();
        close(
            vector(load.force_n())
                + vector(balance.support_force_on_upright_n)
                + vector(balance.travel_actuator_force_on_upright_n),
            DVec3::ZERO,
            1e-8,
        );
        close(
            vector(load.torque_about_upright_origin_nm())
                + vector(balance.support_torque_on_upright_nm),
            DVec3::ZERO,
            1e-8,
        );
        assert!((generalized.travel_n + balance.travel_actuator_n).abs() < 1e-8);
        close(
            vector(generalized.steering_anchor_n),
            vector(balance.steering_anchor_force_n),
            1e-8,
        );
        close(
            vector(balance.support_force_on_frame_n),
            -vector(balance.support_force_on_upright_n),
            1e-8,
        );
        let mut reconstructed_force = DVec3::ZERO;
        let mut reconstructed_torque = DVec3::ZERO;
        let mut frame_torque = DVec3::ZERO;
        let (anchors, attachments): (Vec<_>, Vec<_>) = match model {
            Mechanism::Dwb(_) => {
                let (a, b) = dwb_points();
                (a.to_vec(), b.to_vec())
            }
            Mechanism::Strut(_) => {
                let (a, b) = strut_points();
                (a.to_vec(), b.to_vec())
            }
        };
        for (i, (anchor, attachment)) in anchors.iter().zip(&attachments).enumerate() {
            let anchor = vector(*anchor)
                + if i == anchors.len() - 1 {
                    vector(rack)
                } else {
                    DVec3::ZERO
                };
            let end = vector(pose.transform(*attachment).unwrap());
            let force = balance.constraint_multipliers_n[i] * (end - anchor).normalize();
            reconstructed_force += force;
            reconstructed_torque += (end - vector(pose.position_m())).cross(force);
            frame_torque -= anchor.cross(force);
        }
        if let Mechanism::Strut(_) = model {
            let coaxial_force = vector(balance.support_force_on_upright_n) - reconstructed_force;
            let top = DVec3::new(0., 0.6, 0.);
            let axis = DQuat::from_array(pose.orientation_xyzw()) * DVec3::Y;
            assert!(coaxial_force.dot(axis).abs() < 1e-8);
            reconstructed_force += coaxial_force;
            reconstructed_torque += (top - vector(pose.position_m())).cross(coaxial_force);
            frame_torque -= top.cross(coaxial_force);
        }
        close(
            reconstructed_force,
            vector(balance.support_force_on_upright_n),
            1e-8,
        );
        close(
            reconstructed_torque,
            vector(balance.support_torque_on_upright_nm),
            1e-8,
        );
        close(
            frame_torque,
            vector(balance.support_torque_about_frame_origin_nm),
            1e-8,
        );
    }
}

#[test]
fn invalid_configuration_rates_and_load_amplification_fail_without_changing_map() {
    for model in [dwb(), strut()] {
        assert!(model.differential(0.04, [0.; 3], reference()).is_err());
        assert!(model.differential(f64::NAN, [0.; 3], reference()).is_err());
        assert!(model.differential(0., [0.3, 0., 0.], reference()).is_err());
        let far = LinkPose::new([1., 0., 0.], [0., 0., 0., 1.]).unwrap();
        assert!(model.differential(0., [0.; 3], far).is_err());
        let map = model.differential(0., [0.; 3], reference()).unwrap();
        let before = serde_json::to_string(&map).unwrap();
        assert!(map.velocity(f64::NAN, [0.; 3]).is_err());
        assert!(map.velocity(1001., [0.; 3]).is_err());
        assert!(map.velocity(0., [0., 1001., 0.]).is_err());
        assert!(
            map.velocity(0., [0.; 3])
                .unwrap()
                .point_velocity_m_s([101., 0., 0.])
                .is_err()
        );
        let amplified = LinkLoad::new([0.; 3], [0., 1e18, 0.]).unwrap();
        assert!(map.project_load(amplified).is_err());
        assert!(map.hold_load(amplified).is_err());
        assert_eq!(serde_json::to_string(&map).unwrap(), before);
        assert_eq!(
            map.project_load(LinkLoad::new([0.; 3], [0.; 3]).unwrap())
                .unwrap()
                .travel_n,
            0.
        );
    }
    assert!(LinkLoad::new([f64::NAN, 0., 0.], [0.; 3]).is_err());
    assert!(LinkLoad::new([0.; 3], [0., f64::INFINITY, 0.]).is_err());
    assert!(LinkLoad::at_point([1e18, 0., 0.], [0., 100., 0.], reference()).is_err());
}

#[test]
fn fixed_frame_change_rotates_twists_and_shifts_support_moments_consistently() {
    let rotation = DQuat::from_scaled_axis(DVec3::new(0.2, -0.3, 0.1));
    let translation = DVec3::new(1.2, -0.5, 2.1);
    let reference = LinkPose::new(translation.to_array(), rotation.to_array()).unwrap();
    let moved_anchor = |v: [f64; 3]| (rotation * vector(v) + translation).to_array();
    let (a, b) = dwb_points();
    let moved_dwb = Mechanism::Dwb(
        FiveLink::new(
            a.map(moved_anchor),
            b,
            reference,
            (rotation * DVec3::Y).to_array(),
        )
        .unwrap(),
    );
    let (a, b) = strut_points();
    let moved_strut = Mechanism::Strut(
        StrutLink::new(
            a.map(moved_anchor),
            b,
            moved_anchor([0., 0.6, 0.]),
            [0.; 3],
            reference,
            (rotation * DVec3::Y).to_array(),
        )
        .unwrap(),
    );
    let h = 0.035;
    let rack = [0.004, 0., 0.];
    let rate = [0.02, -0.01, 0.03];
    let load = LinkLoad::new([120., 2300., -300.], [40., -15., 25.]).unwrap();
    let moved_load = LinkLoad::new(
        (rotation * vector(load.force_n())).to_array(),
        (rotation * vector(load.torque_about_upright_origin_nm())).to_array(),
    )
    .unwrap();
    for (base, moved) in [(dwb(), moved_dwb), (strut(), moved_strut)] {
        let pose = base.solve(h, rack, None);
        let map = base.differential(h, rack, pose).unwrap();
        let moved_rack = (rotation * vector(rack)).to_array();
        let pose = moved.solve(h, moved_rack, None);
        let moved_map = moved.differential(h, moved_rack, pose).unwrap();
        let velocity = map.velocity(0.17, rate).unwrap();
        let moved_velocity = moved_map
            .velocity(0.17, (rotation * vector(rate)).to_array())
            .unwrap();
        close(
            vector(moved_velocity.velocity_m_s()),
            rotation * vector(velocity.velocity_m_s()),
            1e-9,
        );
        close(
            vector(moved_velocity.angular_velocity_rad_s()),
            rotation * vector(velocity.angular_velocity_rad_s()),
            1e-9,
        );
        let forces = map.project_load(load).unwrap();
        let moved_forces = moved_map.project_load(moved_load).unwrap();
        assert!((moved_forces.travel_n - forces.travel_n).abs() < 1e-7);
        close(
            vector(moved_forces.steering_anchor_n),
            rotation * vector(forces.steering_anchor_n),
            1e-7,
        );
        let balance = map.hold_load(load).unwrap();
        let moved_balance = moved_map.hold_load(moved_load).unwrap();
        close(
            vector(moved_balance.support_force_on_frame_n),
            rotation * vector(balance.support_force_on_frame_n),
            1e-7,
        );
        let expected = rotation * vector(balance.support_torque_about_frame_origin_nm)
            + translation.cross(rotation * vector(balance.support_force_on_frame_n));
        close(
            vector(moved_balance.support_torque_about_frame_origin_nm),
            expected,
            1e-7,
        );
    }
}
