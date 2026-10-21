use asc_physics::{Body, MassProperties, NormalContact, SupportPlane, WheelKinematics, WheelState};
use glam::{DQuat, DVec3};
fn wheel(center: DVec3, q: DQuat, v: DVec3, w: DVec3) -> WheelKinematics {
    WheelKinematics {
        state: WheelState {
            angle_rad: 0.,
            rate_rad_s: 0.,
        },
        axis_world: (q * DVec3::X).to_array(),
        body: Body::new(
            MassProperties::new(20., [[1.2, 0., 0.], [0., 0.7, 0.], [0., 0., 0.7]]).unwrap(),
            center.to_array(),
            q.to_array(),
            v.to_array(),
            w.to_array(),
        )
        .unwrap()
        .state(),
    }
}
fn near(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() < tol, "{a} != {b}; tolerance {tol}");
}
fn close(a: [f64; 3], b: DVec3, tol: f64) {
    assert!((DVec3::from_array(a) - b).length() < tol, "{a:?} != {b:?}");
}
fn plane() -> SupportPlane {
    SupportPlane::new([0.; 3], [0., 1., 0.]).unwrap()
}

#[test]
fn unilateral_normal_load_and_clamped_damping_obey_mechanical_power_balance() {
    let law = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    for (speed, force, dissipation) in [
        (-0.2, 2200., 40.),
        (0., 2000., 0.),
        (0.5, 1500., 250.),
        (4., 0., 8000.),
    ] {
        let s = law
            .sample(
                &wheel(
                    DVec3::new(0., 0.29, 0.),
                    DQuat::IDENTITY,
                    DVec3::new(0., speed, 0.),
                    DVec3::ZERO,
                ),
                plane(),
            )
            .unwrap();
        near(s.gap_m, -0.01, 1e-12);
        near(s.penetration_m, 0.01, 1e-12);
        near(s.normal_force_n, force, 1e-8);
        near(s.dissipated_power_w, dissipation, 1e-8);
        near(
            s.mechanical_power_w - 200000. * s.penetration_m * speed + s.dissipated_power_w,
            0.,
            1e-8,
        );
        close(s.torque_about_wheel_com_world_nm, DVec3::ZERO, 1e-10);
        assert!(s.geometrically_touching);
        near(s.potential_energy_j, 10., 1e-10);
    }
    let separated = law
        .sample(
            &wheel(
                DVec3::new(0., 0.32, 0.),
                DQuat::IDENTITY,
                DVec3::new(0., -10., 0.),
                DVec3::ZERO,
            ),
            plane(),
        )
        .unwrap();
    assert!(!separated.geometrically_touching);
    assert_eq!(separated.normal_force_n, 0.);
    assert_eq!(separated.dissipated_power_w, 0.);
    let touch = law
        .sample(
            &wheel(
                DVec3::new(0., 0.3, 0.),
                DQuat::IDENTITY,
                DVec3::new(0., -0.2, 0.),
                DVec3::ZERO,
            ),
            plane(),
        )
        .unwrap();
    near(touch.normal_force_n, 200., 1e-8);
}

#[test]
fn cambered_circle_support_geometry_generates_no_axle_torque_or_spin_normal_damping() {
    let law = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let camber = 0.2_f64;
    let q = DQuat::from_rotation_z(camber);
    let center = DVec3::new(0., 0.3 * camber.cos() - 0.01, 0.);
    let axis = q * DVec3::X;
    let a = law
        .sample(&wheel(center, q, DVec3::ZERO, DVec3::ZERO), plane())
        .unwrap();
    let b = law
        .sample(&wheel(center, q, DVec3::ZERO, axis * 100.), plane())
        .unwrap();
    near(a.normal_force_n, 2000., 1e-8);
    near(b.normal_force_n, a.normal_force_n, 1e-8);
    close(
        a.wheel_point_world_m,
        DVec3::new(0.3 * camber.sin(), -0.01, 0.),
        1e-12,
    );
    close(
        a.plane_point_world_m,
        DVec3::new(0.3 * camber.sin(), 0., 0.),
        1e-12,
    );
    close(
        a.torque_about_wheel_com_world_nm,
        DVec3::new(0., 0., 2000. * 0.3 * camber.sin()),
        1e-8,
    );
    near(
        DVec3::from_array(a.torque_about_wheel_com_world_nm).dot(axis),
        0.,
        1e-10,
    );
    near(b.normal_velocity_m_s, 0., 1e-10);
    near(b.dissipated_power_w, 0., 1e-10);
}

#[test]
fn gap_velocity_and_elastic_wrench_match_independent_geometric_energy_differences() {
    let law = NormalContact::new(0.3, 200000., 0., 0.05).unwrap();
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.2, -0.1, 0.3);
    let normal = DVec3::new(0.1, 1., 0.2).normalize();
    let p = SupportPlane::new([0.; 3], normal.to_array()).unwrap();
    let mut center = DVec3::new(0., 0.3, 0.);
    let initial = law
        .sample(&wheel(center, q, DVec3::ZERO, DVec3::ZERO), p)
        .unwrap();
    center -= normal * (initial.gap_m + 0.01);
    let v = DVec3::new(0.02, -0.01, 0.03);
    let w = DVec3::new(0.1, -0.2, 0.3);
    let sample = law.sample(&wheel(center, q, v, w), p).unwrap();
    let eps = 1e-5;
    let values = [-eps, eps].map(|t| {
        let rotation = DQuat::from_scaled_axis(w * t) * q;
        let axis = rotation * DVec3::X;
        let radial = (normal - axis * axis.dot(normal)).normalize();
        let point = center + v * t - 0.3 * radial;
        let gap = normal.dot(point);
        (gap, 100000. * gap.powi(2))
    });
    near(
        sample.normal_velocity_m_s,
        (values[1].0 - values[0].0) / (2. * eps),
        1e-9,
    );
    let power = DVec3::from_array(sample.force_world_n).dot(v)
        + DVec3::from_array(sample.torque_about_wheel_com_world_nm).dot(w);
    near(power, -(values[1].1 - values[0].1) / (2. * eps), 1e-6);
}

#[test]
fn normal_contact_is_covariant_and_rejects_degenerate_or_excessive_penetration() {
    let law = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let q = DQuat::from_rotation_z(0.2);
    let center = DVec3::new(0., 0.3 * 0.2_f64.cos() - 0.01, 0.);
    let v = DVec3::new(0.1, -0.2, 0.3);
    let w = DVec3::new(1., 0.2, -0.3);
    let first = law.sample(&wheel(center, q, v, w), plane()).unwrap();
    let r = DQuat::from_euler(glam::EulerRot::XYZ, -0.3, 0.1, 0.4);
    let t = DVec3::new(-2., 3., 1.);
    let second = law
        .sample(
            &wheel(t + r * center, r * q, r * v, r * w),
            SupportPlane::new(t.to_array(), (r * DVec3::Y).to_array()).unwrap(),
        )
        .unwrap();
    close(
        second.force_world_n,
        r * DVec3::from_array(first.force_world_n),
        1e-7,
    );
    close(
        second.torque_about_wheel_com_world_nm,
        r * DVec3::from_array(first.torque_about_wheel_com_world_nm),
        1e-7,
    );
    near(first.dissipated_power_w, second.dissipated_power_w, 1e-7);
    assert!(
        law.sample(
            &wheel(
                DVec3::new(0., 0.2, 0.),
                DQuat::IDENTITY,
                DVec3::ZERO,
                DVec3::ZERO
            ),
            plane()
        )
        .is_err()
    );
    assert!(
        law.sample(
            &wheel(
                DVec3::new(0., 0.3, 0.),
                DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
                DVec3::ZERO,
                DVec3::ZERO
            ),
            plane()
        )
        .is_err()
    );
    for values in [
        (0., 200000., 1000., 0.05),
        (0.3, -1., 1000., 0.05),
        (0.3, 200000., -1., 0.05),
        (0.3, 200000., 1000., 0.2),
    ] {
        assert!(NormalContact::new(values.0, values.1, values.2, values.3).is_err());
    }
    assert!(SupportPlane::new([0.; 3], [0.; 3]).is_err());
    assert!(SupportPlane::new([f64::NAN, 0., 0.], [0., 1., 0.]).is_err());
}
