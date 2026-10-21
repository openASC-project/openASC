use asc_physics::{
    Body, MassProperties, NormalContact, SupportPlane, TangentialContact, WheelKinematics,
    WheelState,
};
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
fn spinning_wheel_produces_traction_and_opposite_axle_torque_with_exact_power() {
    let normal = NormalContact::new(0.3, 200000., 0., 0.05).unwrap();
    let friction = TangentialContact::new(1000., 1500., 0.8, 1.).unwrap();
    for sign in [-1., 1.] {
        let w = wheel(
            DVec3::new(0., 0.29, 0.),
            DQuat::IDENTITY,
            DVec3::ZERO,
            DVec3::X * sign,
        );
        let c = normal
            .sample_with_friction(&w, plane(), &friction, 1.)
            .unwrap();
        close(c.tangential.force_world_n, DVec3::Z * (300. * sign), 1e-8);
        close(
            c.tangential.torque_about_wheel_com_world_nm,
            DVec3::X * (-90. * sign),
            1e-8,
        );
        near(c.tangential.dissipated_power_w, 90., 1e-8);
        let (force, torque) = (
            DVec3::from_array(c.tangential.force_world_n),
            DVec3::from_array(c.tangential.torque_about_wheel_com_world_nm),
        );
        near(
            force.dot(DVec3::from_array(w.body.velocity_world_m_s))
                + torque.dot(DVec3::from_array(w.body.angular_velocity_world_rad_s)),
            c.tangential.mechanical_power_w,
            1e-10,
        );
        let rolling = wheel(
            DVec3::new(0., 0.29, 0.),
            DQuat::IDENTITY,
            DVec3::Z * (0.3 * sign),
            DVec3::X * sign,
        );
        let c = normal
            .sample_with_friction(&rolling, plane(), &friction, 1.)
            .unwrap();
        close(c.tangential.force_world_n, DVec3::ZERO, 1e-10);
        near(c.tangential.dissipated_power_w, 0., 1e-10);
    }
}

#[test]
fn combined_sliding_uses_one_ellipse_and_never_generates_energy() {
    let normal = NormalContact::new(0.3, 200000., 0., 0.05).unwrap();
    let friction = TangentialContact::new(1000., 2000., 0.5, 1.).unwrap();
    for vx in [-4., -0.1, 0., 0.1, 4.] {
        for vz in [-4., -0.1, 0., 0.1, 4.] {
            let w = wheel(
                DVec3::new(0., 0.29, 0.),
                DQuat::IDENTITY,
                DVec3::new(vx, 0., vz),
                DVec3::ZERO,
            );
            let c = normal
                .sample_with_friction(&w, plane(), &friction, 0.8)
                .unwrap();
            let t = c.tangential;
            near(t.longitudinal_limit_n, 800., 1e-8);
            near(t.lateral_limit_n, 1600., 1e-8);
            let ratio = (1000. * vz / 800.).hypot(2000. * vx / 1600.);
            let scale = 1. / ratio.max(1.);
            near(t.longitudinal_force_n, -1000. * vz * scale, 1e-8);
            near(t.lateral_force_n, -2000. * vx * scale, 1e-8);
            assert!((t.longitudinal_force_n / 800.).hypot(t.lateral_force_n / 1600.) <= 1. + 1e-12);
            let power = DVec3::from_array(t.force_world_n).dot(DVec3::new(vx, 0., vz));
            near(power, t.mechanical_power_w, 1e-8);
            near(-power, t.dissipated_power_w, 1e-8);
            assert!(power <= 0.);
        }
    }
}

#[test]
fn cambered_contact_wrench_is_covariant_and_uses_material_point_not_projection() {
    let normal = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let friction = TangentialContact::new(1000., 1500., 0.8, 1.).unwrap();
    let q = DQuat::from_rotation_z(0.2);
    let v = DVec3::new(0.2, -0.1, 0.7);
    let omega = DVec3::new(3., 0.4, -0.2);
    let p = DVec3::new(1., 0.3 * 0.2f64.cos() - 0.01, 2.);
    let w = wheel(p, q, v, omega);
    let c = normal
        .sample_with_friction(&w, plane(), &friction, 0.9)
        .unwrap();
    let t = &c.tangential;
    let force = DVec3::from_array(t.force_world_n);
    let torque = DVec3::from_array(t.torque_about_wheel_com_world_nm);
    let lever = DVec3::from_array(c.normal.wheel_point_world_m) - p;
    close(t.torque_about_wheel_com_world_nm, lever.cross(force), 1e-9);
    near(force.dot(v) + torque.dot(omega), t.mechanical_power_w, 1e-9);
    let projected_lever = DVec3::from_array(c.normal.plane_point_world_m) - p;
    assert!((projected_lever.cross(force) - torque).length() > 0.1);
    let r = DQuat::from_euler(glam::EulerRot::XYZ, 0.3, -0.6, 0.7);
    let shift = DVec3::new(1000., -2000., 3000.);
    let transformed = wheel(r * p + shift, r * q, r * v, r * omega);
    let c2 = normal
        .sample_with_friction(
            &transformed,
            SupportPlane::new(shift.to_array(), (r * DVec3::Y).to_array()).unwrap(),
            &friction,
            0.9,
        )
        .unwrap();
    close(c2.tangential.force_world_n, r * force, 1e-7);
    close(
        c2.tangential.torque_about_wheel_com_world_nm,
        r * torque,
        1e-7,
    );
    near(c2.tangential.dissipated_power_w, t.dissipated_power_w, 1e-7);
}

#[test]
fn no_normal_load_zero_friction_disabled_axes_and_bad_parameters_are_explicit() {
    let normal = NormalContact::new(0.3, 200000., 1000., 0.05).unwrap();
    let friction = TangentialContact::new(1000., 1000., 1., 1.).unwrap();
    for (height, vy, scale) in [(0.32, 0., 1.), (0.29, 10., 1.), (0.29, 0., 0.)] {
        let c = normal
            .sample_with_friction(
                &wheel(
                    DVec3::new(0., height, 0.),
                    DQuat::IDENTITY,
                    DVec3::new(1., vy, 1.),
                    DVec3::X,
                ),
                plane(),
                &friction,
                scale,
            )
            .unwrap();
        close(c.tangential.force_world_n, DVec3::ZERO, 1e-10);
        near(c.tangential.dissipated_power_w, 0., 1e-10);
    }
    let w = wheel(
        DVec3::new(0., 0.29, 0.),
        DQuat::IDENTITY,
        DVec3::new(1., 0., 1.),
        DVec3::ZERO,
    );
    let disabled = TangentialContact::new(1000., 1000., 0., 1.).unwrap();
    let c = normal
        .sample_with_friction(&w, plane(), &disabled, 1.)
        .unwrap();
    near(c.tangential.longitudinal_force_n, 0., 1e-10);
    near(c.tangential.lateral_force_n, -1000., 1e-8);
    for values in [
        [-1., 1000., 1., 1.],
        [1e8, 1000., 1., 1.],
        [1000., 1000., -1., 1.],
        [1000., 1000., 1., 11.],
        [f64::NAN, 1000., 1., 1.],
    ] {
        assert!(TangentialContact::new(values[0], values[1], values[2], values[3]).is_err());
    }
    for scale in [-1., 11., f64::NAN] {
        assert_eq!(
            normal
                .sample_with_friction(&w, plane(), &friction, scale)
                .unwrap_err()
                .code,
            "physics_tangential_contact"
        );
    }
}
