use asc_physics::{Body, MassProperties, NormalContact, WheelKinematics, WheelState};
use asc_session::face_contact;
use asc_test_support as f;
use glam::{DQuat, DVec3, Mat4};
use std::path::PathBuf;

struct Fixture {
    root: PathBuf,
    ground: asc_ground::Ground,
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.root);
    }
}
fn fixture(triangles: &[[[f32; 3]; 3]]) -> Fixture {
    let root = f::temp_path("face-contact");
    std::fs::create_dir(&root).unwrap();
    let nodes: Vec<_> = triangles
        .iter()
        .map(|p| f::named(f::mesh(p, &[0, 1, 2], 2, 0, 0, 0., 0), "1ROAD"))
        .collect();
    std::fs::write(
        root.join("a.kn5"),
        f::file(f::base(&nodes, Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    std::fs::write(
        root.join("surfaces.ini"),
        "[SURFACE_0]\nKEY=ROAD\nFRICTION=0.83\nDAMPING=0.02\nIS_VALID_TRACK=1\nIS_PITLANE=0\n",
    )
    .unwrap();
    let ground = asc_ground::load(&root.join("models.ini"), &[&root.join("surfaces.ini")]).unwrap();
    Fixture { root, ground }
}
fn triangle(y: f32) -> [[f32; 3]; 3] {
    [[0., y, 0.], [4., y, 0.], [0., y, 4.]]
}
fn wheel(p: [f64; 3], q: DQuat) -> WheelKinematics {
    WheelKinematics {
        state: WheelState {
            angle_rad: 0.,
            rate_rad_s: 0.,
        },
        axis_world: (q * DVec3::X).to_array(),
        body: Body::new(
            MassProperties::new(20., [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]).unwrap(),
            p,
            q.to_array(),
            [0.; 3],
            [0.; 3],
        )
        .unwrap()
        .state(),
    }
}
fn law() -> NormalContact {
    NormalContact::new(0.3, 200000., 1000., 0.05).unwrap()
}

#[test]
fn selected_surface_scales_friction_and_missing_face_supplies_no_tangential_load() {
    let f = fixture(&[triangle(0.)]);
    let friction = asc_physics::TangentialContact::new(10000., 10000., 0.5, 1.).unwrap();
    let mut w = wheel([1., 0.29, 1.], DQuat::IDENTITY);
    w.body.velocity_world_m_s = [0., 0., 10.];
    let c = asc_session::face_contact_with_friction(&f.ground, &w, &law(), 0.5, Some(&friction))
        .unwrap()
        .unwrap();
    let t = c.tangential.unwrap();
    assert!((t.longitudinal_force_n + 830.).abs() < 1e-8);
    assert_eq!(t.surface_friction_scale, 0.83);
    w.body.position_m = [3., 0.29, 3.];
    assert!(
        asc_session::face_contact_with_friction(&f.ground, &w, &law(), 0.5, Some(&friction))
            .unwrap()
            .is_none()
    );
}
#[test]
fn finite_edges_airborne_and_seam_have_no_infinite_or_double_support() {
    let f = fixture(&[triangle(0.), [[4., 0., 0.], [4., 0., 4.], [0., 0., 4.]]]);
    for (p, id, force) in [
        ([1., 0.29, 1.], 0, 2000.),
        ([2., 0.29, 2.], 0, 2000.),
        ([3., 0.29, 3.], 1, 2000.),
        ([1., 0.32, 1.], 0, 0.),
    ] {
        let c = face_contact(&f.ground, &wheel(p, DQuat::IDENTITY), &law(), 0.5)
            .unwrap()
            .unwrap();
        assert_eq!(c.face.triangle, id);
        assert_eq!(c.face.surface_key, "ROAD");
        assert_eq!(c.face.surface_friction, 0.83);
        assert!((c.sample.normal_force_n - force).abs() < 1e-8);
    }
    for p in [[4.01, 0.29, 1.], [-0.01, 0.29, 1.], [1., 2., 1.]] {
        assert!(
            face_contact(&f.ground, &wheel(p, DQuat::IDENTITY), &law(), 0.5)
                .unwrap()
                .is_none()
        );
    }
}
#[test]
fn only_finite_front_faces_enforce_penetration_and_overhead_decks_are_ignored() {
    let f = fixture(&[triangle(0.), triangle(0.75)]);
    let c = face_contact(
        &f.ground,
        &wheel([1., 0.29, 1.], DQuat::IDENTITY),
        &law(),
        0.5,
    )
    .unwrap()
    .unwrap();
    assert_eq!(c.face.triangle, 0);
    assert_eq!(
        face_contact(
            &f.ground,
            &wheel([1., 0.2, 1.], DQuat::IDENTITY),
            &law(),
            0.5
        )
        .unwrap_err()
        .code,
        "physics_contact_penetration"
    );
    assert_eq!(
        face_contact(
            &f.ground,
            &wheel([1., -0.1, 1.], DQuat::IDENTITY),
            &law(),
            0.5
        )
        .unwrap_err()
        .code,
        "track_contact_backside"
    );
    // Circle can intersect an infinite plane extension without intersecting the finite face.
    assert!(
        face_contact(
            &f.ground,
            &wheel([3., 0.1, 3.], DQuat::IDENTITY),
            &law(),
            0.5
        )
        .unwrap()
        .is_none()
    );
    assert!(
        face_contact(
            &f.ground,
            &wheel([1., -1., 1.], DQuat::IDENTITY),
            &law(),
            0.5
        )
        .unwrap()
        .is_none()
    );
}
#[test]
fn sloped_cambered_contact_matches_analytic_circle_and_filters_steep_faces() {
    let f = fixture(&[[[0., 0., 0.], [4., 2., 0.], [0., 0., 4.]]]);
    let n = DVec3::new(-0.5, 1., 0.).normalize();
    let q = DQuat::from_rotation_z(0.15);
    let axle = q * DVec3::X;
    let radial = n - axle * axle.dot(n);
    let extent = 0.3 * radial.length();
    let center = DVec3::new(1., 0.5, 1.) + n * (extent - 0.01);
    let c = face_contact(&f.ground, &wheel(center.to_array(), q), &law(), 0.5)
        .unwrap()
        .unwrap();
    assert!((c.sample.gap_m + 0.01).abs() < 1e-12);
    assert!((DVec3::from_array(c.sample.force_world_n) - 2000. * n).length() < 1e-8);
    let lever = -0.3 * radial.normalize();
    assert!(
        (DVec3::from_array(c.sample.torque_about_wheel_com_world_nm) - lever.cross(2000. * n))
            .length()
            < 1e-8
    );
    assert!(
        face_contact(&f.ground, &wheel(center.to_array(), q), &law(), 0.95)
            .unwrap()
            .is_none()
    );
    assert_eq!(
        face_contact(&f.ground, &wheel(center.to_array(), q), &law(), 0.)
            .unwrap_err()
            .code,
        "track_contact_normal"
    );
}
