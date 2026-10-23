use asc_session::ColliderBox;
use asc_test_support as f;
use asc_vehicle::data::Mode;
use glam::{DQuat, DVec3};
use std::path::PathBuf;
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn fixture() -> Directory {
    let root = f::temp_path("collider-proxy");
    std::fs::create_dir_all(root.join("data")).unwrap();
    for (name, text) in f::vehicle_files() {
        std::fs::write(root.join("data").join(name), text).unwrap();
    }
    for name in ["original.kn5", "collider.kn5"] {
        std::fs::write(root.join(name), f::collider_tetrahedron_file()).unwrap();
    }
    Directory(root)
}
#[test]
fn collider_proxy_composes_explicit_model_graphics_and_world_frames_using_only_indexed_geometry() {
    let d = fixture();
    let before = std::fs::read(d.0.join("collider.kn5")).unwrap();
    let origin = DVec3::new(0.1, 0.2, 0.3);
    let model = DQuat::from_rotation_y(0.2);
    let body = ColliderBox::load(
        &d.0,
        Mode::Unpacked,
        None,
        origin.to_array(),
        model.to_array(),
    )
    .unwrap();
    let mut low = DVec3::splat(f64::INFINITY);
    let mut high = DVec3::splat(f64::NEG_INFINITY);
    for p in [
        [-0.5, 0., -1.],
        [0.5, 0., -1.],
        [0., 1., -1.],
        [0., 0.5, 1.],
    ] {
        let p = origin
            + model
                * (DVec3::new(0., -0.3, 0.1)
                    + DQuat::from_rotation_x(0.5f64.to_radians()) * DVec3::from_array(p));
        low = low.min(p);
        high = high.max(p);
    }
    assert!((DVec3::from_array(body.center_in_chassis_m) - (low + high) * 0.5).length() < 1e-10);
    assert!((DVec3::from_array(body.half_extent_m) - (high - low) * 0.5).length() < 1e-10);
    assert_eq!(body.indexed_vertex_references, 12);
    let q = DQuat::from_rotation_x(0.4);
    let shift = DVec3::new(5., 2., 3.);
    let placed = body.at_pose(shift.to_array(), q.to_array()).unwrap();
    let (center, _) = placed.aabb(0.).unwrap();
    assert!(
        (DVec3::from_array(center) - (shift + q * DVec3::from_array(body.center_in_chassis_m)))
            .length()
            < 1e-10
    );
    assert_eq!(std::fs::read(d.0.join("collider.kn5")).unwrap(), before);
}
#[test]
fn collider_proxy_rejects_invalid_mappings_flat_hidden_and_oversized_inputs() {
    let d = fixture();
    assert!(ColliderBox::load(&d.0, Mode::Unpacked, None, [0.; 3], [0.; 4]).is_err());
    assert!(
        ColliderBox::load(&d.0, Mode::Unpacked, None, [30., 0., 0.], [0., 0., 0., 1.]).is_err()
    );
    for visible in [0, 1] {
        let flat = f::file(
            f::base(
                &[f::mesh(
                    &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
                    &[0, 1, 2],
                    2,
                    visible,
                    visible,
                    0.,
                    0,
                )],
                glam::Mat4::IDENTITY.to_cols_array(),
                1,
            ),
            6,
        );
        std::fs::write(d.0.join("collider.kn5"), flat).unwrap();
        assert!(ColliderBox::load(&d.0, Mode::Unpacked, None, [0.; 3], [0., 0., 0., 1.]).is_err());
    }
    let skinned = f::file(
        f::base(
            &[f::mesh(
                &[[0., 0., 0.], [1., 0., 0.], [0., 1., 1.]],
                &[0, 1, 2],
                3,
                1,
                1,
                0.,
                0,
            )],
            glam::Mat4::IDENTITY.to_cols_array(),
            1,
        ),
        6,
    );
    std::fs::write(d.0.join("collider.kn5"), skinned).unwrap();
    assert!(
        ColliderBox::load(&d.0, Mode::Unpacked, None, [0.; 3], [0., 0., 0., 1.])
            .unwrap_err()
            .to_string()
            .contains("skinned")
    );
    std::fs::write(d.0.join("collider.kn5"), vec![0; 4 * 1024 * 1024 + 1]).unwrap();
    assert!(
        ColliderBox::load(&d.0, Mode::Unpacked, None, [0.; 3], [0., 0., 0., 1.])
            .unwrap_err()
            .to_string()
            .contains("4 MiB")
    );
}
