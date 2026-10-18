use asc_session::place;
use asc_test_support as f;
use asc_vehicle::data::Mode;
use glam::Mat4;
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn fixture(x: f32, duplicate: bool) -> Directory {
    let root = f::temp_path("placement");
    std::fs::create_dir(&root).unwrap();
    let car = root.join("fixture");
    std::fs::create_dir_all(car.join("data")).unwrap();
    for (name, text) in f::vehicle_files() {
        std::fs::write(car.join("data").join(name), text).unwrap();
    }
    std::fs::write(
        car.join("original.kn5"),
        f::file(f::base(&[], Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    let pose = [
        0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., x, 2., 0., 1.,
    ];
    let road = f::named(
        f::mesh(
            &[
                [-50., 0., -50.],
                [50., 0., -50.],
                [-50., 0., 50.],
                [50., 0., 50.],
            ],
            &[0, 2, 1, 1, 2, 3],
            2,
            0,
            0,
            0.,
            0,
        ),
        "1ROAD",
    );
    let cube = f::named(f::mesh(&[[0.; 3]], &[], 2, 0, 0, 0., 0), "AC_PIT_0");
    let marker = f::named(f::base(&[cube], pose, 1), "AC_PIT_0");
    let mut nodes = vec![road, marker.clone()];
    if duplicate {
        nodes.push(marker);
    }
    std::fs::write(
        root.join("original.kn5"),
        f::file(f::base(&nodes, Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=original.kn5\nPOSITION=5,3,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    std::fs::write(
        root.join("surfaces.ini"),
        "[SURFACE_0]\nKEY=ROAD\nFRICTION=.9\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n",
    )
    .unwrap();
    Directory(root)
}
fn snapshot(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
    let mut result = BTreeMap::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(dir).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                stack.push(path);
            } else {
                result.insert(
                    path.strip_prefix(root).unwrap().to_path_buf(),
                    std::fs::read(path).unwrap(),
                );
            }
        }
    }
    result
}
#[test]
fn rotated_vehicle_footprint_has_four_typed_contacts_without_moving_or_writing_content() {
    let dir = fixture(10., false);
    let before = snapshot(&dir.0);
    let result = place(
        &dir.0.join("fixture"),
        &dir.0.join("models.ini"),
        &[&dir.0.join("surfaces.ini")],
        "AC_PIT_0",
        Mode::Auto,
        Some(1),
    )
    .unwrap();
    assert!(result.all_wheels_supported);
    assert!(!result.has_errors);
    assert!(!result.simulation_ready);
    assert_eq!(result.compound, 1);
    let left = &result.contacts["front_left"];
    let rear = &result.contacts["rear_right"];
    assert!((left.origin[0] - 16.8).abs() < 1e-12);
    assert_eq!(left.origin[1], 5.);
    assert!((left.origin[2] + 0.7).abs() < 1e-12);
    assert_eq!(rear.origin, [13.8, 5., 0.75]);
    for contact in result.contacts.values() {
        let hit = contact.hit.as_ref().unwrap();
        assert_eq!(hit.position[1], 3.);
        assert_eq!(hit.distance, 2.);
        assert_eq!(hit.normal, [0., 1., 0.]);
        assert_eq!(hit.surface.friction, 0.9);
    }
    assert_eq!(snapshot(&dir.0), before);
}
#[test]
fn missing_support_is_an_error_even_with_a_valid_spawn_and_vehicle() {
    let dir = fixture(100., false);
    let result = place(
        &dir.0.join("fixture"),
        &dir.0.join("models.ini"),
        &[&dir.0.join("surfaces.ini")],
        "AC_PIT_0",
        Mode::Auto,
        None,
    )
    .unwrap();
    assert!(!result.all_wheels_supported);
    assert!(result.has_errors);
    assert!(!result.simulation_ready);
    assert_eq!(
        result
            .diagnostics
            .iter()
            .filter(|d| d.code == "placement_no_support" && d.severity == "error")
            .count(),
        4
    );
}
#[test]
fn ambiguous_or_absent_selected_spawn_is_fatal_instead_of_arbitrarily_selected() {
    for (duplicate, name) in [(true, "AC_PIT_0"), (false, "AC_PIT_99")] {
        let dir = fixture(10., duplicate);
        let error = place(
            &dir.0.join("fixture"),
            &dir.0.join("models.ini"),
            &[&dir.0.join("surfaces.ini")],
            name,
            Mode::Auto,
            None,
        )
        .err()
        .unwrap();
        assert!(error.to_string().contains("spawn_missing"));
    }
}

#[test]
fn physics_seed_uses_explicit_com_and_tensor_with_the_validated_spawn_basis() {
    let dir = fixture(10., false);
    let report = asc_scene::spawns::load(&dir.0.join("models.ini")).unwrap();
    let spawn = &report.spawns[0];
    let properties =
        asc_physics::MassProperties::new(900., [[200., 0., 0.], [0., 300., 0.], [0., 0., 400.]])
            .unwrap();
    let mut body = asc_session::body_with_spawn_basis(properties, [15., 3.8, 0.], spawn).unwrap();
    assert_eq!(body.state().position_m, [15., 3.8, 0.]);
    assert_ne!(body.state().position_m, spawn.position);
    let force = body.force_at_body_point([0., 0., 900.], [0.; 3]).unwrap();
    assert!((force.force_world_n()[0] - 900.).abs() < 1e-10);
    body.step(1. / 120., force).unwrap();
    assert!((body.state().velocity_world_m_s[0] - 1. / 120.).abs() < 1e-12);
    assert_eq!(body.state().position_m[1], 3.8);
}

#[test]
fn physics_seed_rejects_skew_nonfinite_or_reflected_public_spawn_data() {
    let dir = fixture(10., false);
    let report = asc_scene::spawns::load(&dir.0.join("models.ini")).unwrap();
    let original = &report.spawns[0];
    for change in [0, 1, 2] {
        let mut spawn = original.clone();
        match change {
            0 => spawn.left[0] += 0.0001,
            1 => spawn.left[0] = f64::NAN,
            _ => spawn.left = spawn.left.map(|v| -v),
        }
        let properties = asc_physics::MassProperties::new(
            900.,
            [[200., 0., 0.], [0., 300., 0.], [0., 0., 400.]],
        )
        .unwrap();
        assert_eq!(
            asc_session::body_with_spawn_basis(properties, [0.; 3], &spawn)
                .unwrap_err()
                .code,
            "physics_spawn_basis"
        );
    }
}
