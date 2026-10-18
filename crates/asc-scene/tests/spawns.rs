use asc_scene::spawns::{load, transform};
use asc_test_support as f;
use glam::Mat4;
use std::{io::Cursor, path::PathBuf};

const POSE: [f32; 16] = [
    0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 10., 2., 0., 1.,
];
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn marker(name: &str, children: &[Vec<u8>], matrix: [f32; 16], active: u8) -> Vec<u8> {
    f::named(f::base(children, matrix, active), name)
}
fn cube(name: &str, kind: u32) -> Vec<u8> {
    f::named(
        f::mesh(
            &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
            &[0, 1, 2],
            kind,
            0,
            0,
            100.,
            0,
        ),
        name,
    )
}
fn directory(nodes: &[Vec<u8>]) -> Directory {
    let root = f::temp_path("spawns");
    std::fs::create_dir(&root).unwrap();
    std::fs::write(
        root.join("a.kn5"),
        f::file(f::base(nodes, Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=5,3,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    Directory(root)
}
#[test]
fn node_only_reader_retains_preorder_parents_activity_kind_and_world_without_draws() {
    let parent = marker("AC_PIT_0", &[cube("AC_PIT_0", 2)], POSE, 1);
    let inactive = marker(
        "inactive",
        &[marker("AC_PIT_1", &[], POSE, 1)],
        Mat4::IDENTITY.to_cols_array(),
        0,
    );
    let bytes = f::file(
        f::base(
            &[parent, inactive, cube("standalone", 3)],
            Mat4::IDENTITY.to_cols_array(),
            1,
        ),
        5,
    );
    let nodes = asc_kn5::parse_nodes(Cursor::new(bytes.clone())).unwrap();
    assert_eq!(nodes.nodes.len(), 6);
    assert_eq!(nodes.nodes[0].parent, None);
    assert_eq!(nodes.nodes[1].parent, Some(0));
    assert_eq!(nodes.nodes[2].parent, Some(1));
    assert_eq!(nodes.nodes[3].parent, Some(0));
    assert_eq!(nodes.nodes[4].parent, Some(3));
    assert_eq!(nodes.nodes[5].parent, Some(0));
    assert_eq!(nodes.nodes[1].world, nodes.nodes[2].world);
    assert_eq!(nodes.nodes[1].world, POSE.map(f64::from));
    assert!(!nodes.nodes[4].active);
    assert_eq!(nodes.nodes[5].kind, 3);
    assert_eq!(nodes.summary.drawn_meshes, 0);
    for end in [0, 5, 10, bytes.len() - 1] {
        assert!(asc_kn5::parse_nodes(Cursor::new(&bytes[..end])).is_err());
    }
}
#[test]
fn rotated_marker_applies_model_translation_and_attached_cubes_are_one_identity() {
    for kind in [2, 3] {
        let dir = directory(&[
            marker("AC_PIT_0", &[cube("AC_PIT_0", kind)], POSE, 1),
            cube("AC_START_1", 2),
        ]);
        let report = load(&dir.0.join("models.ini")).unwrap();
        assert!(!report.has_errors);
        assert_eq!(report.spawns.len(), 2);
        let selected = &report.spawns[0];
        assert_eq!(selected.name, "AC_PIT_0");
        assert_eq!(selected.position, [15., 5., 0.]);
        assert_eq!(selected.left, [0., 0., -1.]);
        assert_eq!(selected.forward, [1., 0., 0.]);
        assert_eq!(selected.up, [0., 1., 0.]);
        assert_eq!(report.spawns[1].position, [5., 3., 0.]);
    }
}
#[test]
fn duplicate_identity_is_removed_even_at_identical_positions_and_cannot_reappear() {
    let node = marker("AC_PIT_0", &[], POSE, 1);
    let dir = directory(&[
        node.clone(),
        node.clone(),
        node,
        marker("AC_PIT_1", &[], POSE, 1),
    ]);
    let report = load(&dir.0.join("models.ini")).unwrap();
    assert!(report.has_errors);
    assert_eq!(report.spawns.len(), 1);
    assert_eq!(report.spawns[0].name, "AC_PIT_1");
    assert_eq!(
        report
            .diagnostics
            .iter()
            .filter(|d| d.code == "spawn_duplicate")
            .count(),
        2
    );
    let mut invalid = POSE;
    invalid[0] = 0.2;
    std::fs::write(
        dir.0.join("a.kn5"),
        f::file(
            f::base(
                &[
                    marker("AC_PIT_0", &[], invalid, 1),
                    marker("AC_PIT_0", &[], POSE, 1),
                ],
                Mat4::IDENTITY.to_cols_array(),
                1,
            ),
            6,
        ),
    )
    .unwrap();
    let report = load(&dir.0.join("models.ini")).unwrap();
    assert!(report.spawns.is_empty());
    for code in ["spawn_basis", "spawn_duplicate", "no_spawns"] {
        assert!(report.diagnostics.iter().any(|d| d.code == code));
    }
    let node = marker("AC_PIT_0", &[], POSE, 1);
    std::fs::write(
        dir.0.join("a.kn5"),
        f::file(f::base(&[node], Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(dir.0.join("models.ini"),"[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n[MODEL_1]\nFILE=a.kn5\nPOSITION=100,0,0\nROTATION=0,0,0\n").unwrap();
    let report = load(&dir.0.join("models.ini")).unwrap();
    assert!(report.has_errors);
    assert!(report.spawns.is_empty());
}
#[test]
fn noncanonical_names_inactive_ancestry_skinned_markers_and_bad_bases_are_explicit() {
    let mut skewed = POSE;
    skewed[0] = 0.2;
    let dir = directory(&[
        marker("AC_PIT_00", &[], POSE, 1),
        marker("ac_pit_0", &[], POSE, 1),
        marker("AC_PIT_1", &[], POSE, 0),
        marker("AC_PIT_2", &[], skewed, 1),
        cube("AC_START_1", 3),
        marker("AC_HOTLAP_START_1", &[], POSE, 1),
    ]);
    let report = load(&dir.0.join("models.ini")).unwrap();
    assert!(report.has_errors);
    assert!(report.spawns.is_empty());
    for code in ["spawn_name", "spawn_basis", "spawn_skinned", "no_spawns"] {
        assert!(report.diagnostics.iter().any(|d| d.code == code), "{code}");
    }
    for change in [0, 1, 2, 3, 4, 5] {
        let mut world = Mat4::IDENTITY.to_cols_array().map(f64::from);
        match change {
            0 => world[0] = -1.,
            1 => {
                world[5] = -1.;
                world[10] = -1.;
            }
            2 => world[3] = 0.1,
            3 => world[0] = f64::NAN,
            4 => world[12] = 10_000_001.,
            _ => world[0] = 2.,
        }
        assert_eq!(
            transform("AC_PIT_0", "original", world).unwrap_err().code,
            "spawn_basis"
        );
    }
}
#[test]
fn canonical_identity_budget_and_fatal_model_errors_do_not_yield_partial_success() {
    let nodes = (0..4097)
        .map(|i| marker(&format!("AC_PIT_{i}"), &[], POSE, 1))
        .collect::<Vec<_>>();
    let dir = directory(&nodes);
    assert_eq!(
        load(&dir.0.join("models.ini")).err().unwrap().code,
        "spawn_limit"
    );
    let long_names = (1..=1800)
        .map(|i| marker(&format!("AC_PIT_{i}{}", "9".repeat(10_000)), &[], POSE, 1))
        .collect::<Vec<_>>();
    std::fs::write(
        dir.0.join("a.kn5"),
        f::file(f::base(&long_names, Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    let error = load(&dir.0.join("models.ini")).err().unwrap();
    assert_eq!(error.code, "spawn_limit");
    assert!(error.message.contains("metadata"));
    std::fs::write(dir.0.join("a.kn5"), b"bad").unwrap();
    assert_eq!(
        load(&dir.0.join("models.ini")).err().unwrap().code,
        "spawn_model"
    );
    std::fs::write(
        dir.0.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,90,0\n",
    )
    .unwrap();
    assert_eq!(
        load(&dir.0.join("models.ini")).err().unwrap().code,
        "model_rotation_unsupported"
    );
}
