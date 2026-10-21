use asc_ground::{load, surfaces};
use asc_test_support as fixture;
use glam::{Mat4, Vec3};
use std::{
    io::Cursor,
    path::{Path, PathBuf},
};

struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn definition(key: &str, n: usize, friction: &str) -> String {
    format!(
        "[SURFACE_{n}]\nKEY={key}\nFRICTION={friction}\nDAMPING=0.02\nIS_VALID_TRACK=1\nIS_PITLANE=0\n"
    )
}
fn named_mesh(
    name: &str,
    points: &[[f32; 3]],
    visible: u8,
    renderable: u8,
    lod: f32,
    kind: u32,
) -> Vec<u8> {
    let original = fixture::mesh(points, &[0, 1, 2], kind, visible, renderable, lod, 0);
    let mut out = original[..4].to_vec();
    fixture::string(&mut out, name);
    out.extend(&original[8 + "original_mesh".len()..]);
    out
}
fn triangle(name: &str, y: f32) -> Vec<u8> {
    named_mesh(name, &[[0., y, 0.], [4., y, 0.], [0., y, 4.]], 0, 0, 0., 2)
}
fn directory(nodes: &[Vec<u8>]) -> Directory {
    let root = fixture::temp_path("ground");
    std::fs::create_dir(&root).unwrap();
    std::fs::write(
        root.join("a.kn5"),
        fixture::file(fixture::base(nodes, Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    std::fs::write(root.join("surfaces.ini"), definition("ROAD", 0, "0.83")).unwrap();
    Directory(root)
}
fn ground(root: &Path) -> asc_ground::Ground {
    load(&root.join("models.ini"), &[&root.join("surfaces.ini")]).unwrap()
}
fn near(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-8, "{a} != {b}");
}

#[test]
fn finite_face_queries_clip_edges_and_plane_in_both_windings() {
    for points in [
        [[0., 0., 0.], [4., 2., 0.], [0., 0., 4.]],
        [[0., 0., 0.], [0., 0., 4.], [4., 2., 0.]],
    ] {
        let d = directory(&[named_mesh("1ROAD", &points, 0, 0, 0., 2)]);
        let g = ground(&d.0);
        let faces = g.faces_in_box([1., 0.5, 1.], [0.1; 3]).unwrap();
        assert_eq!(faces.len(), 1);
        let f = &faces[0];
        assert_eq!((f.triangle, f.mesh, f.model), (0, "1ROAD", "a.kn5"));
        assert_eq!(f.surface.key, "ROAD");
        near(f.normal()[1], 1. / 1.25f64.sqrt());
        for p in [[1., 0.5, 1.], [4., 2., 0.], [2., 1., 2.]] {
            assert!(f.contains_point(p).unwrap());
        }
        for p in [[3., 1.5, 3.], [-0.001, -0.0005, 0.], [1., 0.501, 1.]] {
            assert!(!f.contains_point(p).unwrap());
        }
        assert!(g.faces_in_box([1., 5., 1.], [0.1; 3]).unwrap().is_empty());
        assert!(g.faces_in_box([f64::NAN, 0., 0.], [1.; 3]).is_err());
        assert!(g.faces_in_box([0.; 3], [20.0001; 3]).is_err());
        assert!(f.contains_point([f64::INFINITY; 3]).is_err());
    }
}

#[test]
fn face_query_density_cap_errors_instead_of_dropping_candidates() {
    for count in [4096, 4097] {
        let d = directory(&vec![triangle("1ROAD", 0.); count]);
        let g = ground(&d.0);
        let result = g.faces_in_box([1., 0., 1.], [0.3; 3]);
        if count == 4096 {
            let faces = result.unwrap();
            assert_eq!(
                faces.iter().map(|f| f.triangle).collect::<Vec<_>>(),
                (0..count).collect::<Vec<_>>()
            );
        } else {
            assert_eq!(result.unwrap_err().code, "ground_face_limit");
        }
    }
}

#[test]
fn hidden_physical_geometry_keeps_double_precision_and_active_ancestry() {
    let mut inactive = triangle("2ROAD_inactive", 0.);
    inactive[12 + "2ROAD_inactive".len()] = 0;
    let root = fixture::base(
        &[
            triangle("1ROAD", 0.),
            triangle("0ROAD", 0.),
            triangle("000ROAD", 0.),
            triangle("ROAD", 0.),
            inactive,
            named_mesh("3ROAD_skin", &[[0.; 3]; 3], 1, 1, 0., 3),
        ],
        Mat4::IDENTITY.to_cols_array(),
        1,
    );
    let collision = asc_kn5::parse_collision(Cursor::new(fixture::file(root, 5))).unwrap();
    assert_eq!(collision.meshes.len(), 1);
    assert_eq!(collision.meshes[0].name, "1ROAD");
    assert_eq!(collision.summary.drawn_meshes, 0);
    assert_eq!(collision.summary.skinned_meshes, 1);
    let matrix = Mat4::from_translation(Vec3::new(1_000_000., 0., 0.)).to_cols_array();
    let node = named_mesh(
        "1ROAD",
        &[[0.01, 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        1,
        1,
        0.,
        2,
    );
    let bytes = fixture::file(fixture::base(&[node], matrix, 1), 6);
    let physical = asc_kn5::parse_collision(Cursor::new(&bytes)).unwrap();
    let render = asc_kn5::parse(Cursor::new(bytes)).unwrap();
    near(
        physical.meshes[0].positions[0][0],
        1_000_000. + f64::from(0.01f32),
    );
    assert_ne!(
        physical.meshes[0].positions[0][0],
        f64::from(render.vertices[0].position[0])
    );
    let inactive = fixture::file(fixture::base(&[triangle("1ROAD", 0.)], matrix, 0), 6);
    assert!(
        asc_kn5::parse_collision(Cursor::new(inactive))
            .unwrap()
            .meshes
            .is_empty()
    );
}

#[test]
fn ramp_winding_edges_distance_bridge_and_repeatable_ties() {
    for points in [
        [[0., 0., 0.], [2., 2., 0.], [0., 0., 2.]],
        [[0., 0., 2.], [2., 2., 0.], [0., 0., 0.]],
    ] {
        let dir = directory(&[named_mesh("1ROAD", &points, 0, 0, 0., 2)]);
        let ground = ground(&dir.0);
        let hit = ground.probe([0.5, 3., 0.5], 10.).unwrap().unwrap();
        near(hit.position[1], 0.5);
        near(hit.distance, 2.5);
        near(hit.normal[0], -0.5f64.sqrt());
        near(hit.normal[1], 0.5f64.sqrt());
        near(hit.normal[2], 0.);
        assert!(ground.probe([1., 1., 1.], 0.).unwrap().is_some());
        for (p, d) in [
            ([1.1, 4., 1.1], 10.),
            ([0.5, 0.4, 0.5], 10.),
            ([0.5, 3., 0.5], 2.4),
        ] {
            assert!(ground.probe(p, d).unwrap().is_none());
        }
    }
    let dir = directory(&[
        triangle("1ROAD_low", 0.),
        triangle("2ROAD_bridge", 4.),
        triangle("3ROAD_tie", 4.),
    ]);
    let ground = ground(&dir.0);
    assert_eq!(
        ground.probe([1., 5., 1.], 10.).unwrap().unwrap().mesh,
        "2ROAD_bridge"
    );
    near(
        ground.probe([1., 3., 1.], 10.).unwrap().unwrap().position[1],
        0.,
    );
    near(
        ground.probe([1., 4., 1.], 0.).unwrap().unwrap().distance,
        0.,
    );
    for origin in [
        [f64::NAN, 0., 0.],
        [0., f64::INFINITY, 0.],
        [10_000_001., 0., 0.],
    ] {
        assert_eq!(
            ground.probe(origin, 10.).unwrap_err().code,
            "ground_query_range"
        );
    }
    for distance in [-1., f64::NAN, 10_000_001.] {
        assert_eq!(
            ground.probe([0.; 3], distance).unwrap_err().code,
            "ground_query_range"
        );
    }
}

#[test]
fn median_bvh_matches_analytic_plane_across_many_leaves_and_both_model_transforms() {
    let mut nodes = Vec::new();
    for x in 0..12 {
        for z in 0..12 {
            let point = |x: i32, z: i32| [x as f32, (x + 2 * z) as f32 * 0.25, z as f32];
            nodes.push(named_mesh(
                "1ROAD",
                &[point(x, z), point(x + 1, z), point(x, z + 1)],
                0,
                0,
                0.,
                2,
            ));
            nodes.push(named_mesh(
                "1ROAD",
                &[point(x + 1, z + 1), point(x, z + 1), point(x + 1, z)],
                0,
                0,
                0.,
                2,
            ));
        }
    }
    let dir = directory(&nodes);
    let bytes = fixture::file(
        fixture::base(
            &nodes,
            Mat4::from_translation(Vec3::new(20., 2., -5.)).to_cols_array(),
            1,
        ),
        6,
    );
    std::fs::write(dir.0.join("b.kn5"), bytes).unwrap();
    std::fs::write(dir.0.join("models.ini"),"[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n[MODEL_1]\nFILE=b.kn5\nPOSITION=10,3,2\nROTATION=0,0,0\n").unwrap();
    let ground = ground(&dir.0);
    assert_eq!(ground.summary()["triangles"], 576);
    for dx in [0., 30.] {
        for x in 0..24 {
            for z in 0..24 {
                let px = x as f64 * 0.5;
                let pz = z as f64 * 0.5;
                let shift = if dx == 0. { 0. } else { 5. };
                let hit = ground
                    .probe([px + dx, 100., pz - if dx == 0. { 0. } else { 3. }], 100.)
                    .unwrap()
                    .unwrap();
                near(hit.position[1], px * 0.25 + pz * 0.5 + shift);
                near(hit.normal[0], -0.25 / 1.3125f64.sqrt());
                near(hit.normal[1], 1. / 1.3125f64.sqrt());
            }
        }
    }
}

#[test]
fn surfaces_use_complete_overrides_explicit_flags_and_diagnostics() {
    let dir = directory(&[]);
    let defaults = dir.0.join("defaults.ini");
    let selected = dir.0.join("surfaces.ini");
    std::fs::write(
        &defaults,
        definition("ROAD", 0, "0.7") + &definition("GRASS", 1, "0.5"),
    )
    .unwrap();
    std::fs::write(
        &selected,
        definition("ROAD", 0, "0.9")
            + "SIN_HEIGHT=0.01\nWAV=custom.wav\nFUTURE=retained\n[HEADER]\nVERSION=1\n",
    )
    .unwrap();
    let (surfaces, diagnostics) = surfaces::load(&[&defaults, &selected]).unwrap();
    let road = &surfaces["ROAD"];
    assert_eq!(surfaces.len(), 2);
    near(road.friction, 0.9);
    assert!(road.valid_track);
    assert!(!road.pitlane);
    assert_eq!(road.line, 1);
    assert_eq!(road.parameters["SIN_HEIGHT"], 0.01);
    assert_eq!(road.parameters["FUTURE"], "retained");
    assert!(
        diagnostics
            .iter()
            .any(|e| e.code == "surface_field_unsupported" && e.line == 9)
    );
    assert!(diagnostics.iter().any(|e| e.code == "surface_section"));
    for (text, code) in [
        (definition("ROAD", 0, "NaN"), "nonfinite_number"),
        (definition("ROAD", 0, "-1"), "surface_range"),
        (definition("1ROAD", 0, "1"), "surface_key"),
        (
            definition("ROAD", 0, "1") + &definition("ROAD", 1, "1"),
            "surface_duplicate",
        ),
        (
            definition("ROAD", 0, "1").replace("IS_PITLANE=0", "IS_PITLANE=2"),
            "surface_flag",
        ),
        (
            definition("ROAD", 0, "1").replace("DAMPING=0.02\n", ""),
            "surface_field",
        ),
    ] {
        std::fs::write(&selected, text).unwrap();
        let error = surfaces::load(&[&selected]).unwrap_err();
        assert_eq!(error.code, code);
        assert!(error.line > 0);
    }
    std::fs::write(
        &selected,
        definition("ROAD", 0, "1") + &definition("ROAD_EXTRA", 1, "1"),
    )
    .unwrap();
    let (surfaces, _) = surfaces::load(&[&selected]).unwrap();
    assert_eq!(
        surfaces::mesh_surface("001road_detail", &surfaces)
            .unwrap()
            .unwrap()
            .key,
        "ROAD"
    );
    assert_eq!(
        surfaces::mesh_surface("1ROAD_EXTRA", &surfaces)
            .unwrap_err()
            .code,
        "surface_ambiguous"
    );
    assert_eq!(
        surfaces::mesh_surface("1GRAVEL", &surfaces)
            .unwrap_err()
            .code,
        "surface_unmatched"
    );
    for name in ["ROAD", "0ROAD", "000ROAD"] {
        assert!(surfaces::mesh_surface(name, &surfaces).unwrap().is_none());
    }
}

#[test]
fn omitted_geometry_is_reported_and_empty_ground_is_not_ready() {
    let dir = directory(&[
        triangle("1ROAD_EXTRA", 0.),
        triangle("1GRAVEL", 0.),
        triangle("1WALL", 0.),
        named_mesh(
            "1ROAD",
            &[[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            1,
            1,
            0.,
            2,
        ),
        named_mesh("1ROAD_skin", &[[0.; 3]; 3], 1, 1, 0., 3),
    ]);
    std::fs::write(
        dir.0.join("surfaces.ini"),
        definition("ROAD", 0, "1") + &definition("ROAD_EXTRA", 1, "1"),
    )
    .unwrap();
    let ground = ground(&dir.0);
    let summary = ground.summary();
    assert_eq!(summary["triangles"], 0);
    assert_eq!(summary["skipped_vertical_or_degenerate"], 1);
    assert_eq!(summary["bvh_ready"], false);
    assert_eq!(summary["simulation_ready"], false);
    for code in [
        "surface_ambiguous",
        "surface_unmatched",
        "ground_wall_omitted",
        "ground_skinning_unsupported",
        "no_ground",
        "ground_query_scope",
    ] {
        assert!(
            summary["diagnostics"]
                .as_array()
                .unwrap()
                .iter()
                .any(|d| d["code"] == code),
            "{code}"
        );
    }
    assert!(ground.probe([0.; 3], 10.).unwrap().is_none());
}

#[test]
fn malformed_models_translated_range_and_triangle_budget_fail_explicitly() {
    let dir = directory(&[triangle("1ROAD", 0.)]);
    std::fs::write(
        dir.0.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=10000000,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    assert_eq!(
        load(&dir.0.join("models.ini"), &[&dir.0.join("surfaces.ini")])
            .err()
            .unwrap()
            .code,
        "ground_range"
    );
    std::fs::write(
        dir.0.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    std::fs::write(dir.0.join("a.kn5"), b"bad").unwrap();
    assert_eq!(
        asc_scene::sources(&dir.0.join("models.ini"))
            .unwrap()
            .models
            .len(),
        1
    );
    assert_eq!(
        load(&dir.0.join("models.ini"), &[&dir.0.join("surfaces.ini")])
            .err()
            .unwrap()
            .code,
        "ground_model"
    );
    let indices = vec![0; 3_000_003];
    let original = fixture::mesh(&[[0.; 3]], &indices, 2, 0, 0, 0., 0);
    let mut node = original[..4].to_vec();
    fixture::string(&mut node, "1ROAD");
    node.extend(&original[8 + "original_mesh".len()..]);
    std::fs::write(
        dir.0.join("a.kn5"),
        fixture::file(fixture::base(&[node], Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    assert_eq!(
        load(&dir.0.join("models.ini"), &[&dir.0.join("surfaces.ini")])
            .err()
            .unwrap()
            .code,
        "ground_limit"
    );
}

#[test]
fn positive_near_lod_is_retained_by_reader_but_not_claimed_as_ground() {
    let node = named_mesh(
        "1ROAD",
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        0,
        0,
        100.,
        2,
    );
    let dir = directory(&[node]);
    let collision = asc_kn5::load_collision(&dir.0.join("a.kn5")).unwrap();
    assert_eq!(collision.meshes[0].lod[0], 100.);
    let ground = ground(&dir.0);
    assert_eq!(ground.summary()["triangles"], 0);
    assert!(
        ground.summary()["diagnostics"]
            .as_array()
            .unwrap()
            .iter()
            .any(|d| d["code"] == "ground_lod_unsupported")
    );
}

#[test]
fn repeated_surface_metadata_cannot_bypass_the_scene_payload_budget() {
    let dir = directory(&[triangle("1ROAD", 0.)]);
    std::fs::write(
        dir.0.join("surfaces.ini"),
        definition("ROAD", 0, "1") + "FUTURE=" + &"x".repeat(700_000),
    )
    .unwrap();
    let config = (0..64)
        .map(|n| format!("[MODEL_{n}]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n"))
        .collect::<String>();
    std::fs::write(dir.0.join("models.ini"), config).unwrap();
    let error = load(&dir.0.join("models.ini"), &[&dir.0.join("surfaces.ini")])
        .err()
        .unwrap();
    assert_eq!(error.code, "ground_limit");
    assert!(error.message.contains("metadata"));
    assert_eq!(
        surfaces::load(&[dir.0.join("surfaces.ini").as_path(); 17])
            .unwrap_err()
            .code,
        "surface_limit"
    );
}
