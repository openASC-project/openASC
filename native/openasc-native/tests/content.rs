mod support;
use glam::{Mat4, Vec3};
use openasc_native::{
    kn5::{Scene, parse},
    render::camera,
};
use std::io::Cursor;
use support::*;

fn triangle() -> Vec<u8> {
    mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    )
}

#[test]
fn hidden_active_road_can_focus_camera_without_drawing_helper_geometry() {
    let mut road = triangle();
    road[4..8].copy_from_slice(&5_u32.to_le_bytes());
    road.splice(8..8 + "original_mesh".len(), b"1ROAD".iter().copied());
    // visible flag follows child count and active/shadow flags.
    road[4 + 4 + 5 + 4 + 2] = 0;
    let scene = read(file(
        base(
            &[road, triangle()],
            Mat4::from_translation(Vec3::new(2., 3., 4.)).to_cols_array(),
            1,
        ),
        6,
    ));
    assert_eq!(scene.summary.drawn_meshes, 1);
    assert_eq!(
        scene.summary.road_focus_bounds,
        Some([[2., 3., 4.], [3., 3., 5.]])
    );
}

fn read(bytes: Vec<u8>) -> Scene {
    parse(Cursor::new(bytes)).unwrap_or_else(|e| panic!("{e}"))
}
fn fails(bytes: Vec<u8>, code: &str) {
    match parse(Cursor::new(bytes)) {
        Ok(_) => panic!("Expected {code}"),
        Err(e) => assert_eq!(e.code, code, "{e}"),
    }
}
#[test]
fn both_versions_have_identical_visible_geometry() {
    for version in [5, 6] {
        let scene = read(file(
            base(&[triangle()], Mat4::IDENTITY.to_cols_array(), 1),
            version,
        ));
        assert_eq!(
            (
                scene.summary.nodes,
                scene.summary.static_meshes,
                scene.summary.drawn_triangles
            ),
            (2, 1, 1)
        );
        assert_eq!((scene.summary.textures, scene.summary.materials), (1, 1));
        assert_eq!(scene.summary.bounds, Some([[0., 0., 0.], [1., 0., 1.]]));
        assert_eq!(scene.indices, [0, 2, 1]);
    }
}
#[test]
fn nested_transforms_are_composed_in_parent_order() {
    let child = base(
        &[triangle()],
        Mat4::from_rotation_y(std::f32::consts::FRAC_PI_2).to_cols_array(),
        1,
    );
    let scene = read(file(
        base(
            &[child],
            Mat4::from_translation(Vec3::new(2., 3., 4.)).to_cols_array(),
            1,
        ),
        6,
    ));
    for (actual, expected) in scene
        .vertices
        .iter()
        .zip([[2., 3., 4.], [2., 3., 3.], [3., 3., 4.]])
    {
        for (a, e) in actual.position.iter().zip(expected) {
            assert!((a - e).abs() < 1e-6);
        }
    }
}
#[test]
fn visibility_parent_activity_lod_and_skinning_do_not_desynchronize_reader() {
    let points = [[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]];
    let items = vec![
        mesh(&points, &[0, 1, 2], 2, 0, 1, 0., 0),
        mesh(&points, &[0, 1, 2], 2, 1, 0, 0., 0),
        mesh(&points, &[0, 1, 2], 2, 1, 1, 20., 0),
        mesh(&points, &[0, 1, 2], 3, 1, 1, 0., 0),
        base(&[triangle()], Mat4::IDENTITY.to_cols_array(), 0),
        triangle(),
    ];
    let scene = read(file(base(&items, Mat4::IDENTITY.to_cols_array(), 1), 6));
    assert_eq!(scene.summary.static_meshes, 5);
    assert_eq!(scene.summary.skinned_meshes, 1);
    assert_eq!(scene.summary.structural_triangles, 6);
    assert_eq!(scene.summary.drawn_meshes, 1);
    assert_eq!(scene.indices.len(), 3);
}
#[test]
fn every_truncated_prefix_fails_without_panicking() {
    let bytes = file(base(&[triangle()], Mat4::IDENTITY.to_cols_array(), 1), 6);
    for end in 0..bytes.len() {
        assert!(
            parse(Cursor::new(bytes[..end].to_vec())).is_err(),
            "Accepted prefix {end}"
        );
    }
}
#[test]
fn rejects_signature_version_unknown_nodes_trailing_data_and_invalid_flags() {
    let identity = Mat4::IDENTITY.to_cols_array();
    let mut bytes = file(base(&[], identity, 1), 6);
    bytes[0] = 0;
    fails(bytes, "kn5_signature");
    fails(file(base(&[], identity, 1), 7), "kn5_version");
    let mut node = base(&[], identity, 1);
    node[..4].copy_from_slice(&99_u32.to_le_bytes());
    fails(file(node, 6), "kn5_node_type");
    let mut bytes = file(base(&[], identity, 1), 6);
    bytes.push(0);
    fails(bytes, "kn5_trailing_data");
    fails(file(base(&[], identity, 2), 6), "kn5_flag");
}
#[test]
fn rejects_nonfinite_geometry_bad_indices_and_bad_material() {
    let identity = Mat4::IDENTITY.to_cols_array();
    let points = [[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]];
    for indices in [&[0, 3, 1][..], &[0, 1][..]] {
        fails(
            file(
                base(&[mesh(&points, indices, 2, 1, 1, 0., 0)], identity, 1),
                6,
            ),
            "kn5_indices",
        );
    }
    fails(
        file(
            base(&[mesh(&points, &[0, 1, 2], 2, 1, 1, 0., 1)], identity, 1),
            6,
        ),
        "kn5_material",
    );
    let mut points = points;
    points[1][2] = f32::INFINITY;
    fails(
        file(
            base(&[mesh(&points, &[0, 1, 2], 2, 1, 1, 0., 0)], identity, 1),
            6,
        ),
        "kn5_nonfinite",
    );
    let mut m = identity;
    m[0] = f32::NAN;
    fails(file(base(&[], m, 1), 6), "kn5_nonfinite");
    let mut m = identity;
    m[3] = 1.;
    fails(file(base(&[], m, 1), 6), "kn5_transform");
}
#[test]
fn budgets_utf8_and_hierarchy_depth_fail_explicitly() {
    let identity = Mat4::IDENTITY.to_cols_array();
    let mut node = base(&[], identity, 1);
    node[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
    fails(file(node, 6), "kn5_limit");
    let mut node = base(&[], identity, 1);
    node[8] = 255;
    fails(file(node, 6), "kn5_string");
    let mut node = base(&[], identity, 1);
    for _ in 0..129 {
        node = base(&[node], identity, 1);
    }
    fails(file(node, 6), "kn5_limit");
    // A declared oversized geometry count fails before attempting allocation/read.
    let mut node = triangle();
    let offset = 4 + 4 + "original_mesh".len() + 4 + 4;
    node[offset..offset + 4].copy_from_slice(&2_000_001_u32.to_le_bytes());
    fails(file(base(&[node], identity, 1), 6), "kn5_limit");
}
#[test]
fn rebases_indices_between_meshes_and_preserves_winding() {
    let scene = read(file(
        base(&[triangle(), triangle()], Mat4::IDENTITY.to_cols_array(), 1),
        6,
    ));
    assert_eq!(scene.indices, [0, 2, 1, 3, 5, 4]);
}
#[test]
fn camera_fits_all_corners_in_webgpu_clip_space_including_portrait() {
    let bounds = [[-123., -12., -23.], [25., 4., 100.]];
    for (w, h) in [(1240, 900), (319, 177), (177, 319)] {
        let m = Mat4::from_cols_array(&camera(bounds, w, h).unwrap());
        for i in 0..8 {
            let p = Vec3::new(
                bounds[i & 1][0],
                bounds[(i >> 1) & 1][1],
                bounds[(i >> 2) & 1][2],
            );
            let clip = m * p.extend(1.);
            assert!(
                clip.x.abs() < 1. && clip.y.abs() < 1. && clip.z > 0. && clip.z < 1.,
                "{clip:?}"
            );
        }
    }
    assert!(camera(bounds, 0, 1).is_err());
    assert!(camera(bounds, 4097, 1).is_err());
    assert!(camera([[1.; 3], [0.; 3]], 100, 100).is_err());
}
