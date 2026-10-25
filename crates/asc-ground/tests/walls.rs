use asc_ground::{OrientedBox, WallIndex};
use asc_test_support as f;
use glam::{DQuat, DVec3, Mat4, Vec3};
use std::path::PathBuf;
fn body(center: [f64; 3]) -> OrientedBox {
    OrientedBox::new(center, [1.; 3], [0., 0., 0., 1.]).unwrap()
}
fn wall() -> [[f64; 3]; 3] {
    [[-4., -4., 0.], [4., -4., 0.], [0., 4., 0.]]
}
fn near(a: [f64; 3], b: DVec3) {
    assert!((DVec3::from_array(a) - b).length() < 1e-8, "{a:?} != {b:?}");
}
#[test]
fn finite_double_sided_wall_contacts_preserve_margin_normal_and_clipped_patch() {
    for z in [-0.9, 0.9] {
        let a = body([0., 0., z])
            .contact_triangle(wall(), 0.)
            .unwrap()
            .unwrap();
        let [x, y, z0] = wall();
        let b = body([0., 0., z])
            .contact_triangle([x, z0, y], 0.)
            .unwrap()
            .unwrap();
        assert!((a.separation_m + 0.1).abs() < 1e-10);
        near(a.normal_world, DVec3::Z * z.signum());
        near(b.normal_world, DVec3::from_array(a.normal_world));
        assert_eq!(a.points_world_m.len(), 4);
        for p in &a.points_world_m {
            assert!(p[2].abs() < 1e-10 && p[0].abs() <= 1. + 1e-8 && p[1].abs() <= 1. + 1e-8);
            assert!(
                b.points_world_m
                    .iter()
                    .any(|q| (DVec3::from_array(*p) - DVec3::from_array(*q)).length() < 1e-8)
            );
        }
    }
    let c = body([0., 0., 1.0015])
        .contact_triangle(wall(), 0.002)
        .unwrap()
        .unwrap();
    assert!((c.separation_m - 0.0015).abs() < 1e-10);
    assert!(
        body([0., 0., 1.003])
            .contact_triangle(wall(), 0.002)
            .unwrap()
            .is_none()
    );
    assert!(
        body([10., 0., 0.5])
            .contact_triangle(wall(), 0.)
            .unwrap()
            .is_none()
    );
}
#[test]
fn box_triangle_sat_rejects_edge_axis_separation_and_retains_vertex_touching() {
    let small = OrientedBox::new([0.; 3], [0.5; 3], [0., 0., 0., 1.]).unwrap();
    assert!(
        small
            .contact_triangle([[2., 0., 0.], [0., 2., 0.], [2., 2., 0.]], 0.)
            .unwrap()
            .is_none()
    );
    let triangle = [[1., 1., 1.], [2., 1., 1.], [1., 2., 1.]];
    let c = body([0.; 3])
        .contact_triangle(triangle, 0.)
        .unwrap()
        .unwrap();
    assert!(c.separation_m.abs() < 1e-10);
    for p in c.points_world_m {
        near(p, DVec3::ONE);
    }
    // Both exact edge and corner cases remain winding independent.
    let [a, b, c] = triangle;
    let x = body([0.; 3])
        .contact_triangle(triangle, 0.)
        .unwrap()
        .unwrap();
    let y = body([0.; 3])
        .contact_triangle([a, c, b], 0.)
        .unwrap()
        .unwrap();
    near(x.normal_world, DVec3::from_array(y.normal_world));
}
#[test]
fn pitched_rolled_box_contacts_transform_covariantly_with_finite_triangles() {
    let q =
        DQuat::from_rotation_x(0.35) * DQuat::from_rotation_y(-0.7) * DQuat::from_rotation_z(0.2);
    let shift = DVec3::new(10., 3., -4.);
    let b = OrientedBox::new(
        (shift + q * DVec3::new(0., 0., 0.9)).to_array(),
        [1.; 3],
        q.to_array(),
    )
    .unwrap();
    let triangle = wall().map(|p| (shift + q * DVec3::from_array(p)).to_array());
    let contact = b.contact_triangle(triangle, 0.).unwrap().unwrap();
    near(contact.normal_world, q * DVec3::Z);
    assert!((contact.separation_m + 0.1).abs() < 1e-10);
    let reference = body([0., 0., 0.9])
        .contact_triangle(wall(), 0.)
        .unwrap()
        .unwrap();
    for point in contact.points_world_m {
        let local = q.conjugate() * (DVec3::from_array(point) - shift);
        assert!(
            reference
                .points_world_m
                .iter()
                .any(|p| (local - DVec3::from_array(*p)).length() < 1e-8)
        );
    }
}
#[test]
fn box_triangle_queries_reject_invalid_frames_ranges_margins_and_degenerate_geometry() {
    for (center, half, q) in [
        ([f64::NAN, 0., 0.], [1.; 3], [0., 0., 0., 1.]),
        ([0.; 3], [0.; 3], [0., 0., 0., 1.]),
        ([0.; 3], [10.1; 3], [0., 0., 0., 1.]),
        ([0.; 3], [1.; 3], [0.; 4]),
    ] {
        assert!(OrientedBox::new(center, half, q).is_err());
    }
    assert!(body([0.; 3]).contact_triangle([[0.; 3]; 3], 0.).is_err());
    assert!(
        body([0.; 3])
            .contact_triangle([[1e8, 0., 0.], [0.; 3], [1., 0., 0.]], 0.)
            .is_err()
    );
    for margin in [-1., 0.101, f64::NAN] {
        assert!(body([0.; 3]).contact_triangle(wall(), margin).is_err());
    }
    assert!(body([1e7, 0., 0.]).aabb(0.).is_err());
}
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn fixture(nodes: &[Vec<u8>], translation: bool) -> Directory {
    let root = f::temp_path("native-walls");
    std::fs::create_dir_all(&root).unwrap();
    std::fs::write(
        root.join("wall.kn5"),
        f::file(
            f::base(
                nodes,
                if translation {
                    Mat4::from_translation(Vec3::new(2., 3., 4.)).to_cols_array()
                } else {
                    Mat4::IDENTITY.to_cols_array()
                },
                1,
            ),
            6,
        ),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        format!(
            "[MODEL_0]\nFILE=wall.kn5\nPOSITION={},0,0\nROTATION=0,0,0\n",
            if translation { 5 } else { 0 }
        ),
    )
    .unwrap();
    Directory(root)
}
fn mesh(name: &str, points: &[[f32; 3]], indices: &[u16], lod: f32) -> Vec<u8> {
    f::named(f::mesh(points, indices, 2, 0, 0, lod, 0), name)
}
#[test]
fn wall_index_retains_hidden_vertical_physics_and_provenance_without_ground_coefficients() {
    let p = wall().map(|p| p.map(|v| v as f32));
    let hidden = mesh("1wAlL", &p, &[0, 1, 2], 0.);
    let lod = mesh("1WALL_LOD", &p, &[0, 1, 2], 20.);
    let inactive = f::base(
        &[mesh("1WALL_INACTIVE", &p, &[0, 1, 2], 0.)],
        Mat4::IDENTITY.to_cols_array(),
        0,
    );
    let nonphysical = mesh("WALL", &p, &[0, 1, 2], 0.);
    let degenerate = mesh("1WALL_DEGENERATE", &[[0.; 3]; 3], &[0, 1, 2], 0.);
    let road = mesh(
        "1ROAD",
        &[[-4., 0., -4.], [4., 0., -4.], [0., 0., 4.]],
        &[0, 1, 2],
        0.,
    );
    let d = fixture(
        &[hidden, lod, inactive, nonphysical, degenerate, road],
        true,
    );
    let walls = WallIndex::load(&d.0.join("models.ini")).unwrap();
    assert_eq!(walls.summary()["triangles"], 1);
    assert_eq!(walls.summary()["skipped_degenerate"], 1);
    let c = walls.contacts(&body([7., 3., 4.9]), 0.).unwrap();
    assert_eq!(c.len(), 1);
    assert_eq!(c[0].mesh, "1wAlL");
    assert_eq!(c[0].model, "wall.kn5");
    assert_eq!(c[0].triangle, 0);
    near(c[0].geometry.normal_world, DVec3::Z);
    assert!(
        walls
            .contacts(&body([7., 100., 4.9]), 0.)
            .unwrap()
            .is_empty()
    );
    std::fs::write(
        d.0.join("surfaces.ini"),
        "[SURFACE_0]\nKEY=ROAD\nFRICTION=1\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n",
    )
    .unwrap();
    let ground = asc_ground::load(&d.0.join("models.ini"), &[&d.0.join("surfaces.ini")]).unwrap();
    assert_eq!(ground.summary()["triangles"], 1);
    assert!((ground.probe([7., 10., 4.], 20.).unwrap().unwrap().position[1] - 3.).abs() < 1e-8);
}
#[test]
fn wall_index_bvh_uses_height_and_rejects_candidate_overflow_without_truncating() {
    let p = wall().map(|p| p.map(|v| v as f32));
    for count in [4096, 4097] {
        let indices = (0..count).flat_map(|_| [0, 1, 2]).collect::<Vec<u16>>();
        let d = fixture(&[mesh("1WALL", &p, &indices, 0.)], false);
        let index = WallIndex::load(&d.0.join("models.ini")).unwrap();
        let result = index.contacts(&body([0., 0., 0.9]), 0.);
        if count == 4096 {
            assert_eq!(result.unwrap().len(), 4096);
        } else {
            assert_eq!(result.unwrap_err().code, "wall_query_limit");
        }
    }
    let mut points = Vec::new();
    let mut indices = Vec::new();
    for i in 0..4097 {
        let start = points.len() as u16;
        points.extend(p.map(|mut p| {
            p[1] += i as f32 * 20.;
            p
        }));
        indices.extend([start, start + 1, start + 2]);
    }
    let d = fixture(&[mesh("1WALL", &points, &indices, 0.)], false);
    let index = WallIndex::load(&d.0.join("models.ini")).unwrap();
    assert_eq!(index.contacts(&body([0., 0., 0.9]), 0.).unwrap().len(), 1);
    let oversized = (0..250001).flat_map(|_| [0, 1, 2]).collect::<Vec<u16>>();
    let d = fixture(&[mesh("1WALL", &p, &oversized, 0.)], false);
    assert_eq!(
        WallIndex::load(&d.0.join("models.ini")).err().unwrap().code,
        "wall_limit"
    );
}

fn sweep_fixture() -> (Directory, WallIndex) {
    let d = fixture(
        &[mesh(
            "1WALL",
            &[[-20., -20., 0.], [20., -20., 0.], [0., 20., 0.]],
            &[0, 1, 2],
            0.,
        )],
        false,
    );
    let index = WallIndex::load(&d.0.join("models.ini")).unwrap();
    (d, index)
}
fn sweep_motion(
    a: [f64; 3],
    b: [f64; 3],
    qa: DQuat,
    qb: DQuat,
    center: [f64; 3],
    half: [f64; 3],
) -> asc_ground::BoxMotion {
    asc_ground::BoxMotion::new(a, b, qa.to_array(), qb.to_array(), center, half).unwrap()
}
#[test]
fn conservative_wall_sweep_detects_pass_through_with_clear_endpoints_and_closed_boundary_cases() {
    use asc_ground::WallSweepSettings;
    let (_d, index) = sweep_fixture();
    let motion = sweep_motion(
        [0., 0., 5.],
        [0., 0., -5.],
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        [0.; 3],
        [1.; 3],
    );
    assert!(
        index
            .contacts(&motion.at(0.).unwrap(), 0.)
            .unwrap()
            .is_empty()
    );
    assert!(
        index
            .contacts(&motion.at(1.).unwrap(), 0.)
            .unwrap()
            .is_empty()
    );
    let hit = index.sweep(&motion, WallSweepSettings::default()).unwrap();
    let expected = (4. - 1e-6) / 10.;
    assert!((hit.fraction.unwrap() - expected).abs() < 1e-10);
    assert_eq!(hit.candidate_count, 1);
    near(hit.contacts[0].geometry.normal_world, DVec3::Z);
    assert!(hit.contacts[0].geometry.separation_m <= 1e-6 + 1e-9);
    for (a, b, expected) in [
        ([0., 0., 2.], [0., 0., 1.], Some(1. - 1e-6)),
        ([0., 0., 0.5], [0., 0., 5.], Some(0.)),
        ([0., 0., 5.], [0., 0., 5.], None),
        ([0., 0., 5.], [0., 0., 10.], None),
    ] {
        let m = sweep_motion(a, b, DQuat::IDENTITY, DQuat::IDENTITY, [0.; 3], [1.; 3]);
        let h = index.sweep(&m, WallSweepSettings::default()).unwrap();
        match expected {
            Some(t) => assert!((h.fraction.unwrap() - t).abs() < 1e-8),
            None => assert!(h.fraction.is_none()),
        }
    }
}
#[test]
fn conservative_rotation_and_offset_reference_sweeps_match_analytical_first_proximity() {
    use asc_ground::WallSweepSettings;
    let (_d, index) = sweep_fixture();
    for offset in [false, true] {
        let (reference, local, half, aa, bb, cc) = if offset {
            (
                [0., 0., 2.],
                [2., 0., 0.],
                [0.1; 3],
                2.1_f64,
                0.1_f64,
                2_f64,
            )
        } else {
            (
                [0., 0., 1.5],
                [0.; 3],
                [2., 0.2, 0.2],
                2_f64,
                0.2_f64,
                1.5_f64,
            )
        };
        let m = sweep_motion(
            reference,
            reference,
            DQuat::IDENTITY,
            DQuat::from_rotation_y(std::f64::consts::PI),
            local,
            half,
        );
        assert!(index.contacts(&m.at(0.).unwrap(), 0.).unwrap().is_empty());
        assert!(index.contacts(&m.at(1.).unwrap(), 0.).unwrap().is_empty());
        let hit = index.sweep(&m, WallSweepSettings::default()).unwrap();
        let angle = ((cc - 1e-6) / aa.hypot(bb)).asin() - bb.atan2(aa);
        let fraction = hit.fraction.unwrap();
        assert!(
            (fraction - angle / std::f64::consts::PI).abs() < 1e-8,
            "{fraction} vs {}",
            angle / std::f64::consts::PI
        );
        assert!(hit.iterations > 2);
        assert!(!hit.contacts.is_empty());
        let early = m.at((fraction - 1e-5).max(0.)).unwrap();
        assert!(index.contacts(&early, 0.).unwrap().is_empty());
    }
}
#[test]
fn swept_rotation_is_world_covariant_quaternion_sign_invariant_and_finite_face_aware() {
    use asc_ground::WallSweepSettings;
    let (_d, index) = sweep_fixture();
    let rotation =
        DQuat::from_rotation_y(0.4) * DQuat::from_rotation_z(-0.3) * DQuat::from_rotation_x(0.2);
    let shift = DVec3::new(3., -2., 5.);
    let points = [[-20., -20., 0.], [20., -20., 0.], [0., 20., 0.]].map(|p| {
        (shift + rotation * DVec3::from_array(p))
            .to_array()
            .map(|x| x as f32)
    });
    let d = fixture(&[mesh("1WALL", &points, &[0, 2, 1], 0.)], false);
    let transformed = WallIndex::load(&d.0.join("models.ini")).unwrap();
    let start = DVec3::new(0., 0., 2.);
    let end = DQuat::from_rotation_y(2.5);
    let base = sweep_motion(
        start.to_array(),
        start.to_array(),
        DQuat::IDENTITY,
        end,
        [2., 0., 0.],
        [0.1; 3],
    );
    let t0 = index
        .sweep(&base, WallSweepSettings::default())
        .unwrap()
        .fraction
        .unwrap();
    for sign in [1., -1.] {
        let q = if sign > 0. {
            rotation * end
        } else {
            -(rotation * end)
        };
        let m = sweep_motion(
            (shift + rotation * start).to_array(),
            (shift + rotation * start).to_array(),
            rotation,
            q,
            [2., 0., 0.],
            [0.1; 3],
        );
        let h = transformed.sweep(&m, WallSweepSettings::default()).unwrap();
        assert!((h.fraction.unwrap() - t0).abs() < 2e-6);
        assert!(
            (DVec3::from_array(h.contacts[0].geometry.normal_world) - rotation * DVec3::Z).length()
                < 2e-6
        );
    }
    // A finite triangular face cannot block a path only intersecting its infinite plane/AABB.
    let d = fixture(
        &[mesh(
            "1WALL",
            &[[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            &[0, 1, 2],
            0.,
        )],
        false,
    );
    let finite = WallIndex::load(&d.0.join("models.ini")).unwrap();
    let m = sweep_motion(
        [3., 3., 1.],
        [3., 3., -1.],
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        [0.; 3],
        [0.1; 3],
    );
    let h = finite.sweep(&m, WallSweepSettings::default()).unwrap();
    assert_eq!(h.candidate_count, 1);
    assert!(h.fraction.is_none());
}
#[test]
fn swept_bounds_caps_and_exhaustion_fail_instead_of_certifying_an_unresolved_path() {
    use asc_ground::{BoxMotion, WallSweepSettings};
    let (_d, index) = sweep_fixture();
    let m = sweep_motion(
        [0., 0., 5.],
        [0., 0., -5.],
        DQuat::IDENTITY,
        DQuat::IDENTITY,
        [0.; 3],
        [1.; 3],
    );
    assert_eq!(
        index
            .sweep(
                &m,
                WallSweepSettings {
                    max_iterations: 1,
                    ..WallSweepSettings::default()
                }
            )
            .unwrap_err()
            .code,
        "wall_sweep_iterations"
    );
    for tolerance in [-1., 0., 0.0011, f64::NAN] {
        assert!(
            index
                .sweep(
                    &m,
                    WallSweepSettings {
                        distance_tolerance_m: tolerance,
                        ..WallSweepSettings::default()
                    }
                )
                .is_err()
        );
    }
    assert!(m.at(-0.1).is_err());
    assert!(m.at(f64::NAN).is_err());
    assert!(
        BoxMotion::new(
            [0.; 3],
            [f64::NAN, 0., 0.],
            [0., 0., 0., 1.],
            [0., 0., 0., 1.],
            [0.; 3],
            [1.; 3]
        )
        .is_err()
    );
    assert!(
        BoxMotion::new(
            [0.; 3],
            [0.; 3],
            [0.; 4],
            [0., 0., 0., 1.],
            [0.; 3],
            [1.; 3]
        )
        .is_err()
    );
    assert!(
        BoxMotion::new(
            [1e7, 0., 0.],
            [0.; 3],
            [0., 0., 0., 1.],
            [0., 0., 0., 1.],
            [0.; 3],
            [1.; 3]
        )
        .is_err()
    );
    for count in [4096, 4097] {
        let ids = (0..count).flat_map(|_| [0, 1, 2]).collect::<Vec<u16>>();
        let d = fixture(
            &[mesh(
                "1WALL",
                &[[-20., -20., 0.], [20., -20., 0.], [0., 20., 0.]],
                &ids,
                0.,
            )],
            false,
        );
        let walls = WallIndex::load(&d.0.join("models.ini")).unwrap();
        let result = walls.sweep(&m, WallSweepSettings::default());
        if count == 4096 {
            assert_eq!(result.unwrap().contacts.len(), 4096);
        } else {
            assert_eq!(result.unwrap_err().code, "wall_sweep_limit");
        }
    }
}
