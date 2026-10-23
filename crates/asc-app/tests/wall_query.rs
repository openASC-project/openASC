use asc_test_support as f;
use glam::{Mat4, Vec3};
use std::{path::PathBuf, process::Command};
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn fixture() -> (Directory, serde_json::Value) {
    let root = f::temp_path("wall-query-cli");
    std::fs::create_dir_all(root.join("car/data")).unwrap();
    for (name, text) in f::vehicle_files() {
        std::fs::write(root.join("car/data").join(name), text).unwrap();
    }
    for name in ["original.kn5", "collider.kn5"] {
        std::fs::write(root.join("car").join(name), f::collider_tetrahedron_file()).unwrap();
    }
    let mesh = f::named(
        f::mesh(
            &[[-4., -4., 0.], [4., -4., 0.], [0., 4., 0.]],
            &[0, 1, 2],
            2,
            0,
            0,
            0.,
            0,
        ),
        "1WALL",
    );
    let spawn = f::named(f::base(&[], Mat4::IDENTITY.to_cols_array(), 1), "AC_PIT_0");
    std::fs::write(
        root.join("track.kn5"),
        f::file(
            f::base(&[mesh, spawn], Mat4::IDENTITY.to_cols_array(), 1),
            6,
        ),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=track.kn5\nPOSITION=5,2,4\nROTATION=0,0,0\n",
    )
    .unwrap();
    let c = serde_json::json!({"car_directory":"car","models_ini":"models.ini","data":"unpacked","compound":null,"spawn":"AC_PIT_0",
        "model_origin_in_chassis_m":[0,0,0],"model_orientation_in_chassis_xyzw":[0,0,0,1],
        "probes":[{"position_m":[0,0,0.5],"orientation_xyzw":[0,0,0,1],"margin_m":0},{"position_m":[0,0,10],"orientation_xyzw":[0,0,0,1],"margin_m":0}]});
    (Directory(root), c)
}
fn invoke(d: &Directory, c: &serde_json::Value) -> std::process::Output {
    std::fs::write(d.0.join("query.json"), c.to_string()).unwrap();
    Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("wall-query")
        .arg(d.0.join("query.json"))
        .output()
        .unwrap()
}
#[test]
fn cli_queries_original_collider_and_wall_patch_with_explicit_spawn_and_source_provenance() {
    let (d, c) = fixture();
    let before = std::fs::read(d.0.join("car/collider.kn5")).unwrap();
    let result = invoke(&d, &c);
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let v: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(
        v["records"][0]["position_world_m"],
        serde_json::json!([5., 2., 4.5])
    );
    assert_eq!(v["records"][0]["contacts"][0]["mesh"], "1WALL");
    assert_eq!(
        v["records"][0]["contacts"][0]["geometry"]["normal_world"],
        serde_json::json!([0., 0., 1.])
    );
    assert_eq!(v["records"][1]["contacts"], serde_json::json!([]));
    assert_eq!(v["collision_response"], false);
    assert_eq!(std::fs::read(d.0.join("car/collider.kn5")).unwrap(), before);
    // A rolled chassis still uses a proper full 3D box query.
    let mut q = c.clone();
    q["probes"][0]["orientation_xyzw"] = serde_json::json!(
        glam::Quat::from_axis_angle(Vec3::Z, 0.3)
            .as_dquat()
            .to_array()
    );
    assert!(invoke(&d, &q).status.success());
}
#[test]
fn cli_rejects_invalid_or_oversized_queries_without_partial_json() {
    let (d, c) = fixture();
    let mut cases = Vec::new();
    let mut v = c.clone();
    v["probes"] = serde_json::json!([]);
    cases.push(v);
    let mut v = c.clone();
    v["probes"][0]["orientation_xyzw"] = serde_json::json!([0, 0, 0, 0]);
    cases.push(v);
    let mut v = c.clone();
    v["probes"][0]["margin_m"] = 0.2.into();
    cases.push(v);
    let mut v = c.clone();
    v["extra"] = true.into();
    cases.push(v);
    let mut v = c.clone();
    v["spawn"] = "missing".into();
    cases.push(v);
    for c in cases {
        let r = invoke(&d, &c);
        assert!(!r.status.success());
        assert!(r.stdout.is_empty());
    }
    // A later probe exceeding the aggregate report budget must not print earlier records.
    let indices = (0..4096).flat_map(|_| [0, 1, 2]).collect::<Vec<u16>>();
    let wall = f::named(
        f::mesh(
            &[[-4., -4., 0.], [4., -4., 0.], [0., 4., 0.]],
            &indices,
            2,
            0,
            0,
            0.,
            0,
        ),
        "1WALL",
    );
    let spawn = f::named(f::base(&[], Mat4::IDENTITY.to_cols_array(), 1), "AC_PIT_0");
    std::fs::write(
        d.0.join("track.kn5"),
        f::file(
            f::base(&[wall, spawn], Mat4::IDENTITY.to_cols_array(), 1),
            6,
        ),
    )
    .unwrap();
    let mut crowded = c.clone();
    crowded["probes"] = serde_json::json!([c["probes"][0].clone(), c["probes"][0].clone()]);
    let r = invoke(&d, &crowded);
    assert!(!r.status.success());
    assert!(r.stdout.is_empty());
    assert!(String::from_utf8_lossy(&r.stderr).contains("4096 total"));
    std::fs::write(d.0.join("query.json"), vec![b' '; 8193]).unwrap();
    let r = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("wall-query")
        .arg(d.0.join("query.json"))
        .output()
        .unwrap();
    assert!(!r.status.success());
    assert!(r.stdout.is_empty());
}
