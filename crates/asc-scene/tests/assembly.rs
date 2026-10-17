use asc_scene::load;
use asc_test_support as fixture;
use glam::Mat4;
use std::{
    path::PathBuf,
    time::{SystemTime, UNIX_EPOCH},
};

struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn directory() -> Directory {
    let serial = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!("openasc-scene-{}-{serial}", std::process::id()));
    std::fs::create_dir(&path).unwrap();
    let mut mesh = fixture::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    let index = mesh
        .windows(13)
        .position(|v| v == b"original_mesh")
        .unwrap();
    mesh[index..index + 13].copy_from_slice(b"1ROAD________");
    let bytes = fixture::file(fixture::base(&[mesh], Mat4::IDENTITY.to_cols_array(), 1), 6);
    std::fs::write(path.join("a.kn5"), bytes).unwrap();
    Directory(path)
}
fn declaration(name: &str, file: &str, position: &str) -> String {
    format!("[{name}]\nFILE={file}\nPOSITION={position}\nROTATION=0,0,0\n")
}
#[test]
fn translates_and_rebases_repeated_models_preserving_local_materials_and_normals() {
    let dir = directory();
    let config = dir.0.join("models_layout.ini");
    std::fs::write(
        &config,
        declaration("MODEL_8", "a.kn5", "0,0,0")
            + &declaration("MODEL_2", "a.kn5", "10,2,-3")
            + "[DYNAMIC_OBJECT_0]\nFILE=a.kn5\n",
    )
    .unwrap();
    let track = load(&config).unwrap();
    let scene = &track.scene;
    assert_eq!(scene.indices, [0, 2, 1, 3, 5, 4]);
    assert_eq!(scene.vertices[3].position, [10., 2., -3.]);
    assert_eq!(scene.vertices[3].normal, [0., 1., 0.]);
    assert_eq!(scene.draws[1].start, 3);
    assert_eq!(scene.draws[1].material, 1);
    assert_eq!(scene.draws[1].center, [10.5, 2., -2.5]);
    assert_eq!(scene.summary.bounds, Some([[0., 0., -3.], [11., 2., 1.]]));
    assert_eq!(scene.summary.road_focus_bounds, scene.summary.bounds);
    assert_eq!(
        (scene.summary.drawn_meshes, scene.summary.drawn_triangles),
        (2, 2)
    );
    assert_eq!(scene.summary.version, 0);
    assert_eq!(track.manifest.models[0].section, "MODEL_8"); // Source order, not numeric sort.
    assert_eq!(track.manifest.models[1].textures, 1..2);
    assert_eq!(track.manifest.models[1].materials, 1..2);
    assert_eq!(track.manifest.models[1].version, 6);
    assert!(track.manifest.diagnostics[0].contains("DYNAMIC_OBJECT_0"));
}
#[test]
fn static_errors_are_fatal_with_source_lines_instead_of_silent_partial_scenes() {
    let dir = directory();
    let config = dir.0.join("models.ini");
    for (tail, code, line) in [
        (
            "[MODEL_1]\nFILE=missing.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
            "resource_io",
            6,
        ),
        (
            "[MODEL_1]\nFILE=a.kn5\nPOSITION=0,0\nROTATION=0,0,0\n",
            "invalid_vector",
            7,
        ),
        (
            "[MODEL_1]\nFILE=a.kn5\nPOSITION=0,nan,0\nROTATION=0,0,0\n",
            "nonfinite_number",
            7,
        ),
        (
            "[MODEL_1]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,90,0\n",
            "model_rotation_unsupported",
            8,
        ),
        (
            "[MODEL_1]\nFILE=a.kn5\nROTATION=0,0,0\n",
            "missing_transform",
            5,
        ),
        (
            "[MODEL_1]\nFILE=a.kn5\nPOSITION=1e100,0,0\nROTATION=0,0,0\n",
            "scene_range",
            7,
        ),
    ] {
        std::fs::write(&config, declaration("MODEL_0", "a.kn5", "0,0,0") + tail).unwrap();
        let error = load(&config).err().unwrap();
        assert_eq!((error.code, error.line), (code, line), "{error}");
    }
}
#[test]
fn portable_absolute_drive_stream_and_parent_paths_are_rejected_on_every_host() {
    let dir = directory();
    let config = dir.0.join("models.ini");
    for (file, code) in [
        ("../a.kn5", "resource_escape"),
        ("..\\a.kn5", "resource_escape"),
        ("/a.kn5", "resource_path"),
        ("C:\\a.kn5", "resource_path"),
        ("C:a.kn5", "resource_path"),
        ("\\\\server\\a.kn5", "resource_path"),
        ("a:stream.kn5", "resource_path"),
    ] {
        std::fs::write(&config, declaration("MODEL_0", file, "0,0,0")).unwrap();
        assert_eq!(load(&config).err().unwrap().code, code, "{file}");
    }
}
#[cfg(unix)]
#[test]
fn a_symlink_resolving_outside_the_track_is_rejected() {
    let dir = directory();
    let outside = directory();
    std::os::unix::fs::symlink(outside.0.join("a.kn5"), dir.0.join("link.kn5")).unwrap();
    let config = dir.0.join("models.ini");
    std::fs::write(&config, declaration("MODEL_0", "link.kn5", "0,0,0")).unwrap();
    assert_eq!(load(&config).err().unwrap().code, "resource_escape");
}
#[test]
fn unknown_keys_are_reported_empty_scenes_and_model_budget_are_rejected() {
    let dir = directory();
    let config = dir.0.join("models.ini");
    std::fs::write(
        &config,
        declaration("MODEL_0", "a.kn5", "0,0,0") + "SHADOW=1\n",
    )
    .unwrap();
    assert!(load(&config).unwrap().manifest.diagnostics[0].contains("SHADOW"));
    std::fs::write(&config, "[DYNAMIC_OBJECT_0]\nFILE=a.kn5").unwrap();
    assert_eq!(load(&config).err().unwrap().code, "no_models");
    let text: String = (0..257)
        .map(|i| declaration(&format!("MODEL_{i}"), "a.kn5", "0,0,0"))
        .collect();
    std::fs::write(&config, text).unwrap();
    assert_eq!(load(&config).err().unwrap().code, "scene_limit");
}
