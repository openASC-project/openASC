use asc_render::load_track_textures;
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
fn track() -> (Directory, asc_scene::Track) {
    let serial = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "openasc-track-textures-{}-{serial}",
        std::process::id()
    ));
    std::fs::create_dir(&path).unwrap();
    let mesh = fixture::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    for (name, colour) in [("a.kn5", 0xf800), ("b.kn5", 0x07e0)] {
        let node = fixture::base(
            std::slice::from_ref(&mesh),
            Mat4::IDENTITY.to_cols_array(),
            1,
        );
        std::fs::write(
            path.join(name),
            fixture::file_texture(node, 6, &fixture::bc1(colour)),
        )
        .unwrap();
    }
    let config = path.join("models.ini");
    std::fs::write(&config, "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n[MODEL_1]\nFILE=b.kn5\nPOSITION=2,0,0\nROTATION=0,0,0\n").unwrap();
    let loaded = asc_scene::load(&config).unwrap();
    (Directory(path), loaded)
}
#[test]
fn identical_texture_names_in_different_kn5_files_keep_their_own_pixels() {
    let (dir, track) = track();
    let before = std::fs::read(dir.0.join("a.kn5")).unwrap();
    let resources = load_track_textures(&track);
    assert!(
        resources.diagnostics.is_empty(),
        "{:?}",
        resources.diagnostics
    );
    assert_eq!(resources.material_textures, [Some(0), Some(1)]);
    assert_eq!(
        &resources.textures[0].levels[0].rgba[..4],
        &[255, 0, 0, 255]
    );
    assert_eq!(
        &resources.textures[1].levels[0].rgba[..4],
        &[0, 255, 0, 255]
    );
    assert_eq!(std::fs::read(dir.0.join("a.kn5")).unwrap(), before);
}
#[test]
fn a_missing_local_reference_never_borrows_a_different_models_texture() {
    let (_dir, mut track) = track();
    track.scene.textures[0].name = "different".into();
    let resources = load_track_textures(&track);
    assert_eq!(resources.material_textures, [None, Some(0)]);
    assert_eq!(
        &resources.textures[0].levels[0].rgba[..4],
        &[0, 255, 0, 255]
    );
    assert_eq!(resources.diagnostics.len(), 1);
}
#[test]
fn source_budget_is_shared_across_models_instead_of_reset_for_each_file() {
    let (_dir, mut track) = track();
    for texture in &mut track.scene.textures {
        texture.length = 64 * 1024 * 1024 + 1;
    }
    let resources = load_track_textures(&track);
    assert!(resources.textures.is_empty());
    assert_eq!(resources.diagnostics.len(), 2);
    assert!(resources.diagnostics[0].contains("declared range"));
    assert!(resources.diagnostics[1].contains("128 MiB scene budget"));
}
#[test]
#[ignore = "Requires a GPU or explicit software Vulkan adapter."]
fn composed_models_render_both_local_diffuse_textures() {
    let (_dir, track) = track();
    let resources = load_track_textures(&track);
    let frame = pollster::block_on(asc_render::render_with_textures(
        &track.scene,
        Some(&resources),
        319,
        241,
        std::env::var_os("OPENASC_TEST_FALLBACK").is_some(),
    ))
    .unwrap();
    assert_eq!(frame.report.uploaded_textures, 2);
    assert_eq!(frame.report.textured_draws, 2);
    assert!(
        frame
            .rgba
            .chunks_exact(4)
            .any(|p| p[0] > 40 && p[1] < 10 && p[2] < 10),
        "First model must be red"
    );
    assert!(
        frame
            .rgba
            .chunks_exact(4)
            .any(|p| p[1] > 40 && p[0] < 10 && p[2] < 10),
        "Second model must be green"
    );
}
