use asc_kn5 as kn5;
use asc_render::resources::texture as textures;
use asc_test_support as support;
use glam::Mat4;
use std::{io::Cursor, path::PathBuf};

struct Source(PathBuf);
impl Drop for Source {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.0);
    }
}
fn source() -> (Source, kn5::Scene) {
    let mut header = [0u32; 31];
    header[0] = 124;
    header[1] = 0x1007;
    header[2] = 4;
    header[3] = 4;
    header[6] = 1;
    header[18] = 32;
    header[19] = 4;
    header[20] = u32::from_le_bytes(*b"DXT1");
    let mut dds = b"DDS ".to_vec();
    for v in header {
        dds.extend(v.to_le_bytes());
    }
    dds.extend([0, 248, 0, 0, 0, 0, 0, 0]);
    let mesh = support::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    let bytes = support::file_texture(
        support::base(&[mesh], Mat4::IDENTITY.to_cols_array(), 1),
        6,
        &dds,
    );
    let path = support::temp_path("native-textures").with_extension("kn5");
    std::fs::write(&path, &bytes).unwrap();
    let scene = kn5::parse(Cursor::new(bytes)).unwrap_or_else(|e| panic!("{e}"));
    (Source(path), scene)
}
#[test]
fn unique_case_insensitive_references_are_cached_and_source_is_unchanged() {
    let (source, mut scene) = source();
    let before = std::fs::read(&source.0).unwrap();
    let mut material = scene.materials[0].clone();
    material.diffuse_refs[0].make_ascii_uppercase();
    scene.materials.push(material);
    let mut draw = scene.draws[0].clone();
    draw.material = 1;
    scene.draws.push(draw);
    let resources = textures::load(&scene, &source.0);
    assert_eq!(resources.material_textures, [Some(0), Some(0)]);
    assert_eq!(resources.textures.len(), 1);
    assert_eq!(
        &resources.textures[0].levels[0].rgba[..4],
        &[255, 0, 0, 255]
    );
    assert_eq!(std::fs::read(&source.0).unwrap(), before);
    assert!(resources.diagnostics.is_empty());
}
#[test]
fn duplicate_inactive_missing_and_ambiguous_sampler_references_are_diagnosed() {
    let (source, mut scene) = source();
    scene.textures[0].active = false;
    assert!(textures::load(&scene, &source.0).material_textures[0].is_none());
    scene.textures[0].active = true;
    let d = &scene.textures[0];
    scene.textures.push(kn5::Embedded {
        name: d.name.to_uppercase(),
        active: true,
        offset: d.offset,
        length: d.length,
    });
    assert!(textures::load(&scene, &source.0).material_textures[0].is_none());
    scene.textures.pop();
    scene.materials[0].diffuse_refs.push("other".into());
    assert!(textures::load(&scene, &source.0).material_textures[0].is_none());
    scene.materials[0].diffuse_refs = vec!["missing".into()];
    let r = textures::load(&scene, &source.0);
    assert!(r.textures.is_empty());
    assert_eq!(r.diagnostics.len(), 1);
}
#[test]
fn bad_embedded_ranges_and_source_budget_fall_back_without_large_allocations() {
    let (source, mut scene) = source();
    scene.textures[0].length = 128 * 1024 * 1024 + 1;
    let r = textures::load(&scene, &source.0);
    assert!(r.textures.is_empty());
    assert!(r.diagnostics[0].contains("budget"));
    scene.textures[0].length = 136;
    scene.textures[0].offset = u32::MAX as u64;
    let r = textures::load(&scene, &source.0);
    assert!(r.textures.is_empty());
    assert!(
        r.diagnostics[0].contains("declared range"),
        "{:?}",
        r.diagnostics
    );
    scene.textures[0].offset = u64::MAX;
    let r = textures::load(&scene, &source.0);
    assert!(r.textures.is_empty());
    assert!(
        r.diagnostics[0].contains("declared range"),
        "{:?}",
        r.diagnostics
    );
}

#[test]
fn malformed_dds_uses_diagnostic_colour_and_caches_the_failure() {
    let (source, _) = source();
    let mesh = support::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    std::fs::write(
        &source.0,
        support::file(support::base(&[mesh], Mat4::IDENTITY.to_cols_array(), 1), 6),
    )
    .unwrap();
    let mut scene = kn5::load(&source.0).unwrap_or_else(|e| panic!("{e}"));
    scene.materials.push(scene.materials[0].clone());
    let mut draw = scene.draws[0].clone();
    draw.material = 1;
    scene.draws.push(draw);
    let r = textures::load(&scene, &source.0);
    assert_eq!(r.material_textures, [None, None]);
    assert!(r.textures.is_empty());
    assert_eq!(r.diagnostics.len(), 1);
    assert!(r.diagnostics[0].contains("dds_header"));
}
