//! Original generated fixtures. No game or SDK bytes are included.
use glam::Mat4;
use std::{
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::{SystemTime, UNIX_EPOCH},
};
/// Original tetrahedron with an unreferenced outlier; no game-derived geometry.
pub fn collider_tetrahedron_file() -> Vec<u8> {
    file(
        base(
            &[mesh(
                &[
                    [-0.5, 0., -1.],
                    [0.5, 0., -1.],
                    [0., 1., -1.],
                    [0., 0.5, 1.],
                    [99., 99., 99.],
                ],
                &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0],
                2,
                1,
                1,
                0.,
                0,
            )],
            Mat4::IDENTITY.to_cols_array(),
            1,
        ),
        6,
    )
}

/// Distinct within a test process even when several calls see the same clock tick.
pub fn temp_path(label: &str) -> PathBuf {
    static SERIAL: AtomicU64 = AtomicU64::new(0);
    let counter = SERIAL.fetch_add(1, Ordering::Relaxed);
    let stamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "openasc-{label}-{}-{stamp}-{counter}",
        std::process::id()
    ))
}

/// Original 4x4 single-colour BC1 DDS, with no game-derived bytes.
pub fn bc1(colour: u16) -> Vec<u8> {
    let mut header = [0u32; 31];
    header[0] = 124;
    header[1] = 0x1007;
    header[2] = 4;
    header[3] = 4;
    header[6] = 1;
    header[18] = 32;
    header[19] = 4;
    header[20] = u32::from_le_bytes(*b"DXT1");
    let mut out = b"DDS ".to_vec();
    for value in header {
        out.extend(value.to_le_bytes());
    }
    out.extend(colour.to_le_bytes());
    out.extend([0; 6]);
    out
}

pub fn layer_properties() -> Vec<(&'static str, [f32; 10])> {
    ["ksDiffuse", "multR", "multG", "multB", "multA", "magicMult"]
        .into_iter()
        .map(|name| {
            let mut values = [0.; 10];
            values[..3].copy_from_slice(&[1., 1., 1.]);
            (name, values)
        })
        .collect()
}
pub fn multilayer_file(node: Vec<u8>) -> Vec<u8> {
    let names = ["base", "mask", "r", "g", "b", "a"];
    let mut textures = [
        bc1(0xffff),
        bc1(0xf800),
        bc1(0x07e0),
        bc1(0x001f),
        bc1(0xf800),
        bc1(0xffff),
    ];
    // Red data mask with zero alpha: only the R detail contributes.
    textures[1][84..88].copy_from_slice(b"DXT5");
    textures[1].splice(128..128, [0; 8]);
    let textures: Vec<_> = names
        .iter()
        .zip(&textures)
        .map(|(name, data)| (*name, data.as_slice()))
        .collect();
    let samplers = [
        "txDiffuse",
        "txMask",
        "txDetailR",
        "txDetailG",
        "txDetailB",
        "txDetailA",
    ];
    let mappings: Vec<_> = samplers
        .iter()
        .zip(names)
        .enumerate()
        .map(|(i, (sampler, name))| (*sampler, i as u32, name))
        .collect();
    material_file(
        node,
        6,
        &textures,
        "ksMultilayer",
        &layer_properties(),
        &mappings,
    )
}

pub fn u32(out: &mut Vec<u8>, value: u32) {
    out.extend(value.to_le_bytes());
}
pub fn floats(out: &mut Vec<u8>, values: &[f32]) {
    for value in values {
        out.extend(value.to_le_bytes());
    }
}
pub fn string(out: &mut Vec<u8>, value: &str) {
    u32(out, value.len() as u32);
    out.extend(value.as_bytes());
}
/// Rename an original generated node, preserving all other fields.
pub fn named(node: Vec<u8>, name: &str) -> Vec<u8> {
    let old = u32::from_le_bytes(node[4..8].try_into().unwrap()) as usize;
    let mut out = node[..4].to_vec();
    string(&mut out, name);
    out.extend(&node[8 + old..]);
    out
}
pub fn base(children: &[Vec<u8>], matrix: [f32; 16], active: u8) -> Vec<u8> {
    let mut out = Vec::new();
    u32(&mut out, 1);
    string(&mut out, "original_root");
    u32(&mut out, children.len() as u32);
    out.push(active);
    floats(&mut out, &matrix);
    for child in children {
        out.extend(child);
    }
    out
}
pub fn mesh(
    points: &[[f32; 3]],
    indices: &[u16],
    kind: u32,
    visible: u8,
    renderable: u8,
    lod: f32,
    material: u32,
) -> Vec<u8> {
    let mut out = Vec::new();
    u32(&mut out, kind);
    string(&mut out, "original_mesh");
    u32(&mut out, 0);
    out.extend([1, 1, visible, 0]);
    if kind == 3 {
        u32(&mut out, 1);
        string(&mut out, "original_bone");
        floats(&mut out, &Mat4::IDENTITY.to_cols_array());
    }
    u32(&mut out, points.len() as u32);
    for point in points {
        floats(&mut out, point);
        floats(&mut out, &[0., 1., 0., 0., 0., 1., 0., 0.]);
        if kind == 3 {
            floats(&mut out, &[1., 0., 0., 0., 0., -1., -1., -1.]);
        }
    }
    u32(&mut out, indices.len() as u32);
    for i in indices {
        out.extend(i.to_le_bytes());
    }
    u32(&mut out, material);
    u32(&mut out, 0);
    floats(&mut out, &[lod, 0.]);
    if kind == 2 {
        floats(&mut out, &[0., 0., 0., 1.]);
        out.push(renderable);
    }
    out
}
pub fn file(node: Vec<u8>, version: u32) -> Vec<u8> {
    file_texture(node, version, &[11, 12, 13])
}
pub fn file_texture(node: Vec<u8>, version: u32, texture: &[u8]) -> Vec<u8> {
    material_file(
        node,
        version,
        &[("original_texture", texture)],
        "original_shader",
        &[("ksDiffuse", [0.7, 0., 0., 0., 0., 0., 0., 0., 0., 0.])],
        &[("txDiffuse", 0, "original_texture")],
    )
}
pub fn material_file(
    node: Vec<u8>,
    version: u32,
    textures: &[(&str, &[u8])],
    shader: &str,
    properties: &[(&str, [f32; 10])],
    mappings: &[(&str, u32, &str)],
) -> Vec<u8> {
    let mut out = b"sc6969".to_vec();
    u32(&mut out, version);
    if version == 6 {
        u32(&mut out, 0);
    }
    u32(&mut out, textures.len() as u32);
    for (name, texture) in textures {
        u32(&mut out, 1);
        string(&mut out, name);
        u32(&mut out, texture.len() as u32);
        out.extend(*texture);
    }
    u32(&mut out, 1);
    string(&mut out, "original_material");
    string(&mut out, shader);
    out.extend([0, 0]);
    u32(&mut out, 0);
    u32(&mut out, properties.len() as u32);
    for (name, values) in properties {
        string(&mut out, name);
        floats(&mut out, values);
    }
    u32(&mut out, mappings.len() as u32);
    for (sampler, slot, texture) in mappings {
        string(&mut out, sampler);
        u32(&mut out, *slot);
        string(&mut out, texture);
    }
    out.extend(node);
    out
}

mod vehicle;
pub use vehicle::{suspension_motion_file, vehicle_files};
