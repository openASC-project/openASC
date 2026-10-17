//! Original generated fixtures. No game or SDK bytes are included.
use glam::Mat4;

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
    let mut out = b"sc6969".to_vec();
    u32(&mut out, version);
    if version == 6 {
        u32(&mut out, 0);
    }
    u32(&mut out, 1);
    u32(&mut out, 1);
    string(&mut out, "original_texture");
    u32(&mut out, 3);
    out.extend([11, 12, 13]);
    u32(&mut out, 1);
    string(&mut out, "original_material");
    string(&mut out, "original_shader");
    out.extend([0, 0]);
    u32(&mut out, 0);
    u32(&mut out, 1);
    string(&mut out, "ksDiffuse");
    floats(&mut out, &[0.7, 0., 0., 0., 0., 0., 0., 0., 0., 0.]);
    u32(&mut out, 1);
    string(&mut out, "txDiffuse");
    u32(&mut out, 0);
    string(&mut out, "original_texture");
    out.extend(node);
    out
}
