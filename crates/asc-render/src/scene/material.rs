//! Reconstructed multilayer base-colour inputs; original lighting remains incomplete.
use asc_kn5::Material;

pub const LAYER_SAMPLERS: [&str; 5] =
    ["txMask", "txDetailR", "txDetailG", "txDetailB", "txDetailA"];
pub struct Layers {
    pub scales: [[f32; 2]; 4],
    pub magic: f32,
    pub world: bool,
}
pub fn layers(material: &Material) -> Result<Option<Layers>, String> {
    let world = match material.shader.as_str() {
        "ksMultilayer" | "ksMultilayer_fresnel_nm" => true,
        "ksMultilayer_objsp" => false,
        _ => return Ok(None),
    };
    let mut scales = [[0.; 2]; 4];
    for (i, name) in ["multR", "multG", "multB", "multA"].iter().enumerate() {
        let value = material
            .properties
            .get(*name)
            .ok_or_else(|| format!("Missing {name}; multilayer base colour disabled."))?;
        scales[i] = if world && i < 3 {
            [value[0]; 2]
        } else {
            [value[1], value[2]]
        };
    }
    let magic = material
        .properties
        .get("magicMult")
        .ok_or("Missing magicMult; multilayer base colour disabled.")?[0];
    if scales
        .iter()
        .flatten()
        .chain(std::iter::once(&magic))
        .any(|v| !v.is_finite() || v.abs() > 10000.)
    {
        return Err("Multilayer parameter exceeds the finite +/-10000 preview range.".into());
    }
    Ok(Some(Layers {
        scales,
        magic,
        world,
    }))
}
