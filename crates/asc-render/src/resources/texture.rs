//! Resolve unique active embedded diffuse references and decode within scene budgets.
use asc_kn5::Scene;
use asc_texture as dds;
use std::{
    collections::HashMap,
    fs::File,
    io::{Read, Seek, SeekFrom},
    ops::Range,
    path::Path,
};

pub struct Resources {
    pub textures: Vec<dds::Texture>,
    pub material_textures: Vec<Option<usize>>,
    pub diagnostics: Vec<String>,
}

pub fn load(scene: &Scene, path: &Path) -> Resources {
    load_sources(
        scene,
        &[(path, 0..scene.materials.len(), 0..scene.textures.len())],
    )
}

pub fn load_track(track: &asc_scene::Track) -> Resources {
    let sources: Vec<_> = track
        .manifest
        .models
        .iter()
        .map(|m| (m.path.as_path(), m.materials.clone(), m.textures.clone()))
        .collect();
    load_sources(&track.scene, &sources)
}

fn load_sources(scene: &Scene, sources: &[(&Path, Range<usize>, Range<usize>)]) -> Resources {
    let mut result = Resources {
        textures: Vec::new(),
        material_textures: vec![None; scene.materials.len()],
        diagnostics: Vec::new(),
    };
    let (mut source_bytes, mut decoded_bytes) = (0, 0);
    let mut used = vec![false; scene.materials.len()];
    for draw in &scene.draws {
        used[draw.material] = true;
    }
    for (path, materials, textures) in sources {
        let mut names: HashMap<String, Vec<usize>> = HashMap::new();
        for index in textures.clone() {
            let entry = &scene.textures[index];
            names
                .entry(entry.name.to_ascii_lowercase())
                .or_default()
                .push(index);
        }
        let mut cache = HashMap::new();
        for index in materials.clone().filter(|&i| used[i]) {
            let material = &scene.materials[index];
            let matches = if material.diffuse_refs.len() == 1 {
                names.get(&material.diffuse_refs[0].to_ascii_lowercase())
            } else {
                None
            };
            let Some(matches) = matches.filter(|v| v.len() == 1 && scene.textures[v[0]].active)
            else {
                result.diagnostics.push(format!(
                    "{}: no unique active embedded txDiffuse; diagnostic colour.",
                    format_args!("{} [{}]", material.name, path.display())
                ));
                continue;
            };
            let descriptor_index = matches[0];
            let slot = *cache.entry(descriptor_index).or_insert_with(|| {
                let descriptor = &scene.textures[descriptor_index];
                let decoded = (|| -> Result<dds::Texture, Box<dyn std::error::Error>> {
                    if descriptor.length > 128 * 1024 * 1024 - source_bytes {
                        return Err(
                            "Embedded source textures exceed the 128 MiB scene budget.".into()
                        );
                    }
                    source_bytes += descriptor.length;
                    let mut file = File::open(path)?;
                    let file_length = file.metadata()?.len();
                    if descriptor
                        .offset
                        .checked_add(descriptor.length as u64)
                        .is_none_or(|end| end > file_length)
                    {
                        return Err(
                            "Embedded texture changed or ended before its declared range.".into(),
                        );
                    }
                    file.seek(SeekFrom::Start(descriptor.offset))?;
                    let mut data = vec![0; descriptor.length];
                    file.read_exact(&mut data)?;
                    Ok(dds::decode(&data, 256 * 1024 * 1024 - decoded_bytes)?)
                })();
                match decoded {
                    Ok(texture) => {
                        decoded_bytes += texture.info.decoded_bytes;
                        let id = result.textures.len();
                        result.textures.push(texture);
                        Some(id)
                    }
                    Err(error) => {
                        result.diagnostics.push(format!(
                            "{} [{}]: {error} Diagnostic colour.",
                            descriptor.name,
                            path.display()
                        ));
                        None
                    }
                }
            });
            result.material_textures[index] = slot;
        }
    }
    result
}
