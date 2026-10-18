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
    pub material_layers: Vec<Option<[usize; 5]>>,
    pub diagnostics: Vec<String>,
    pub allow_bc: bool,
}

pub fn load(scene: &Scene, path: &Path) -> Resources {
    load_sources(
        scene,
        &[(path, 0..scene.materials.len(), 0..scene.textures.len())],
        false,
    )
}

pub fn load_track(track: &asc_scene::Track) -> Resources {
    let sources: Vec<_> = track
        .manifest
        .models
        .iter()
        .map(|m| (m.path.as_path(), m.materials.clone(), m.textures.clone()))
        .collect();
    load_sources(&track.scene, &sources, false)
}

pub fn load_prepared(scene: &Scene, path: &Path) -> Resources {
    load_sources(
        scene,
        &[(path, 0..scene.materials.len(), 0..scene.textures.len())],
        true,
    )
}
pub fn load_track_prepared(track: &asc_scene::Track) -> Resources {
    let sources: Vec<_> = track
        .manifest
        .models
        .iter()
        .map(|m| (m.path.as_path(), m.materials.clone(), m.textures.clone()))
        .collect();
    load_sources(&track.scene, &sources, true)
}

fn load_sources(
    scene: &Scene,
    sources: &[(&Path, Range<usize>, Range<usize>)],
    prepared: bool,
) -> Resources {
    let mut result = Resources {
        textures: Vec::new(),
        material_textures: vec![None; scene.materials.len()],
        material_layers: vec![None; scene.materials.len()],
        diagnostics: Vec::new(),
        allow_bc: prepared,
    };
    let (mut source_bytes, mut retained_bytes) = (0, 0);
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
        for phase in 0..2 {
            for index in materials.clone().filter(|&i| used[i]) {
                let material = &scene.materials[index];
                let layered = if phase == 0 {
                    false
                } else {
                    match crate::scene::material::layers(material) {
                        Ok(value) => value.is_some(),
                        Err(error) => {
                            result
                                .diagnostics
                                .push(format!("{}: {error}", material.name));
                            false
                        }
                    }
                };
                if phase == 1 && !layered {
                    continue;
                }
                let mut slots = [None; 6];
                slots[0] = result.material_textures[index];
                for (channel, output_slot) in slots
                    .iter_mut()
                    .enumerate()
                    .take(if layered { 6 } else { 1 })
                    .skip(phase)
                {
                    let sampler = if channel == 0 {
                        "txDiffuse"
                    } else {
                        crate::scene::material::LAYER_SAMPLERS[channel - 1]
                    };
                    let references: Vec<&str> = if channel == 0 {
                        material.diffuse_refs.iter().map(String::as_str).collect()
                    } else {
                        material
                            .mappings
                            .iter()
                            .filter(|m| m.sampler == sampler)
                            .map(|m| m.texture.as_str())
                            .collect()
                    };
                    let data_map = channel == 1;
                    let matches = if references.len() == 1 {
                        names.get(&references[0].to_ascii_lowercase())
                    } else {
                        None
                    };
                    let Some(matches) =
                        matches.filter(|v| v.len() == 1 && scene.textures[v[0]].active)
                    else {
                        result.diagnostics.push(format!(
                            "{}: no unique active embedded {sampler}; diagnostic colour.",
                            format_args!("{} [{}]", material.name, path.display())
                        ));
                        continue;
                    };
                    let descriptor_index = matches[0];
                    let slot = *cache
                        .entry((descriptor_index, data_map))
                        .or_insert_with(|| {
                            let descriptor = &scene.textures[descriptor_index];
                            let decoded =
                                (|| -> Result<dds::Texture, Box<dyn std::error::Error>> {
                                    if descriptor.length > 128 * 1024 * 1024 - source_bytes {
                                        return Err(
                                    "Embedded source textures exceed the 128 MiB scene budget."
                                        .into(),
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
                                    "Embedded texture changed or ended before its declared range."
                                        .into(),
                                );
                                    }
                                    file.seek(SeekFrom::Start(descriptor.offset))?;
                                    let mut data = vec![0; descriptor.length];
                                    file.read_exact(&mut data)?;
                                    Ok(if prepared {
                                        if data_map {
                                            dds::prepare_data(
                                                &data,
                                                256 * 1024 * 1024 - retained_bytes,
                                            )?
                                        } else {
                                            dds::prepare(&data, 256 * 1024 * 1024 - retained_bytes)?
                                        }
                                    } else {
                                        if data_map {
                                            dds::decode_data(
                                                &data,
                                                256 * 1024 * 1024 - retained_bytes,
                                            )?
                                        } else {
                                            dds::decode(&data, 256 * 1024 * 1024 - retained_bytes)?
                                        }
                                    })
                                })();
                            match decoded {
                                Ok(texture) => {
                                    retained_bytes += texture.retained_bytes();
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
                    *output_slot = slot;
                }
                result.material_textures[index] = slots[0];
                if layered {
                    if slots.iter().all(Option::is_some) {
                        result.material_layers[index] =
                            Some(std::array::from_fn(|i| slots[i + 1].unwrap()));
                    } else {
                        result.diagnostics.push(format!(
                            "{}: incomplete multilayer resources; diffuse-only fallback.",
                            material.name
                        ));
                    }
                }
            }
        }
    }
    result
}
