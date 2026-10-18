//! Confined static models.ini assembly. Nonzero model rotations remain unvalidated.
pub mod spawns;
use asc_config::{Error, Result, Section, finite_number, read_ini};
use asc_kn5::{Scene, Summary};
use serde::Serialize;
use std::{
    ops::Range,
    path::{Path, PathBuf},
};

const MAX_MODELS: usize = 256;
const MAX_GEOMETRY: usize = 256 * 1024 * 1024;
const MAX_COORDINATE: f64 = 10_000_000.;

#[derive(Serialize)]
pub struct Origin {
    pub section: String,
    pub file: String,
    #[serde(skip)]
    pub path: PathBuf,
    pub line: usize,
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub version: u32,
    pub materials: Range<usize>,
    pub textures: Range<usize>,
}
#[derive(Serialize)]
pub struct Manifest {
    pub scope: &'static str,
    pub config: PathBuf,
    pub models: Vec<Origin>,
    pub diagnostics: Vec<String>,
}
pub struct Track {
    pub scene: Scene,
    pub manifest: Manifest,
}

fn vector(section: &Section, key: &str, source: &str) -> Result<[f64; 3]> {
    let value = section.get(key).ok_or_else(|| {
        Error::new(
            "missing_transform",
            source,
            section.line,
            &format!("[{}] requires explicit {key}.", section.name),
        )
    })?;
    let fields: Vec<_> = value.text.split(',').collect();
    if fields.len() != 3 {
        return Err(Error::new(
            "invalid_vector",
            source,
            value.line,
            "Expected exactly three comma-separated numbers.",
        ));
    }
    let mut result = [0.; 3];
    for (out, field) in result.iter_mut().zip(fields) {
        *out = finite_number(field, source, value.line)?;
    }
    Ok(result)
}
fn confined(root: &Path, relative: &str, source: &str, line: usize) -> Result<PathBuf> {
    let portable = relative.replace('\\', "/");
    // Check both Windows and Unix path syntax on every host, including drive-relative paths.
    if portable.starts_with('/') || portable.contains(':') || portable.contains('\0') {
        return Err(Error::new(
            "resource_path",
            source,
            line,
            "Expected a relative resource path without a drive or stream.",
        ));
    }
    if portable.split('/').any(|part| part == "..") {
        return Err(Error::new(
            "resource_escape",
            source,
            line,
            "Parent traversal is unsupported.",
        ));
    }
    let path = root
        .join(portable)
        .canonicalize()
        .map_err(|e| Error::new("resource_io", source, line, &e.to_string()))?;
    if !path.starts_with(root) {
        return Err(Error::new(
            "resource_escape",
            source,
            line,
            "Resource resolves outside the selected track directory.",
        ));
    }
    if !path.is_file() {
        return Err(Error::new(
            "model_file",
            source,
            line,
            "Expected an existing KN5 file.",
        ));
    }
    Ok(path)
}
fn shifted(
    bounds: Option<[[f32; 3]; 2]>,
    position: [f64; 3],
    source: &str,
    line: usize,
) -> Result<Option<[[f32; 3]; 2]>> {
    bounds
        .map(|mut b| {
            for row in &mut b {
                for (v, p) in row.iter_mut().zip(position) {
                    *v = coordinate(f64::from(*v) + p, source, line)?;
                }
            }
            Ok(b)
        })
        .transpose()
}
fn coordinate(value: f64, source: &str, line: usize) -> Result<f32> {
    if !value.is_finite() || value.abs() > MAX_COORDINATE {
        return Err(Error::new(
            "scene_range",
            source,
            line,
            "World coordinates exceed +/-10000000 metres.",
        ));
    }
    Ok(value as f32)
}
fn union(into: &mut Option<[[f32; 3]; 2]>, bounds: Option<[[f32; 3]; 2]>) {
    if let Some(b) = bounds {
        if let Some(a) = into {
            for axis in 0..3 {
                a[0][axis] = a[0][axis].min(b[0][axis]);
                a[1][axis] = a[1][axis].max(b[1][axis]);
            }
        } else {
            *into = Some(b);
        }
    }
}

pub struct ModelSource {
    pub section: String,
    pub file: String,
    pub path: PathBuf,
    pub line: usize,
    pub position: [f64; 3],
    pub rotation: [f64; 3],
}
pub struct Sources {
    pub config: PathBuf,
    pub models: Vec<ModelSource>,
    pub diagnostics: Vec<String>,
}
/// Resolve and validate declarations without opening model geometry.
pub fn sources(config: &Path) -> Result<Sources> {
    let document = read_ini(config)?;
    let source = &document.source;
    let canonical = config
        .canonicalize()
        .map_err(|e| Error::new("scene_io", source, 0, &e.to_string()))?;
    let root = canonical.parent().ok_or_else(|| {
        Error::new(
            "scene_root",
            source,
            0,
            "Configuration needs a parent directory.",
        )
    })?;
    let mut manifest = Sources {
        config: canonical.clone(),
        models: Vec::new(),
        diagnostics: Vec::new(),
    };
    for section in document.sections {
        if !section
            .name
            .strip_prefix("MODEL_")
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|c| c.is_ascii_digit()))
        {
            manifest.diagnostics.push(format!(
                "{source}:{}: [{}] is not loaded; only static MODEL_n declarations are supported.",
                section.line, section.name
            ));
            continue;
        }
        if manifest.models.len() >= MAX_MODELS {
            return Err(Error::new(
                "scene_limit",
                source,
                section.line,
                "Scene exceeds 256 models.",
            ));
        }
        let file = section.get("FILE").ok_or_else(|| {
            Error::new("model_file", source, section.line, "MODEL_n requires FILE.")
        })?;
        if !file.text.to_ascii_lowercase().ends_with(".kn5") {
            return Err(Error::new(
                "model_file",
                source,
                file.line,
                "FILE must point to a KN5 file.",
            ));
        }
        let position = vector(&section, "POSITION", source)?;
        let rotation = vector(&section, "ROTATION", source)?;
        if rotation.iter().any(|v| *v != 0.) {
            return Err(Error::new(
                "model_rotation_unsupported",
                source,
                section.get("ROTATION").unwrap().line,
                "Nonzero models.ini rotation convention is unvalidated; refusing to silently misplace a model.",
            ));
        }
        for v in position {
            coordinate(v, source, section.get("POSITION").unwrap().line)?;
        }
        for (key, value) in &section.values {
            if !["FILE", "POSITION", "ROTATION"].contains(&key.as_str()) {
                manifest.diagnostics.push(format!(
                    "{source}:{}: [{}] {key} is not interpreted.",
                    value.line, section.name
                ));
            }
        }
        let path = confined(root, &file.text, source, file.line)?;
        manifest.models.push(ModelSource {
            section: section.name.clone(),
            file: file.text.clone(),
            path,
            line: file.line,
            position,
            rotation,
        });
    }
    if manifest.models.is_empty() {
        return Err(Error::new(
            "no_models",
            source,
            0,
            "No static MODEL_n declarations found.",
        ));
    }
    Ok(manifest)
}

pub fn load(config: &Path) -> Result<Track> {
    let declared = sources(config)?;
    let source_name = declared.config.to_string_lossy().into_owned();
    let source = source_name.as_str();
    let mut manifest = Manifest {
        scope: "native_static_models",
        config: declared.config,
        models: Vec::new(),
        diagnostics: declared.diagnostics,
    };
    let mut metadata_bytes = 0usize;
    let mut scene = Scene {
        // A composed scene is not a KN5 file. Actual format versions belong to each Origin.
        summary: Summary { version: 0, nodes: 0, static_meshes: 0, skinned_meshes: 0, textures: 0, materials: 0, structural_triangles: 0, drawn_meshes: 0, drawn_triangles: 0, bounds: None, road_focus_bounds: None, diagnostics: vec!["Composed static scene: version 0 is a composition marker; see models for actual KN5 versions.".into()] },
        vertices: Vec::new(), indices: Vec::new(), materials: Vec::new(), textures: Vec::new(), draws: Vec::new(),
    };
    for entry in declared.models {
        let position = entry.position;
        let rotation = entry.rotation;
        let mut model = asc_kn5::load(&entry.path).map_err(|e| {
            Error::new(
                "scene_model",
                source,
                entry.line,
                &format!("{}: {e}", entry.file),
            )
        })?;
        let geometry = (scene.vertices.len() + model.vertices.len()) * size_of::<asc_kn5::Vertex>()
            + (scene.indices.len() + model.indices.len()) * 4;
        metadata_bytes += model.textures.iter().map(|t| t.name.len()).sum::<usize>()
            + model
                .materials
                .iter()
                .map(|m| {
                    m.name.len()
                        + m.shader.len()
                        + m.diffuse_refs.iter().map(String::len).sum::<usize>()
                        + m.properties
                            .iter()
                            .map(|(k, v)| k.len() + size_of_val(v))
                            .sum::<usize>()
                        + m.mappings
                            .iter()
                            .map(|v| v.sampler.len() + v.texture.len())
                            .sum::<usize>()
                })
                .sum::<usize>()
            + model
                .summary
                .diagnostics
                .iter()
                .map(String::len)
                .sum::<usize>();
        if geometry > MAX_GEOMETRY
            || scene.summary.nodes + model.summary.nodes > 200_000
            || scene.materials.len() + model.materials.len() > 100_000
            || scene.textures.len() + model.textures.len() > 100_000
            || metadata_bytes > 32 * 1024 * 1024
        {
            return Err(Error::new(
                "scene_limit",
                source,
                entry.line,
                "Combined scene exceeds geometry (256 MiB), metadata payload (32 MiB), node (200000), material or texture (100000) budgets.",
            ));
        }
        let origin = Origin {
            section: entry.section.clone(),
            file: entry.file.clone(),
            path: entry.path,
            line: entry.line,
            position,
            rotation,
            version: model.summary.version,
            materials: scene.materials.len()..scene.materials.len() + model.materials.len(),
            textures: scene.textures.len()..scene.textures.len() + model.textures.len(),
        };
        let base_vertex = scene.vertices.len() as u32;
        let base_index = scene.indices.len() as u32;
        for vertex in &mut model.vertices {
            for (v, p) in vertex.position.iter_mut().zip(position) {
                *v = coordinate(f64::from(*v) + p, source, entry.line)?;
            }
        }
        for draw in &mut model.draws {
            draw.start += base_index;
            draw.material += origin.materials.start;
            for (v, p) in draw.center.iter_mut().zip(position) {
                *v = coordinate(f64::from(*v) + p, source, entry.line)?;
            }
        }
        union(
            &mut scene.summary.bounds,
            shifted(model.summary.bounds, position, source, entry.line)?,
        );
        union(
            &mut scene.summary.road_focus_bounds,
            shifted(
                model.summary.road_focus_bounds,
                position,
                source,
                entry.line,
            )?,
        );
        scene.summary.nodes += model.summary.nodes;
        scene.summary.static_meshes += model.summary.static_meshes;
        scene.summary.skinned_meshes += model.summary.skinned_meshes;
        scene.summary.structural_triangles += model.summary.structural_triangles;
        scene.summary.drawn_meshes += model.summary.drawn_meshes;
        scene.summary.drawn_triangles += model.summary.drawn_triangles;
        scene.summary.diagnostics.extend(
            model
                .summary
                .diagnostics
                .into_iter()
                .map(|d| format!("{}: {d}", entry.file)),
        );
        scene.vertices.extend(model.vertices);
        scene
            .indices
            .extend(model.indices.into_iter().map(|i| i + base_vertex));
        scene.draws.extend(model.draws);
        scene.materials.extend(model.materials);
        scene.textures.extend(model.textures);
        manifest.models.push(origin);
    }
    if manifest.models.is_empty() {
        return Err(Error::new(
            "no_models",
            source,
            0,
            "No static MODEL_n declarations found.",
        ));
    }
    scene.summary.materials = scene.materials.len();
    scene.summary.textures = scene.textures.len();
    scene
        .summary
        .diagnostics
        .extend(manifest.diagnostics.iter().cloned());
    Ok(Track { scene, manifest })
}
