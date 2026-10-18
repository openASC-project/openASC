use asc_config::{Error, Result, finite_number, read_ini};
use serde::Serialize;
use std::{collections::BTreeMap, path::Path};

#[derive(Debug, Clone, Serialize)]
pub struct Surface {
    pub key: String,
    pub friction: f64,
    pub damping: f64,
    pub valid_track: bool,
    pub pitlane: bool,
    pub source: String,
    pub section: String,
    pub line: usize,
    pub parameters: BTreeMap<String, serde_json::Value>,
}

/// Later files replace a complete same-KEY definition, never individual fields.
pub fn load(paths: &[&Path]) -> Result<(BTreeMap<String, Surface>, Vec<Error>)> {
    if paths.len() > 16 {
        return Err(Error::new(
            "surface_limit",
            "<surfaces>",
            0,
            "At most 16 surface definition files are supported.",
        ));
    }
    let mut merged = BTreeMap::new();
    let mut diagnostics = Vec::new();
    for path in paths {
        let source = path.to_string_lossy();
        if !std::fs::metadata(path)
            .map_err(|e| Error::new("surface_io", &source, 0, &e.to_string()))?
            .is_file()
        {
            return Err(Error::new(
                "surface_io",
                &source,
                0,
                "Expected a regular INI file.",
            ));
        }
        let document = read_ini(path)?;
        let mut parsed = BTreeMap::new();
        for section in document.sections {
            if !section
                .name
                .strip_prefix("SURFACE_")
                .is_some_and(|v| !v.is_empty() && v.bytes().all(|c| c.is_ascii_digit()))
            {
                diagnostics.push(Error::new(
                    "surface_section",
                    &source,
                    section.line,
                    "Only SURFACE_n definitions are interpreted.",
                ));
                continue;
            }
            let required = |key: &str| {
                section.get(key).ok_or_else(|| {
                    Error::new(
                        "surface_field",
                        &source,
                        section.line,
                        &format!("[{}] requires {key}.", section.name),
                    )
                })
            };
            let key = required("KEY")?.text.to_ascii_uppercase();
            if !key.as_bytes().first().is_some_and(u8::is_ascii_alphabetic)
                || !key
                    .bytes()
                    .all(|c| c.is_ascii_alphanumeric() || b"_-".contains(&c))
            {
                return Err(Error::new(
                    "surface_key",
                    &source,
                    required("KEY")?.line,
                    "Expected an ASCII letter followed by letters, digits, underscores or hyphens.",
                ));
            }
            if parsed.contains_key(&key) {
                return Err(Error::new(
                    "surface_duplicate",
                    &source,
                    required("KEY")?.line,
                    "Duplicate surface KEY in the same file.",
                ));
            }
            let number = |key: &str| {
                let v = required(key)?;
                let n = finite_number(&v.text, &source, v.line)?;
                if n < 0. {
                    return Err(Error::new(
                        "surface_range",
                        &source,
                        v.line,
                        "FRICTION and DAMPING must be nonnegative.",
                    ));
                }
                Ok(n)
            };
            let flag = |key: &str| match required(key)?.text.as_str() {
                "0" => Ok(false),
                "1" => Ok(true),
                _ => Err(Error::new(
                    "surface_flag",
                    &source,
                    required(key)?.line,
                    "Expected 0 or 1.",
                )),
            };
            let mut surface = Surface {
                key: key.clone(),
                friction: number("FRICTION")?,
                damping: number("DAMPING")?,
                valid_track: flag("IS_VALID_TRACK")?,
                pitlane: flag("IS_PITLANE")?,
                source: source.to_string(),
                section: section.name.clone(),
                line: section.line,
                parameters: BTreeMap::new(),
            };
            for (name, value) in &section.values {
                let parsed = match name.as_str() {
                    "FRICTION" | "DAMPING" | "WAV_PITCH" | "DIRT_ADDITIVE" | "BLACK_FLAG_TIME"
                    | "SIN_HEIGHT" | "SIN_LENGTH" | "VIBRATION_GAIN" | "VIBRATION_LENGTH" => {
                        serde_json::json!(finite_number(&value.text, &source, value.line)?)
                    }
                    "IS_VALID_TRACK" | "IS_PITLANE" => serde_json::json!(flag(name)?),
                    _ => {
                        if !["KEY", "WAV", "FF_EFFECT"].contains(&name.as_str()) {
                            diagnostics.push(Error::new(
                                "surface_field_unsupported",
                                &source,
                                value.line,
                                &format!("{name} is retained without interpretation."),
                            ));
                        }
                        serde_json::json!(value.text)
                    }
                };
                surface.parameters.insert(name.clone(), parsed);
            }
            parsed.insert(key, surface);
            if parsed.len() > 4096 {
                return Err(Error::new(
                    "surface_limit",
                    &source,
                    section.line,
                    "At most 4096 surface keys are supported.",
                ));
            }
        }
        merged.extend(parsed);
        if merged.len() > 4096 {
            return Err(Error::new(
                "surface_limit",
                &source,
                0,
                "At most 4096 combined surface keys are supported.",
            ));
        }
    }
    if merged.is_empty() {
        return Err(Error::new(
            "no_surfaces",
            "<surfaces>",
            0,
            "No explicit surface definitions loaded.",
        ));
    }
    Ok((merged, diagnostics))
}

pub fn mesh_surface<'a>(
    name: &str,
    surfaces: &'a BTreeMap<String, Surface>,
) -> Result<Option<&'a Surface>> {
    let name = name.to_ascii_uppercase();
    let prefix = name.bytes().take_while(u8::is_ascii_digit).count();
    if !name[..prefix].bytes().any(|c| c != b'0') {
        return Ok(None);
    }
    let mut matches = surfaces
        .values()
        .filter(|s| name[prefix..].starts_with(&s.key));
    let first = matches.next().ok_or_else(|| {
        Error::new(
            "surface_unmatched",
            "<mesh>",
            0,
            "Physical mesh has no supported explicit surface key.",
        )
    })?;
    if matches.next().is_some() {
        return Err(Error::new(
            "surface_ambiguous",
            "<mesh>",
            0,
            "Physical mesh matches multiple surface keys.",
        ));
    }
    Ok(Some(first))
}
