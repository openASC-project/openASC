//! Canonical static spawn transforms; geometric markers do not solve chassis placement.
use asc_config::{Error, Result};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::Path,
};

#[derive(Debug, Clone, Serialize)]
pub struct Diagnostic {
    pub code: &'static str,
    pub source: String,
    pub line: usize,
    pub message: String,
    pub severity: &'static str,
}
#[derive(Debug, Clone, Serialize)]
pub struct Spawn {
    pub name: String,
    pub model: String,
    pub world: [f64; 16],
    pub position: [f64; 3],
    pub left: [f64; 3],
    pub up: [f64; 3],
    pub forward: [f64; 3],
}
#[derive(Serialize)]
pub struct Report {
    pub scope: &'static str,
    pub convention: &'static str,
    pub spawns: Vec<Spawn>,
    pub diagnostics: Vec<Diagnostic>,
    pub has_errors: bool,
}
fn canonical(name: &str) -> bool {
    if name == "AC_HOTLAP_START_0" {
        return true;
    }
    let Some(index) = name
        .strip_prefix("AC_PIT_")
        .or_else(|| name.strip_prefix("AC_START_"))
    else {
        return false;
    };
    index == "0"
        || (index
            .as_bytes()
            .first()
            .is_some_and(|c| (b'1'..=b'9').contains(c))
            && index.bytes().all(|c| c.is_ascii_digit()))
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(x, y)| x * y).sum()
}
pub fn transform(name: &str, model: &str, world: [f64; 16]) -> Result<Spawn> {
    let left = [world[0], world[1], world[2]];
    let up = [world[4], world[5], world[6]];
    let forward = [world[8], world[9], world[10]];
    let axes = [left, up, forward];
    let cross = [
        up[1] * forward[2] - up[2] * forward[1],
        up[2] * forward[0] - up[0] * forward[2],
        up[0] * forward[1] - up[1] * forward[0],
    ];
    if world.iter().any(|v| !v.is_finite())
        || [3, 7, 11].into_iter().any(|i| world[i].abs() > 1e-5)
        || (world[15] - 1.).abs() > 1e-5
        || axes.iter().any(|&a| (dot(a, a).sqrt() - 1.).abs() > 1e-3)
        || [(0, 1), (0, 2), (1, 2)]
            .into_iter()
            .any(|(a, b)| dot(axes[a], axes[b]).abs() > 1e-3)
        || (dot(left, cross) - 1.).abs() > 1e-3
        || up[1] <= 0.5
        || world[12..15].iter().any(|v| v.abs() > 10_000_000.)
    {
        return Err(Error::new(
            "spawn_basis",
            model,
            0,
            &format!(
                "{name}: expected a finite affine, upright, right-handed rigid basis within coordinate limits."
            ),
        ));
    }
    Ok(Spawn {
        name: name.into(),
        model: model.into(),
        world,
        position: [world[12], world[13], world[14]],
        left,
        up,
        forward,
    })
}

pub fn load(config: &Path) -> Result<Report> {
    let sources = crate::sources(config)?;
    let source = sources.config.to_string_lossy();
    let mut spawns = BTreeMap::new();
    let mut seen = BTreeSet::new();
    let mut log = Diagnostics {
        values: Vec::new(),
        bytes: 0,
    };
    let mut nodes = 0usize;
    for message in sources.diagnostics {
        log.push("scene_declaration", &source, 0, message, "warning")?;
    }
    for model in sources.models {
        let parsed = asc_kn5::load_nodes(&model.path).map_err(|e| {
            Error::new(
                "spawn_model",
                &source,
                model.line,
                &format!("{}: {e}", model.file),
            )
        })?;
        nodes += parsed.summary.nodes;
        if nodes > 200_000 {
            return Err(Error::new(
                "spawn_limit",
                &source,
                model.line,
                "Combined models exceed 200000 source nodes.",
            ));
        }
        for node in &parsed.nodes {
            let upper = node.name.to_ascii_uppercase();
            if !node.active
                || !["AC_PIT_", "AC_START_", "AC_HOTLAP_START_"]
                    .iter()
                    .any(|prefix| upper.starts_with(prefix))
            {
                continue;
            }
            if node.kind == 2 || node.kind == 3 {
                if let Some(parent) = node.parent.and_then(|i| parsed.nodes.get(i)) {
                    if parent.kind == 1 && parent.name == node.name && parent.world == node.world {
                        continue;
                    }
                }
            }
            if node.kind == 3 {
                log.push(
                    "spawn_skinned",
                    &model.file,
                    0,
                    format!("{}: standalone skinned markers are unsupported.", node.name),
                    "warning",
                )?;
                continue;
            }
            if !canonical(&node.name) {
                log.push(
                    "spawn_name",
                    &model.file,
                    0,
                    format!(
                        "{}: outside the canonical supported naming subset.",
                        node.name
                    ),
                    "warning",
                )?;
                continue;
            }
            if seen.contains(&node.name) {
                spawns.remove(&node.name);
                log.push(
                    "spawn_duplicate",
                    &model.file,
                    0,
                    format!("{}: occurs more than once and is omitted.", node.name),
                    "error",
                )?;
                continue;
            }
            if seen.len() >= 4096 {
                return Err(Error::new(
                    "spawn_limit",
                    &source,
                    0,
                    "At most 4096 canonical spawn identities are supported.",
                ));
            }
            log.retain(node.name.len() + 128, &source)?;
            seen.insert(node.name.clone());
            let mut world = node.world;
            for (v, shift) in world[12..15].iter_mut().zip(model.position) {
                *v += shift;
            }
            match transform(&node.name, &model.file, world) {
                Ok(value) => {
                    log.retain(node.name.len() * 2 + model.file.len() * 2 + 512, &source)?;
                    spawns.insert(node.name.clone(), value);
                }
                Err(e) => log.push(e.code, &e.source, e.line, e.message, "error")?,
            }
        }
    }
    if spawns.is_empty() {
        log.push(
            "no_spawns",
            &source,
            0,
            "No active unambiguous supported spawn remains.".into(),
            "error",
        )?;
    }
    let diagnostics = log.values;
    let has_errors = diagnostics.iter().any(|d| d.severity == "error");
    Ok(Report {
        scope: "native_static_spawn_transforms",
        convention: "local +X left, +Y up, +Z forward; no Euler conversion",
        spawns: spawns.into_values().collect(),
        diagnostics,
        has_errors,
    })
}

struct Diagnostics {
    values: Vec<Diagnostic>,
    bytes: usize,
}
impl Diagnostics {
    fn retain(&mut self, bytes: usize, source: &str) -> Result<()> {
        self.bytes += bytes;
        if self.bytes > 32 * 1024 * 1024 {
            return Err(Error::new(
                "spawn_limit",
                source,
                0,
                "Retained spawn/diagnostic metadata exceeds 32 MiB.",
            ));
        }
        Ok(())
    }
    fn push(
        &mut self,
        code: &'static str,
        model: &str,
        line: usize,
        message: String,
        severity: &'static str,
    ) -> Result<()> {
        self.retain(model.len() + message.len() + 128, model)?;
        self.values.push(Diagnostic {
            code,
            source: model.into(),
            line,
            message,
            severity,
        });
        Ok(())
    }
}
