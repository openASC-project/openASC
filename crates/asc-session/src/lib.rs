//! Geometric session preparation. Native driving simulation remains unimplemented.
mod body;
mod ground_contact;
mod kinematics;
use asc_scene::spawns::{Diagnostic, Spawn};
pub use body::body_with_spawn_basis;
pub use ground_contact::{TrackContact, TrackFace, face_contact, face_contact_with_friction};
pub use kinematics::{
    dwb_geometry, dwb_geometry_with_reflection, strut_geometry, strut_geometry_with_reflection,
};
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

#[derive(Serialize)]
pub struct Support {
    pub position: [f64; 3],
    pub distance: f64,
    pub normal: [f64; 3],
    pub mesh: String,
    pub model: String,
    pub triangle: usize,
    pub surface: asc_ground::surfaces::Surface,
}
impl From<asc_ground::Hit<'_>> for Support {
    fn from(hit: asc_ground::Hit<'_>) -> Self {
        Self {
            position: hit.position,
            distance: hit.distance,
            normal: hit.normal,
            mesh: hit.mesh.into(),
            model: hit.model.into(),
            triangle: hit.triangle,
            surface: hit.surface.clone(),
        }
    }
}
#[derive(Serialize)]
pub struct Contact {
    pub origin: [f64; 3],
    pub hit: Option<Support>,
}
#[derive(Serialize)]
pub struct Placement {
    pub scope: &'static str,
    pub config: PathBuf,
    pub car: String,
    pub compound: u32,
    pub spawn: Spawn,
    pub contacts: BTreeMap<String, Contact>,
    pub all_wheels_supported: bool,
    pub simulation_ready: bool,
    pub diagnostics: Vec<Diagnostic>,
    pub has_errors: bool,
}

pub fn place(
    car: &Path,
    config: &Path,
    surfaces: &[&Path],
    spawn: &str,
    mode: asc_vehicle::data::Mode,
    compound: Option<u32>,
) -> asc_vehicle::Result<Placement> {
    let vehicle = asc_vehicle::load(car, mode, compound)?;
    let report = asc_scene::spawns::load(config)?;
    let selected=report.spawns.into_iter().find(|s|s.name==spawn).ok_or_else(||asc_config::Error::new("spawn_missing","<placement>",0,&format!("Selected {spawn} is absent, ambiguous or unsupported; inspect spawns for full diagnostics.")))?;
    let ground = asc_ground::load(config, surfaces)?;
    let mut diagnostics = report.diagnostics;
    for value in vehicle.diagnostics {
        diagnostics.push(Diagnostic {
            code: value.code,
            source: value.source,
            line: value.line,
            message: value.message,
            severity: value.severity,
        });
    }
    for value in ground.diagnostics() {
        diagnostics.push(Diagnostic {
            code: value.code,
            source: value.source.clone(),
            line: value.line,
            message: value.message.clone(),
            severity: "warning",
        });
    }
    let mut contacts = BTreeMap::new();
    for (name, offset) in vehicle.wheel_offsets_m {
        // Marker footprint only: BASEY and tyre radius are not a solved chassis height.
        let origin = std::array::from_fn(|i| {
            selected.position[i] + selected.left[i] * offset[0] + selected.forward[i] * offset[2]
        });
        let hit = ground.probe(origin, 20.)?.map(Support::from);
        if hit.is_none() {
            diagnostics.push(Diagnostic {
                code: "placement_no_support",
                source: selected.model.clone(),
                line: 0,
                message: format!(
                    "{name}: no ground within 20 metres below the selected spawn footprint."
                ),
                severity: "error",
            });
        }
        contacts.insert(name, Contact { origin, hit });
    }
    diagnostics.push(Diagnostic{code:"placement_scope",source:selected.model.clone(),line:0,message:"Geometric wheel-footprint preflight only; chassis equilibrium, graphics placement, rigid-body/tyre forces and driving are not solved.".into(),severity:"warning"});
    let mut seen = BTreeSet::new();
    diagnostics.retain(|d| {
        seen.insert((
            d.code,
            d.source.clone(),
            d.line,
            d.message.clone(),
            d.severity,
        ))
    });
    let has_errors = diagnostics.iter().any(|d| d.severity == "error");
    Ok(Placement {
        scope: "native_geometric_vehicle_placement",
        config: config.into(),
        car: vehicle.car,
        compound: vehicle.compound,
        spawn: selected,
        all_wheels_supported: contacts.values().all(|c| c.hit.is_some()),
        contacts,
        simulation_ready: false,
        diagnostics,
        has_errors,
    })
}
