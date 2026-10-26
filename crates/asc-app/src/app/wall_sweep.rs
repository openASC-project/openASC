//! Bounded continuous prescribed-path collider/WALL query, independent of force dynamics.
use asc_ground::{BoxMotion, WallSweepSettings};
use asc_vehicle::{Result, data::Mode};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::{io::Read, path::Path};
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Motion {
    reference_start_m: [f64; 3],
    reference_end_m: [f64; 3],
    orientation_start_xyzw: [f64; 4],
    orientation_end_xyzw: [f64; 4],
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    car_directory: String,
    models_ini: String,
    data: String,
    compound: Option<u32>,
    spawn: Option<String>,
    model_origin_in_chassis_m: [f64; 3],
    model_orientation_in_chassis_xyzw: [f64; 4],
    #[serde(default)]
    settings: WallSweepSettings,
    #[serde(default)]
    allow_certified_departures: bool,
    motions: Vec<Motion>,
}
pub fn run(path: &Path) -> Result<serde_json::Value> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(8193)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 8192 {
        return Err("wall-sweep config exceeds 8192 bytes".into());
    }
    let c: Configuration = serde_json::from_slice(&bytes)?;
    if !(1..=32).contains(&c.motions.len()) {
        return Err("wall-sweep requires 1..32 prescribed motions".into());
    }
    let mode = match c.data.as_str() {
        "auto" => Mode::Auto,
        "packed" => Mode::Packed,
        "unpacked" => Mode::Unpacked,
        _ => return Err("Choose auto, packed or unpacked data".into()),
    };
    let parent = path.parent().unwrap_or(Path::new("."));
    let track = parent.join(&c.models_ini);
    let collider = asc_session::ColliderBox::load(
        &parent.join(&c.car_directory),
        mode,
        c.compound,
        c.model_origin_in_chassis_m,
        c.model_orientation_in_chassis_xyzw,
    )?;
    let walls = asc_ground::WallIndex::load(&track)?;
    let (origin, basis, spawn_diagnostics) =
        super::wall_query::source_pose(&track, c.spawn.as_deref())?;
    let mut records = Vec::new();
    let mut contacts = 0;
    for m in &c.motions {
        // Validate the original inputs before composition: do not normalize an invalid quaternion.
        BoxMotion::new(
            m.reference_start_m,
            m.reference_end_m,
            m.orientation_start_xyzw,
            m.orientation_end_xyzw,
            collider.center_in_chassis_m,
            collider.half_extent_m,
        )?;
        let motion = BoxMotion::new(
            (origin + basis * DVec3::from_array(m.reference_start_m)).to_array(),
            (origin + basis * DVec3::from_array(m.reference_end_m)).to_array(),
            (basis * DQuat::from_array(m.orientation_start_xyzw)).to_array(),
            (basis * DQuat::from_array(m.orientation_end_xyzw)).to_array(),
            collider.center_in_chassis_m,
            collider.half_extent_m,
        )?;
        let sweep = if c.allow_certified_departures {
            walls.sweep_allowing_certified_departures(&motion, c.settings)?
        } else {
            walls.sweep(&motion, c.settings)?
        };
        contacts += sweep.contacts.len() + sweep.certified_departures.len();
        if contacts > 4096 {
            return Err("wall-sweep exceeds 4096 total triangle contacts/certificates".into());
        }
        records.push(serde_json::json!({"motion":m,"sweep":sweep}));
    }
    let has_source_errors = spawn_diagnostics.iter().any(|d| d.severity == "error");
    Ok(
        serde_json::json!({"scope":"native_prescribed_collider_wall_sweep",
        "explicit_configuration":c,"collider_proxy":collider,"walls":walls.summary(),
        "spawn_diagnostics":spawn_diagnostics,"has_source_errors":has_source_errors,"records":records,
        "motion_interpolation":"linear reference translation and constant-speed shortest quaternion arc; rotating local collider centre",
        "event_kind":"conservative SAT proximity", "exact_time_of_impact":false,
        "collision_response":false,"native_driving_collision_response":false,
        "original_collision_shape_validated":false,"simulation_ready":false}),
    )
}
