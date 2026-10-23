//! Bounded source-based body/WALL geometry preflight; no physical state is advanced.
use asc_vehicle::{Result, data::Mode};
use glam::{DMat3, DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::{io::Read, path::Path};
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Probe {
    position_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    margin_m: f64,
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
    probes: Vec<Probe>,
}
pub fn run(path: &Path) -> Result<serde_json::Value> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(8193)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 8192 {
        return Err("wall-query config exceeds 8192 bytes".into());
    }
    let c: Configuration = serde_json::from_slice(&bytes)?;
    if !(1..=64).contains(&c.probes.len()) {
        return Err("wall-query requires 1..64 probes".into());
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
    let mut spawn_diagnostics = Vec::new();
    let (origin, rotation) = if let Some(name) = &c.spawn {
        let report = asc_scene::spawns::load(&track)?;
        let s = report
            .spawns
            .iter()
            .find(|s| s.name == *name)
            .ok_or("Selected wall-query spawn is absent, ambiguous or unsupported")?;
        let basis = DMat3::from_cols(
            DVec3::from_array(s.left),
            DVec3::from_array(s.up),
            DVec3::from_array(s.forward),
        );
        if (basis.determinant() - 1.).abs() > 1e-6
            || (basis.transpose() * basis - DMat3::IDENTITY)
                .to_cols_array()
                .iter()
                .any(|v| v.abs() > 1e-6)
        {
            return Err("wall-query spawn must provide a rigid right-handed basis".into());
        }
        let value = (
            DVec3::from_array(s.position),
            DQuat::from_mat3(&basis).normalize(),
        );
        spawn_diagnostics = report.diagnostics;
        value
    } else {
        (DVec3::ZERO, DQuat::IDENTITY)
    };
    let mut count = 0;
    let mut records = Vec::new();
    for probe in &c.probes {
        let p = DVec3::from_array(probe.position_m);
        let q = DQuat::from_array(probe.orientation_xyzw);
        if !p.is_finite()
            || p.abs().max_element() > 1e7
            || !q.is_finite()
            || (q.length_squared() - 1.).abs() > 1e-8
        {
            return Err("wall-query probes require bounded positions and unit quaternions".into());
        }
        let world_position = origin + rotation * p;
        let world_rotation = rotation * q.normalize();
        let body = collider.at_pose(world_position.to_array(), world_rotation.to_array())?;
        let contacts = walls.contacts(&body, probe.margin_m)?;
        count += contacts.len();
        if count > 4096 {
            return Err("wall-query report exceeds 4096 total triangle contacts".into());
        }
        records.push(serde_json::json!({"position_world_m":world_position.to_array(),"orientation_xyzw":world_rotation.to_array(),"margin_m":probe.margin_m,"contacts":contacts}));
    }
    let has_source_errors = spawn_diagnostics.iter().any(|d| d.severity == "error");
    Ok(
        serde_json::json!({"scope":"native_collider_wall_geometry_preflight","explicit_configuration":c,"body_shape":"enclosing_chassis_frame_box","collider_proxy":collider,"walls":walls.summary(),"spawn":c.spawn,"spawn_diagnostics":spawn_diagnostics,"has_source_errors":has_source_errors,"records":records,
        "collision_response":false,"continuous_collision_detection":false,"original_collision_shape_validated":false,"simulation_ready":false}),
    )
}
