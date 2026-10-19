//! Bounded explicit fixed-frame dynamics workbench; no automatic AC force mapping.
use super::dynamics_config::Spring;
use asc_physics::{LinkLoad, MassProperties, TravelMechanism, TravelState, TravelSystem};
use serde::{Deserialize, Serialize};
use std::{error::Error, fs::File, io::Read, path::Path};

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    schema_version: u32,
    travel_axis: [f64; 3],
    rack_m: [f64; 3],
    mass_kg: f64,
    inertia_rows_kg_m2: [[f64; 3]; 3],
    center_of_mass_local_m: [f64; 3],
    gravity_m_s2: [f64; 3],
    initial_travel_m: f64,
    initial_rate_m_s: f64,
    step_s: f64,
    steps: u32,
    record_every: u32,
    external_force_n: [f64; 3],
    external_torque_about_upright_origin_nm: [f64; 3],
    springs: Vec<Spring>,
}
pub fn run(
    directory: &Path,
    axle: &str,
    configuration: &Path,
    mode: asc_vehicle::data::Mode,
) -> Result<serde_json::Value, Box<dyn Error>> {
    let mut bytes = Vec::new();
    File::open(configuration)?
        .take(16385)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 16384 {
        return Err("Suspension run configuration exceeds 16384 bytes".into());
    }
    let config: Configuration = serde_json::from_slice(&bytes)?;
    if config.schema_version != 1
        || !(1..=10000).contains(&config.steps)
        || config.record_every == 0
        || config.steps.div_ceil(config.record_every) + 1 > 1024
        || config.springs.len() > 16
        || !config.step_s.is_finite()
        || !(1e-6..=1. / 30.).contains(&config.step_s)
    {
        return Err("Expected schema 1, 1..10000 steps, positive record_every, at most 1024 records/16 springs and a 1e-6..1/30 s step".into());
    }
    let source = asc_vehicle::suspension::load(directory, mode)?;
    let selected = source
        .axles
        .get(axle)
        .ok_or("Choose an explicit front or rear axle")?;
    let geometry = match selected.kind.as_str() {
        "DWB" => TravelMechanism::Dwb(asc_session::dwb_geometry(selected, config.travel_axis)?),
        "STRUT" => {
            TravelMechanism::Strut(asc_session::strut_geometry(selected, config.travel_axis)?)
        }
        _ => return Err("Unsupported suspension geometry type".into()),
    };
    let mut system = TravelSystem::new(
        geometry,
        MassProperties::new(config.mass_kg, config.inertia_rows_kg_m2)?,
        config.center_of_mass_local_m,
        config.rack_m,
        config.gravity_m_s2,
        TravelState {
            travel_m: config.initial_travel_m,
            rate_m_s: config.initial_rate_m_s,
        },
    )?;
    for spring in &config.springs {
        system.add_spring(spring.build()?)?;
    }
    let external = LinkLoad::new(
        config.external_force_n,
        config.external_torque_about_upright_origin_nm,
    )?;
    let mut records = Vec::new();
    records.push(serde_json::json!({"step":0,"time_s":0.,"sample":system.sample(external)?}));
    for step in 1..=config.steps {
        let sample = system.step(config.step_s, external)?;
        if step % config.record_every == 0 || step == config.steps {
            records.push(serde_json::json!({"step":step,"time_s":f64::from(step)*config.step_s,"sample":sample}));
        }
    }
    Ok(serde_json::json!({
        "scope":"native_fixed_frame_upright_dynamics", "car":source.car,"axle":axle,"type":selected.kind,
        "data_source":source.data_source,"source_diagnostics":source.diagnostics,
        "explicit_configuration":config,"records":records,
        "fixed_frame":true,"fixed_rack":true,"massless_links":true,
        "automatic_source_mass_or_force_parameters":false,"original_force_mapping_validated":false,
        "vehicle_simulation":false,"simulation_ready":false,"renderer_animation":false,
        "limitations":["Raw source design frame; no side/setup corrections",
            "One massive upright and massless ideal links; no moving chassis or wheel spin",
            "No tyre/contact/FFB forces or automatic AC spring calibration",
            "Explicit midpoint on a bounded local assembly branch; no universal stability guarantee",
            "Constraint reactions exclude direct spring-anchor reactions; travel actuator is a numerical residual"]
    }))
}
