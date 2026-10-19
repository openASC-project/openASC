//! Explicit coupled chassis workbench, with caller-supplied frame and mass mapping.
use super::dynamics_config::Spring;
use asc_physics::{
    Body, ChassisSystem, LinkPose, MassProperties, MountedUpright, TravelMechanism, TravelState,
    Wrench,
};
use serde::{Deserialize, Serialize};
use std::{error::Error, fs::File, io::Read, path::Path};

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Chassis {
    mass_kg: f64,
    inertia_rows_kg_m2: [[f64; 3]; 3],
    position_world_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    velocity_world_m_s: [f64; 3],
    angular_velocity_world_rad_s: [f64; 3],
    external_force_world_n: [f64; 3],
    external_torque_about_com_world_nm: [f64; 3],
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Upright {
    car_directory: String,
    axle: String,
    data: String,
    travel_axis: [f64; 3],
    rack_m: [f64; 3],
    frame_position_in_chassis_m: [f64; 3],
    frame_orientation_xyzw: [f64; 4],
    mass_kg: f64,
    inertia_rows_kg_m2: [[f64; 3]; 3],
    center_of_mass_local_m: [f64; 3],
    initial_travel_m: f64,
    initial_rate_m_s: f64,
    springs: Vec<Spring>,
    external_force_world_n: [f64; 3],
    external_torque_about_com_world_nm: [f64; 3],
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    schema_version: u32,
    chassis: Chassis,
    gravity_world_m_s2: [f64; 3],
    uprights: Vec<Upright>,
    step_s: f64,
    steps: u32,
    record_every: u32,
}
pub fn run(configuration: &Path) -> Result<serde_json::Value, Box<dyn Error>> {
    let mut bytes = Vec::new();
    File::open(configuration)?
        .take(65537)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 65536 {
        return Err("Chassis run configuration exceeds 65536 bytes".into());
    }
    let config: Configuration = serde_json::from_slice(&bytes)?;
    if config.schema_version != 1
        || !(1..=4).contains(&config.uprights.len())
        || config.uprights.iter().any(|c| c.springs.len() > 16)
        || !(1..=10000).contains(&config.steps)
        || config.record_every == 0
        || config.steps.div_ceil(config.record_every) + 1 > 256
        || !config.step_s.is_finite()
        || !(1e-6..=1. / 30.).contains(&config.step_s)
    {
        return Err("Expected schema 1, 1..4 uprights, at most 16 springs each, 1..10000 steps, positive record_every, at most 256 records and a 1e-6..1/30 s step".into());
    }
    let c = &config.chassis;
    let body = Body::new(
        MassProperties::new(c.mass_kg, c.inertia_rows_kg_m2)?,
        c.position_world_m,
        c.orientation_xyzw,
        c.velocity_world_m_s,
        c.angular_velocity_world_rad_s,
    )?;
    let chassis_load = Wrench::new(
        c.external_force_world_n,
        c.external_torque_about_com_world_nm,
    )?;
    let mut uprights = Vec::new();
    let mut loads = Vec::new();
    let mut sources = Vec::new();
    let parent = configuration.parent().unwrap_or(Path::new("."));
    for input in &config.uprights {
        let mode = match input.data.as_str() {
            "auto" => asc_vehicle::data::Mode::Auto,
            "packed" => asc_vehicle::data::Mode::Packed,
            "unpacked" => asc_vehicle::data::Mode::Unpacked,
            _ => return Err("Choose auto, packed or unpacked data".into()),
        };
        let source = asc_vehicle::suspension::load(&parent.join(&input.car_directory), mode)?;
        let selected = source
            .axles
            .get(&input.axle)
            .ok_or("Choose an explicit front or rear axle")?;
        let geometry = match selected.kind.as_str() {
            "DWB" => TravelMechanism::Dwb(asc_session::dwb_geometry(selected, input.travel_axis)?),
            "STRUT" => {
                TravelMechanism::Strut(asc_session::strut_geometry(selected, input.travel_axis)?)
            }
            _ => return Err("Unsupported suspension geometry type".into()),
        };
        let mut upright = MountedUpright::new(
            geometry,
            MassProperties::new(input.mass_kg, input.inertia_rows_kg_m2)?,
            input.center_of_mass_local_m,
            input.rack_m,
            LinkPose::new(
                input.frame_position_in_chassis_m,
                input.frame_orientation_xyzw,
            )?,
            TravelState {
                travel_m: input.initial_travel_m,
                rate_m_s: input.initial_rate_m_s,
            },
        )?;
        for spring in &input.springs {
            upright.add_spring(spring.build()?)?;
        }
        uprights.push(upright);
        loads.push(Wrench::new(
            input.external_force_world_n,
            input.external_torque_about_com_world_nm,
        )?);
        sources.push(
            serde_json::json!({"car":source.car,"axle":input.axle,"type":selected.kind,
            "data_source":source.data_source,"source_diagnostics":source.diagnostics}),
        );
    }
    let mut system = ChassisSystem::new(body, uprights, config.gravity_world_m_s2)?;
    let mut records = vec![
        serde_json::json!({"step":0,"time_s":0.,"sample":system.sample(chassis_load,&loads)?}),
    ];
    for step in 1..=config.steps {
        let sample = system.step(config.step_s, chassis_load, &loads)?;
        if step % config.record_every == 0 || step == config.steps {
            records.push(serde_json::json!({"step":step,"time_s":f64::from(step)*config.step_s,"sample":sample}));
        }
    }
    Ok(serde_json::json!({
        "scope":"native_coupled_chassis_upright_dynamics","explicit_configuration":config,"source_geometry":sources,"records":records,
        "moving_chassis":true,"fixed_racks":true,"massless_links":true,
        "automatic_source_frames_mass_or_force_parameters":false,"original_vehicle_dynamics_validated":false,
        "simulation_ready":false,"renderer_animation":false,
        "limitations":["Explicit caller-provided chassis mounts and local frames; no automatic side/setup mapping",
            "One chassis and 1..4 massive uprights; no wheel-spin DOFs or tyre/contact/FFB forces",
            "Synthetic sample parameters; no original-game mass/inertia/preload calibration",
            "Explicit Lie-midpoint integration on a bounded local branch; no universal stability guarantee"]
    }))
}
