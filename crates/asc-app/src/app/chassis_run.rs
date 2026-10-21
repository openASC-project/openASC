//! Explicit coupled chassis workbench, with caller-supplied frame and mass mapping.
use super::dynamics_config::Spring;
use asc_physics::{
    Body, ChassisKinematics, ChassisLoads, ChassisSample, ChassisSystem, LinkPose, MassProperties,
    MountedUpright, NormalContact, NormalContactSample, SupportPlane, TravelMechanism, TravelState,
    Wheel, WheelLoad, WheelState, Wrench,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    wheel: Option<WheelConfiguration>,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct WheelConfiguration {
    center_in_upright_m: [f64; 3],
    axle_in_upright: [f64; 3],
    mass_kg: f64,
    axial_inertia_kg_m2: f64,
    transverse_inertia_kg_m2: f64,
    initial_angle_rad: f64,
    initial_rate_rad_s: f64,
    external_force_world_n: [f64; 3],
    external_torque_about_com_world_nm: [f64; 3],
    drive_torque_nm: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    normal_contact: Option<NormalConfiguration>,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct NormalConfiguration {
    radius_m: f64,
    stiffness_n_m: f64,
    damping_n_s_m: f64,
    max_penetration_m: f64,
}
impl NormalConfiguration {
    fn build(&self) -> asc_physics::Result<NormalContact> {
        NormalContact::new(
            self.radius_m,
            self.stiffness_n_m,
            self.damping_n_s_m,
            self.max_penetration_m,
        )
    }
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct PlaneConfiguration {
    point_world_m: [f64; 3],
    normal_world: [f64; 3],
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
    #[serde(skip_serializing_if = "Option::is_none")]
    support_plane: Option<PlaneConfiguration>,
}
struct Environment {
    base: ChassisLoads,
    plane: Option<SupportPlane>,
    models: Vec<Option<NormalContact>>,
}
impl Environment {
    fn contacts(
        &self,
        k: &ChassisKinematics,
    ) -> asc_physics::Result<Vec<Option<NormalContactSample>>> {
        self.models
            .iter()
            .zip(&k.uprights)
            .enumerate()
            .map(|(index, (model, u))| {
                if let Some(model) = model {
                    let plane = self.plane.ok_or_else(|| {
                        asc_physics::Error::new(
                            "contact_plane",
                            "A normal-contact model requires support_plane",
                        )
                    })?;
                    let wheel = u.wheel.as_ref().ok_or_else(|| {
                        asc_physics::Error::new(
                            "contact_wheel",
                            "Normal contact requires an attached wheel",
                        )
                    })?;
                    Ok(Some(model.sample(wheel, plane).map_err(|error| {
                        asc_physics::Error::new(
                            error.code,
                            &format!("Upright {index}: {}", error.message),
                        )
                    })?))
                } else {
                    Ok(None)
                }
            })
            .collect()
    }
    fn loads(&self, k: &ChassisKinematics) -> asc_physics::Result<ChassisLoads> {
        let mut loads = self.base.clone();
        for (load, contact) in loads.wheels.iter_mut().zip(self.contacts(k)?) {
            if let Some(contact) = contact {
                load.add_external(contact.wrench()?)?;
            }
        }
        Ok(loads)
    }
    fn record(
        &self,
        system: &ChassisSystem,
        step: u32,
        time_s: f64,
        sample: ChassisSample,
    ) -> asc_physics::Result<serde_json::Value> {
        let energy = sample.kinetic_energy_j + sample.potential_energy_j;
        let dissipation = sample.dissipated_power_w;
        let mut result = serde_json::json!({"step":step,"time_s":time_s,"sample":sample});
        if self.plane.is_some() {
            let contacts = self.contacts(&system.kinematics()?)?;
            let potential = contacts
                .iter()
                .flatten()
                .map(|s| s.potential_energy_j)
                .sum::<f64>();
            let dissipated = contacts
                .iter()
                .flatten()
                .map(|s| s.dissipated_power_w)
                .sum::<f64>();
            result["normal_contacts"] = serde_json::json!(contacts);
            result["normal_contact_potential_energy_j"] = potential.into();
            result["mechanical_energy_with_normal_contact_j"] = (energy + potential).into();
            result["total_dissipated_power_w"] = (dissipation + dissipated).into();
        }
        Ok(result)
    }
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
    let mut wheel_loads = Vec::new();
    let mut wheel_count = 0;
    let mut sources = Vec::new();
    let mut models = Vec::new();
    let mut contact_count = 0;
    let plane = config
        .support_plane
        .as_ref()
        .map(|p| SupportPlane::new(p.point_world_m, p.normal_world))
        .transpose()?;
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
        let wheel_load = if let Some(w) = &input.wheel {
            upright.attach_wheel(Wheel::new(
                w.center_in_upright_m,
                w.axle_in_upright,
                w.mass_kg,
                w.axial_inertia_kg_m2,
                w.transverse_inertia_kg_m2,
                WheelState {
                    angle_rad: w.initial_angle_rad,
                    rate_rad_s: w.initial_rate_rad_s,
                },
            )?)?;
            wheel_count += 1;
            let model = w
                .normal_contact
                .as_ref()
                .map(NormalConfiguration::build)
                .transpose()?;
            if model.is_some() {
                contact_count += 1;
            }
            models.push(model);
            WheelLoad::new(
                Wrench::new(
                    w.external_force_world_n,
                    w.external_torque_about_com_world_nm,
                )?,
                w.drive_torque_nm,
            )?
        } else {
            models.push(None);
            WheelLoad::default()
        };
        wheel_loads.push(wheel_load);
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
    if plane.is_some() != (contact_count > 0) {
        return Err("Supply support_plane and at least one wheel normal_contact together".into());
    }
    let environment = Environment {
        base: ChassisLoads {
            chassis: chassis_load,
            uprights: loads,
            wheels: wheel_loads,
        },
        plane,
        models,
    };
    let mut system = if plane.is_some() {
        ChassisSystem::new_with_forces(body, uprights, config.gravity_world_m_s2, |_, k| {
            environment.loads(k)
        })?
    } else {
        ChassisSystem::new(body, uprights, config.gravity_world_m_s2)?
    };
    let first = if plane.is_some() {
        system.sample_with_forces(|_, k| environment.loads(k))?
    } else {
        system.sample_with_wheels(
            environment.base.chassis,
            &environment.base.uprights,
            &environment.base.wheels,
        )?
    };
    let mut records = vec![environment.record(&system, 0, 0., first)?];
    for step in 1..=config.steps {
        let sample = if plane.is_some() {
            system.step_with_forces(config.step_s, |_, k| environment.loads(k))?
        } else {
            system.step_with_wheels(
                config.step_s,
                environment.base.chassis,
                &environment.base.uprights,
                &environment.base.wheels,
            )?
        };
        if step % config.record_every == 0 || step == config.steps {
            records.push(environment.record(
                &system,
                step,
                f64::from(step) * config.step_s,
                sample,
            )?);
        }
    }
    Ok(serde_json::json!({
        "scope":"native_coupled_chassis_upright_dynamics","explicit_configuration":config,"source_geometry":sources,"records":records,
        "moving_chassis":true,"fixed_racks":true,"massless_links":true,
        "wheel_spin_dofs":wheel_count,"wheel_mass_is_additional_to_upright":true,
        "normal_contact_wheels":contact_count,"normal_contact_uses_track_geometry":false,
        "automatic_source_frames_mass_or_force_parameters":false,"original_vehicle_dynamics_validated":false,
        "simulation_ready":false,"renderer_animation":false,
        "limitations":["Explicit caller-provided chassis mounts and local frames; no automatic side/setup mapping",
            "Optional thin-circle support on a static infinite plane; no tangential tyre, track-contact or FFB forces",
            "Synthetic sample parameters; no original-game mass/inertia/preload calibration",
            "Explicit Lie-midpoint integration on a bounded local branch; no universal stability guarantee"]
    }))
}
