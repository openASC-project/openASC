//! Explicit coupled chassis workbench, with caller-supplied frame and mass mapping.
use super::chassis_controls::{Control, Timeline};
use super::dynamics_config::Spring;
use asc_physics::{
    Body, ChassisKinematics, ChassisLoads, ChassisSample, ChassisSystem, LinkPose, MassProperties,
    MountedUpright, NormalContact, NormalContactSample, SupportPlane, TangentialContact,
    TangentialContactSample, TravelMechanism, TravelState, Wheel, WheelBrake, WheelBrakeSample,
    WheelLoad, WheelState, Wrench,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    tangential_contact: Option<TangentialConfiguration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    brake: Option<BrakeConfiguration>,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct BrakeConfiguration {
    maximum_torque_nm: f64,
    transition_rate_rad_s: f64,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct TangentialConfiguration {
    longitudinal_viscosity_n_s_m: f64,
    lateral_viscosity_n_s_m: f64,
    longitudinal_friction: f64,
    lateral_friction: f64,
}
impl TangentialConfiguration {
    fn build(&self) -> asc_physics::Result<TangentialContact> {
        TangentialContact::new(
            self.longitudinal_viscosity_n_s_m,
            self.lateral_viscosity_n_s_m,
            self.longitudinal_friction,
            self.lateral_friction,
        )
    }
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
    #[serde(skip_serializing_if = "Option::is_none")]
    friction_scale: Option<f64>,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct TrackConfiguration {
    models_ini: String,
    surface_files: Vec<String>,
    min_normal_y: f64,
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SpawnConfiguration {
    name: String,
    chassis_com_offset_in_spawn_m: [f64; 3],
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
    #[serde(skip_serializing_if = "Option::is_none")]
    support_track: Option<TrackConfiguration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    initial_spawn: Option<SpawnConfiguration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    controls: Option<Vec<Control>>,
}
#[derive(Default)]
struct Contact {
    normal: Option<NormalContactSample>,
    face: Option<asc_session::TrackFace>,
    tangential: Option<TangentialContactSample>,
}
struct Environment {
    base: ChassisLoads,
    plane: Option<SupportPlane>,
    track: Option<(asc_ground::Ground, f64)>,
    models: Vec<Option<NormalContact>>,
    tangential: Vec<Option<TangentialContact>>,
    plane_friction: Option<f64>,
    brakes: Vec<Option<WheelBrake>>,
    controls: Option<Timeline>,
}
impl Environment {
    fn needs_provider(&self) -> bool {
        self.has_contact() || self.controls.is_some() || self.brakes.iter().any(Option::is_some)
    }
    fn actuation(
        &self,
        time_s: f64,
        k: &ChassisKinematics,
    ) -> asc_physics::Result<(Control, Vec<Option<WheelBrakeSample>>, f64)> {
        let control = if let Some(timeline) = &self.controls {
            timeline.sample(time_s)?
        } else {
            Control {
                time_s,
                drive_fraction: 1.,
                brake_fraction: 0.,
            }
        };
        let mut motor_power = 0.;
        let mut brakes = Vec::new();
        for (i, u) in k.uprights.iter().enumerate() {
            if let Some(wheel) = &u.wheel {
                motor_power += self.base.wheels[i].internal_torque_nm()
                    * control.drive_fraction
                    * wheel.state.rate_rad_s;
                brakes.push(
                    self.brakes[i]
                        .as_ref()
                        .map(|b| b.sample(wheel.state.rate_rad_s, control.brake_fraction))
                        .transpose()?,
                );
            } else {
                brakes.push(None);
            }
        }
        Ok((control, brakes, motor_power))
    }
    fn has_contact(&self) -> bool {
        self.plane.is_some() || self.track.is_some()
    }
    fn contacts(&self, k: &ChassisKinematics) -> asc_physics::Result<Vec<Contact>> {
        self.models
            .iter()
            .zip(&k.uprights)
            .enumerate()
            .map(|(index, (model, u))| {
                if let Some(model) = model {
                    let wheel = u.wheel.as_ref().ok_or_else(|| {
                        asc_physics::Error::new(
                            "contact_wheel",
                            "Normal contact requires an attached wheel",
                        )
                    })?;
                    let result = if let Some((ground, min_y)) = &self.track {
                        asc_session::face_contact_with_friction(ground, wheel, model, *min_y, self.tangential[index].as_ref())
                            .map(|c| c.map(|c| Contact { normal:Some(c.sample), face:Some(c.face), tangential:c.tangential }))
                    } else if let Some(plane) = self.plane {
                        if let Some(friction) = &self.tangential[index] {
                            model.sample_with_friction(wheel,plane,friction,self.plane_friction.ok_or_else(||asc_physics::Error::new("contact_friction","Plane tangential contact requires explicit friction_scale"))?)
                                .map(|c|Some(Contact {normal:Some(c.normal),face:None,tangential:Some(c.tangential)}))
                        } else {
                            model.sample(wheel, plane).map(|s| Some(Contact { normal:Some(s), ..Contact::default() }))
                        }
                    } else {
                        Err(asc_physics::Error::new(
                            "contact_support",
                            "A normal-contact model requires a support environment",
                        ))
                    };
                    Ok(result
                        .map_err(|error| {
                            asc_physics::Error::new(
                                error.code,
                                &format!("Upright {index}: {}", error.message),
                            )
                        })?
                        .unwrap_or_default())
                } else {
                    Ok(Contact::default())
                }
            })
            .collect()
    }
    fn loads(&self, time_s: f64, k: &ChassisKinematics) -> asc_physics::Result<ChassisLoads> {
        let mut loads = self.base.clone();
        let (control, brakes, _) = self.actuation(time_s, k)?;
        for (load, brake) in loads.wheels.iter_mut().zip(brakes) {
            load.set_internal_torque(control.drive_fraction * load.internal_torque_nm())?;
            if let Some(brake) = brake {
                load.add_internal_torque(brake.torque_on_wheel_nm)?;
            }
        }
        for (load, contact) in loads.wheels.iter_mut().zip(self.contacts(k)?) {
            if let Some(contact) = contact.normal {
                load.add_external(contact.wrench()?)?;
            }
            if let Some(contact) = contact.tangential {
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
        let mut brake_dissipation = 0.;
        if self.controls.is_some() || self.brakes.iter().any(Option::is_some) {
            let (control, brakes, motor_power) = self.actuation(time_s, &system.kinematics()?)?;
            brake_dissipation = brakes
                .iter()
                .flatten()
                .map(|b| b.dissipated_power_w)
                .sum::<f64>();
            result["controls"] = serde_json::json!(control);
            result["wheel_brakes"] = serde_json::json!(brakes);
            result["motor_power_w"] = motor_power.into();
            result["brake_dissipated_power_w"] = brake_dissipation.into();
            result["total_dissipated_power_w"] = (dissipation + brake_dissipation).into();
        }
        if self.has_contact() {
            let evaluated = self.contacts(&system.kinematics()?)?;
            let contacts: Vec<_> = evaluated.iter().map(|c| c.normal.as_ref()).collect();
            let faces: Vec<_> = evaluated.iter().map(|c| c.face.as_ref()).collect();
            let tangential: Vec<_> = evaluated.iter().map(|c| c.tangential.as_ref()).collect();
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
            let tangential_dissipated = tangential
                .iter()
                .flatten()
                .map(|c| c.dissipated_power_w)
                .sum::<f64>();
            result["normal_contacts"] = serde_json::json!(contacts);
            if self.track.is_some() {
                result["track_contact_faces"] = serde_json::json!(faces);
            }
            if self.tangential.iter().any(Option::is_some) {
                result["tangential_contacts"] = serde_json::json!(tangential);
                result["tangential_dissipated_power_w"] = tangential_dissipated.into();
            }
            result["normal_contact_potential_energy_j"] = potential.into();
            result["mechanical_energy_with_normal_contact_j"] = (energy + potential).into();
            result["total_dissipated_power_w"] =
                (dissipation + dissipated + tangential_dissipated + brake_dissipation).into();
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
    let parent = configuration.parent().unwrap_or(Path::new("."));
    let properties = MassProperties::new(c.mass_kg, c.inertia_rows_kg_m2)?;
    let mut spawn_diagnostics = Vec::new();
    let (position, orientation) = if let Some(s) = &config.initial_spawn {
        let track = config
            .support_track
            .as_ref()
            .ok_or("initial_spawn requires support_track")?;
        if c.position_world_m != [0.; 3]
            || c.orientation_xyzw != [0., 0., 0., 1.]
            || s.chassis_com_offset_in_spawn_m
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 100.)
        {
            return Err("Spawn placement requires zero chassis position, identity orientation and an explicit finite offset within +/-100 m".into());
        }
        let report = asc_scene::spawns::load(&parent.join(&track.models_ini))?;
        let spawn = report
            .spawns
            .iter()
            .find(|v| v.name == s.name)
            .ok_or("Selected initial_spawn is missing, ambiguous or unsupported")?;
        let axes = [spawn.left, spawn.up, spawn.forward];
        let position = std::array::from_fn(|i| {
            spawn.position[i]
                + (0..3)
                    .map(|j| axes[j][i] * s.chassis_com_offset_in_spawn_m[j])
                    .sum::<f64>()
        });
        let state =
            asc_session::body_with_spawn_basis(properties.clone(), position, spawn)?.state();
        spawn_diagnostics = report.diagnostics;
        (position, state.orientation_xyzw)
    } else {
        (c.position_world_m, c.orientation_xyzw)
    };
    let body = Body::new(
        properties,
        position,
        orientation,
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
    let mut tangential = Vec::new();
    let mut brakes = Vec::new();
    let mut contact_count = 0;
    let plane = config
        .support_plane
        .as_ref()
        .map(|p| SupportPlane::new(p.point_world_m, p.normal_world))
        .transpose()?;
    if plane.is_some() && config.support_track.is_some() {
        return Err("Choose support_plane or support_track, never both".into());
    }
    let track = config
        .support_track
        .as_ref()
        .map(|t| -> Result<_, Box<dyn Error>> {
            if !(1..=16).contains(&t.surface_files.len())
                || !t.min_normal_y.is_finite()
                || !(1e-4..=1.).contains(&t.min_normal_y)
            {
                return Err(
                    "Track support requires 1..16 surface files and min_normal_y within 1e-4..1"
                        .into(),
                );
            }
            let paths: Vec<_> = t.surface_files.iter().map(|p| parent.join(p)).collect();
            let refs: Vec<_> = paths.iter().map(|p| p.as_path()).collect();
            let ground = asc_ground::load(&parent.join(&t.models_ini), &refs)?;
            Ok((ground, t.min_normal_y))
        })
        .transpose()?;
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
            brakes.push(
                w.brake
                    .as_ref()
                    .map(|b| WheelBrake::new(b.maximum_torque_nm, b.transition_rate_rad_s))
                    .transpose()?,
            );
            let model = w
                .normal_contact
                .as_ref()
                .map(NormalConfiguration::build)
                .transpose()?;
            if model.is_some() {
                contact_count += 1;
            }
            if w.tangential_contact.is_some() && model.is_none() {
                return Err("Tangential contact requires a wheel normal_contact".into());
            }
            tangential.push(
                w.tangential_contact
                    .as_ref()
                    .map(TangentialConfiguration::build)
                    .transpose()?,
            );
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
            tangential.push(None);
            brakes.push(None);
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
    if (plane.is_some() || track.is_some()) != (contact_count > 0) {
        return Err(
            "Supply support_plane or support_track and at least one wheel normal_contact together"
                .into(),
        );
    }
    let tangential_count = tangential.iter().flatten().count();
    let plane_friction = config.support_plane.as_ref().and_then(|p| p.friction_scale);
    if plane_friction.is_some_and(|v| !v.is_finite() || !(0. ..=10.).contains(&v))
        || (plane.is_some() && tangential_count > 0 && plane_friction.is_none())
    {
        return Err(
            "Plane tangential contact requires explicit friction_scale within 0..10".into(),
        );
    }
    let environment = Environment {
        base: ChassisLoads {
            chassis: chassis_load,
            uprights: loads,
            wheels: wheel_loads,
        },
        plane,
        track,
        models,
        tangential,
        plane_friction,
        brakes,
        controls: config
            .controls
            .as_ref()
            .map(|c| Timeline::new(c))
            .transpose()?,
    };
    if environment.controls.is_some() && wheel_count == 0 {
        return Err("Wheel controls require at least one wheel".into());
    }
    let mut system = if environment.needs_provider() {
        ChassisSystem::new_with_forces(body, uprights, config.gravity_world_m_s2, |offset, k| {
            environment.loads(offset, k)
        })?
    } else {
        ChassisSystem::new(body, uprights, config.gravity_world_m_s2)?
    };
    let first = if environment.needs_provider() {
        system.sample_with_forces(|offset, k| environment.loads(offset, k))?
    } else {
        system.sample_with_wheels(
            environment.base.chassis,
            &environment.base.uprights,
            &environment.base.wheels,
        )?
    };
    let mut records = vec![environment.record(&system, 0, 0., first)?];
    for step in 1..=config.steps {
        let sample = if environment.needs_provider() {
            let start = f64::from(step - 1) * config.step_s;
            system.step_with_forces(config.step_s, |offset, k| {
                environment.loads(start + offset, k)
            })?
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
        "normal_contact_wheels":contact_count,"normal_contact_uses_track_geometry":environment.track.is_some(),
        "tangential_contact_wheels":tangential_count,"tangential_contact_model":"optional_memoryless_viscous_slip_friction_ellipse",
        "wheel_brake_models":environment.brakes.iter().flatten().count(),"scheduled_wheel_controls":environment.controls.is_some(),
        "wheel_brake_model":"optional_passive_regularized_internal_torque","static_wheel_lock":false,
        "track_geometry":environment.track.as_ref().map(|(g,_)|g.summary()),
        "track_diagnostics":environment.track.as_ref().map(|(g,_)|g.diagnostics()),
        "initial_spawn_diagnostics":spawn_diagnostics,
        "edge_vertex_response":false,"continuous_collision_detection":false,
        "automatic_source_frames_mass_or_force_parameters":false,"original_vehicle_dynamics_validated":false,
        "simulation_ready":false,"renderer_animation":false,
        "limitations":["Explicit caller-provided chassis mounts and local frames; no automatic side/setup mapping",
            "Optional thin-circle support and memoryless tangential slip friction; no edge/vertex collision, CCD, static stiction, transient tyre patch or FFB",
            "Synthetic sample parameters; no original-game mass/inertia/preload calibration",
            "Explicit Lie-midpoint integration on a bounded local branch; no universal stability guarantee"]
    }))
}
