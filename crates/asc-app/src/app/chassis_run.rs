//! Explicit coupled chassis workbench, with caller-supplied frame and mass mapping.
use super::chassis_controls::{Control, Timeline};
use super::chassis_steering::{Steering, SteeringKey};
use super::dynamics_config::Spring;
use asc_physics::{
    Body, ChassisKinematics, ChassisLoads, ChassisSample, ChassisSystem, LinkPose, MassProperties,
    MountedUpright, NormalContact, NormalContactSample, RackMotion, SupportPlane,
    TangentialContact, TangentialContactSample, TravelMechanism, TravelState, Wheel, WheelBrake,
    WheelBrakeSample, WheelLoad, WheelState, Wrench,
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
    #[serde(default)]
    source_geometry_reflect_x: bool,
    travel_axis: [f64; 3],
    rack_m: [f64; 3],
    #[serde(skip_serializing_if = "Option::is_none")]
    steering: Option<SteeringConfiguration>,
    #[serde(skip_serializing_if = "Option::is_none")]
    frame_position_in_chassis_m: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    source_wheel_geometry: Option<SourceWheelGeometry>,
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
struct SteeringConfiguration {
    axis_in_source: [f64; 3],
    keys: Vec<SteeringKey>,
}
enum RackPrescription {
    Fixed(RackMotion),
    Steered(Steering),
}
impl RackPrescription {
    fn sample(&self, time: f64) -> asc_physics::Result<RackMotion> {
        match self {
            Self::Fixed(r) => Ok(*r),
            Self::Steered(s) => s.sample(time),
        }
    }
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
    #[serde(skip_serializing_if = "Option::is_none")]
    radius_m: Option<f64>,
    stiffness_n_m: f64,
    damping_n_s_m: f64,
    max_penetration_m: f64,
}
impl NormalConfiguration {
    fn build(&self, source_radius: Option<f64>) -> Result<NormalContact, Box<dyn Error>> {
        let radius = match (self.radius_m, source_radius) {
            (Some(r), None) | (None, Some(r)) => r,
            _ => {
                return Err(
                    "Choose an explicit normal radius or source_wheel_geometry, never both".into(),
                );
            }
        };
        Ok(NormalContact::new(
            radius,
            self.stiffness_n_m,
            self.damping_n_s_m,
            self.max_penetration_m,
        )?)
    }
}
#[derive(Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
enum Corner {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}
impl Corner {
    fn name(self) -> &'static str {
        match self {
            Self::FrontLeft => "front_left",
            Self::FrontRight => "front_right",
            Self::RearLeft => "rear_left",
            Self::RearRight => "rear_right",
        }
    }
    fn axle(self) -> &'static str {
        match self {
            Self::FrontLeft | Self::FrontRight => "front",
            Self::RearLeft | Self::RearRight => "rear",
        }
    }
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct SourceWheelGeometry {
    corner: Corner,
    compound: Option<u32>,
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
    racks: Vec<RackPrescription>,
    steering_count: usize,
}
impl Environment {
    fn racks(&self, time: f64) -> asc_physics::Result<Vec<RackMotion>> {
        self.racks.iter().map(|r| r.sample(time)).collect()
    }
    fn needs_provider(&self) -> bool {
        self.has_contact()
            || self.controls.is_some()
            || self.brakes.iter().any(Option::is_some)
            || self.steering_count > 0
    }
    fn actuation(
        &self,
        time_s: f64,
        k: &ChassisKinematics,
    ) -> asc_physics::Result<(Control, Vec<Option<WheelBrakeSample>>, f64)> {
        self.actuation_override(time_s, k, None)
    }
    fn actuation_override(
        &self,
        time_s: f64,
        k: &ChassisKinematics,
        command: Option<Control>,
    ) -> asc_physics::Result<(Control, Vec<Option<WheelBrakeSample>>, f64)> {
        let control = if let Some(command) = command {
            command
        } else if let Some(timeline) = &self.controls {
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
        self.loads_override(time_s, k, None)
    }
    fn loads_override(
        &self,
        time_s: f64,
        k: &ChassisKinematics,
        command: Option<Control>,
    ) -> asc_physics::Result<ChassisLoads> {
        let mut loads = self.base.clone();
        let (control, brakes, _) = self.actuation_override(time_s, k, command)?;
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
        let rack_power = sample.rack_actuator_power_w;
        let mut result = serde_json::json!({"step":step,"time_s":time_s,"sample":sample});
        let mut brake_dissipation = 0.;
        if self.controls.is_some()
            || self.brakes.iter().any(Option::is_some)
            || self.steering_count > 0
        {
            let (control, brakes, motor_power) = self.actuation(time_s, &system.kinematics()?)?;
            brake_dissipation = brakes
                .iter()
                .flatten()
                .map(|b| b.dissipated_power_w)
                .sum::<f64>();
            result["controls"] = serde_json::json!(control);
            result["wheel_brakes"] = serde_json::json!(brakes);
            result["motor_power_w"] = motor_power.into();
            result["actuator_input_power_w"] = (motor_power + rack_power).into();
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
pub(super) struct Workbench {
    config: Configuration,
    environment: Environment,
    pub system: ChassisSystem,
    initial: ChassisSystem,
    sources: Vec<serde_json::Value>,
    spawn_diagnostics: Vec<asc_scene::spawns::Diagnostic>,
    wheel_count: usize,
    contact_count: usize,
    tangential_count: usize,
    live_ticks: u64,
    parent: std::path::PathBuf,
}
impl Workbench {
    pub fn step_s(&self) -> f64 {
        self.config.step_s
    }
    pub fn time_s(&self) -> f64 {
        self.live_ticks as f64 * self.config.step_s
    }
    pub fn reset(&mut self) {
        self.system = self.initial.clone();
        self.live_ticks = 0;
    }
    pub fn diagnostics(&self) -> serde_json::Value {
        serde_json::json!({"source_geometry":self.sources,"spawn_diagnostics":self.spawn_diagnostics,
            "track_geometry":self.environment.track.as_ref().map(|(g,_)|g.summary()),
            "track_diagnostics":self.environment.track.as_ref().map(|(g,_)|g.diagnostics())})
    }
    pub fn visual_paths(
        &self,
    ) -> Result<
        (
            std::path::PathBuf,
            std::path::PathBuf,
            asc_vehicle::data::Mode,
        ),
        Box<dyn Error>,
    > {
        if self.wheel_count != 4
            || self.contact_count != 4
            || self.tangential_count != 4
            || self.environment.brakes.iter().flatten().count() != 4
            || self.environment.steering_count == 0
        {
            return Err("Native drive requires four wheels with normal/tangential contact and brakes, plus explicit steering axes".into());
        }
        let track = self
            .config
            .support_track
            .as_ref()
            .ok_or("Native drive requires support_track")?;
        let first = &self.config.uprights[0];
        let car = self.parent.join(&first.car_directory).canonicalize()?;
        for u in &self.config.uprights {
            if self.parent.join(&u.car_directory).canonicalize()? != car || u.data != first.data {
                return Err(
                    "Native visual driving requires one car/data source across all four corners"
                        .into(),
                );
            }
        }
        let mode = match first.data.as_str() {
            "auto" => asc_vehicle::data::Mode::Auto,
            "packed" => asc_vehicle::data::Mode::Packed,
            _ => asc_vehicle::data::Mode::Unpacked,
        };
        Ok((car, self.parent.join(&track.models_ini), mode))
    }
    pub fn live_step(
        &mut self,
        command: super::drive_input::Command,
        actuator: super::drive_input::SteeringActuator,
    ) -> asc_physics::Result<ChassisSample> {
        use glam::DVec3;
        command.validate()?;
        actuator.validate()?;
        let dt = self.config.step_s;
        let time = self.time_s();
        let start = self.system.rack_motions();
        let mut accelerations = Vec::new();
        for (u, r) in self.config.uprights.iter().zip(&start) {
            let a = if let Some(s) = &u.steering {
                let axis = DVec3::from_array(s.axis_in_source).normalize();
                let p = axis.dot(DVec3::from_array(r.position_m) - DVec3::from_array(u.rack_m));
                let v = axis.dot(DVec3::from_array(r.velocity_m_s));
                axis * actuator.acceleration(p, v, command.steering, dt)?
            } else {
                DVec3::ZERO
            };
            accelerations.push(a);
        }
        let sample = self.system.step_with_racks_and_forces(
            dt,
            |offset| {
                start
                    .iter()
                    .zip(&accelerations)
                    .map(|(r, a)| {
                        RackMotion::new(
                            (DVec3::from_array(r.position_m)
                                + DVec3::from_array(r.velocity_m_s) * offset
                                + 0.5 * (*a) * offset * offset)
                                .to_array(),
                            (DVec3::from_array(r.velocity_m_s) + (*a) * offset).to_array(),
                            a.to_array(),
                        )
                    })
                    .collect()
            },
            |offset, k| {
                self.environment.loads_override(
                    time + offset,
                    k,
                    Some(Control {
                        time_s: time + offset,
                        drive_fraction: command.drive,
                        brake_fraction: command.brake,
                    }),
                )
            },
        )?;
        self.live_ticks += 1;
        Ok(sample)
    }
}
pub(super) fn load(configuration: &Path) -> Result<Workbench, Box<dyn Error>> {
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
    let mut racks = Vec::new();
    let mut steering_count = 0;
    let mut contact_count = 0;
    let mut source_corners = std::collections::BTreeSet::new();
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
            "DWB" => TravelMechanism::Dwb(asc_session::dwb_geometry_with_reflection(
                selected,
                input.travel_axis,
                input.source_geometry_reflect_x,
            )?),
            "STRUT" => TravelMechanism::Strut(asc_session::strut_geometry_with_reflection(
                selected,
                input.travel_axis,
                input.source_geometry_reflect_x,
            )?),
            _ => return Err("Unsupported suspension geometry type".into()),
        };
        let rack = if let Some(s) = &input.steering {
            steering_count += 1;
            RackPrescription::Steered(Steering::new(input.rack_m, s.axis_in_source, &s.keys)?)
        } else {
            RackPrescription::Fixed(RackMotion::fixed(input.rack_m)?)
        };
        let initial_rack = rack.sample(0.)?;
        let mut source_radius = None;
        let mut resolved_geometry = None;
        let frame_position = match (
            &input.source_wheel_geometry,
            input.frame_position_in_chassis_m,
        ) {
            (None, Some(position)) => position,
            (Some(reference), None) => {
                if reference.corner.axle() != input.axle {
                    return Err(
                        "source_wheel_geometry corner must match the explicit upright axle".into(),
                    );
                }
                let root = parent.join(&input.car_directory).canonicalize()?;
                if !source_corners.insert((root.clone(), reference.corner.name())) {
                    return Err(
                        "source_wheel_geometry must not select the same car corner twice".into(),
                    );
                }
                let wheel = input
                    .wheel
                    .as_ref()
                    .ok_or("source_wheel_geometry requires a wheel")?;
                let vehicle = asc_vehicle::load(&root, mode, reference.compound)?;
                let center = vehicle.wheel_offsets_m[reference.corner.name()];
                source_radius = Some(vehicle.axles[&input.axle].tyre_radius_m);
                let design = match &geometry {
                    TravelMechanism::Dwb(m) => m
                        .solve(input.initial_travel_m, initial_rack.position_m, None)?
                        .pose()?,
                    TravelMechanism::Strut(m) => m
                        .solve(input.initial_travel_m, initial_rack.position_m, None)?
                        .pose()?,
                };
                let lever = LinkPose::new([0.; 3], input.frame_orientation_xyzw)?
                    .transform(design.transform(wheel.center_in_upright_m)?)?;
                let position = std::array::from_fn(|i| center[i] - lever[i]);
                resolved_geometry = Some(
                    serde_json::json!({"corner":reference.corner,"compound":vehicle.compound,
                    "wheel_center_in_chassis_m":center,"normal_radius_m":source_radius,"frame_position_in_chassis_m":position,
                    "field_sources":vehicle.field_sources,"source_diagnostics":vehicle.diagnostics,
                    "policy":"initial physical wheel centre registered to typed vehicle footprint; explicit source-frame orientation retained",
                    "original_setup_or_frame_mapping_validated":false}),
                );
                position
            }
            _ => {
                return Err(
                    "Supply exactly one of frame_position_in_chassis_m and source_wheel_geometry"
                        .into(),
                );
            }
        };
        let mut upright = MountedUpright::new(
            geometry,
            MassProperties::new(input.mass_kg, input.inertia_rows_kg_m2)?,
            input.center_of_mass_local_m,
            initial_rack.position_m,
            LinkPose::new(frame_position, input.frame_orientation_xyzw)?,
            TravelState {
                travel_m: input.initial_travel_m,
                rate_m_s: input.initial_rate_m_s,
            },
        )?;
        upright.set_rack_motion(initial_rack)?;
        racks.push(rack);
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
                .map(|m| m.build(source_radius))
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
            "data_source":source.data_source,"source_diagnostics":source.diagnostics,"resolved_wheel_geometry":resolved_geometry,
            "source_geometry_reflect_x":input.source_geometry_reflect_x,
            "source_frame_policy":"reflect raw hardpoint X only; all explicit vectors, tensors, frame rotations and loads already use the resulting coordinates"}),
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
        racks,
        steering_count,
        controls: config
            .controls
            .as_ref()
            .map(|c| Timeline::new(c))
            .transpose()?,
    };
    if environment.controls.is_some() && wheel_count == 0 {
        return Err("Wheel controls require at least one wheel".into());
    }
    let system = if environment.needs_provider() {
        ChassisSystem::new_with_forces(body, uprights, config.gravity_world_m_s2, |offset, k| {
            environment.loads(offset, k)
        })?
    } else {
        ChassisSystem::new(body, uprights, config.gravity_world_m_s2)?
    };
    Ok(Workbench {
        initial: system.clone(),
        system,
        config,
        environment,
        sources,
        spawn_diagnostics,
        wheel_count,
        contact_count,
        tangential_count,
        live_ticks: 0,
        parent: parent.to_path_buf(),
    })
}
pub fn run(configuration: &Path) -> Result<serde_json::Value, Box<dyn Error>> {
    let Workbench {
        config,
        environment,
        mut system,
        sources,
        spawn_diagnostics,
        wheel_count,
        contact_count,
        tangential_count,
        ..
    } = load(configuration)?;
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
        let sample = if environment.steering_count > 0 {
            let start = f64::from(step - 1) * config.step_s;
            system.step_with_racks_and_forces(
                config.step_s,
                |offset| environment.racks(start + offset),
                |offset, k| environment.loads(start + offset, k),
            )?
        } else if environment.needs_provider() {
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
        "moving_chassis":true,"fixed_racks":environment.steering_count==0,"prescribed_racks":environment.steering_count,"massless_links":true,
        "wheel_spin_dofs":wheel_count,"wheel_mass_is_additional_to_upright":true,
        "source_wheel_geometry_registrations":config.uprights.iter().filter(|u|u.source_wheel_geometry.is_some()).count(),
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
        "limitations":["Explicit local frame orientations; optional initial wheel-centre registration, no automatic side/setup mapping",
            "Optional thin-circle support and memoryless tangential slip friction; no edge/vertex collision, CCD, static stiction, transient tyre patch or FFB",
            "Synthetic sample parameters; no original-game mass/inertia/preload calibration",
            "Explicit Lie-midpoint integration on a bounded local branch; no universal stability guarantee"]
    }))
}

#[cfg(test)]
mod live_tests {
    use super::*;
    use crate::app::drive_input::{Command, SteeringActuator};
    struct Directory(std::path::PathBuf);
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    fn fixture() -> (Directory, Workbench) {
        let root = asc_test_support::temp_path("live-dynamics");
        std::fs::create_dir_all(root.join("data")).unwrap();
        std::fs::write(
            root.join("data/suspensions.ini"),
            asc_test_support::suspension_motion_file(),
        )
        .unwrap();
        let mut c: serde_json::Value = serde_json::from_str(include_str!(
            "../../../../examples/chassis-steering-track.json"
        ))
        .unwrap();
        c.as_object_mut().unwrap().remove("support_track");
        c.as_object_mut().unwrap().remove("initial_spawn");
        c["support_plane"] =
            serde_json::json!({"point_world_m":[0,0,0],"normal_world":[0,1,0],"friction_scale":1});
        c["chassis"]["position_world_m"] = serde_json::json!([0, 0.29, 0]);
        for u in c["uprights"].as_array_mut().unwrap() {
            u["car_directory"] = ".".into();
        }
        let path = root.join("dynamics.json");
        std::fs::write(&path, c.to_string()).unwrap();
        let bench = load(&path).unwrap();
        (Directory(root), bench)
    }
    fn actuator() -> SteeringActuator {
        SteeringActuator {
            displacement_limit_m: 0.01,
            frequency_rad_s: 8.,
            acceleration_limit_m_s2: 0.2,
            rate_limit_m_s: 0.08,
        }
    }
    fn source_fixture() -> (Directory, serde_json::Value) {
        let (dir, _) = fixture();
        for (name, text) in asc_test_support::vehicle_files() {
            std::fs::write(dir.0.join("data").join(name), text).unwrap();
        }
        let source = asc_test_support::suspension_motion_file()
            .replace("WHEELBASE=2.4", "WHEELBASE=3.2")
            .replace("CG_LOCATION=.5", "CG_LOCATION=.37");
        std::fs::write(dir.0.join("data/suspensions.ini"), source).unwrap();
        std::fs::write(
            dir.0.join("original.kn5"),
            [b"sc6969".as_slice(), &5u32.to_le_bytes()].concat(),
        )
        .unwrap();
        let mut c: serde_json::Value =
            serde_json::from_slice(&std::fs::read(dir.0.join("dynamics.json")).unwrap()).unwrap();
        c.as_object_mut().unwrap().remove("support_plane");
        c["gravity_world_m_s2"] = serde_json::json!([0, 0, 0]);
        for (i, u) in c["uprights"].as_array_mut().unwrap().iter_mut().enumerate() {
            u.as_object_mut()
                .unwrap()
                .remove("frame_position_in_chassis_m");
            u["source_wheel_geometry"] = serde_json::json!({"corner":(["front_left","front_right","rear_left","rear_right"][i]),"compound":1});
            u["wheel"].as_object_mut().unwrap().remove("normal_contact");
            u["wheel"]
                .as_object_mut()
                .unwrap()
                .remove("tangential_contact");
        }
        (dir, c)
    }
    fn load_config(dir: &Directory, c: &serde_json::Value) -> Result<Workbench, Box<dyn Error>> {
        let path = dir.0.join("registered.json");
        std::fs::write(&path, c.to_string())?;
        load(&path)
    }
    #[test]
    fn source_centres_register_nonzero_wheel_levers_and_initial_linkage_poses_in_rotated_chassis() {
        use glam::{DQuat, DVec3};
        let (dir, mut c) = source_fixture();
        let original = std::fs::read(dir.0.join("data/suspensions.ini")).unwrap();
        let rotation = DQuat::from_rotation_y(0.4) * DQuat::from_rotation_x(0.1);
        c["chassis"]["position_world_m"] = serde_json::json!([2., 3., 4.]);
        c["chassis"]["orientation_xyzw"] = serde_json::json!(rotation.to_array());
        for (i, u) in c["uprights"].as_array_mut().unwrap().iter_mut().enumerate() {
            u["initial_travel_m"] = if i < 2 { 0.02 } else { -0.015 }.into();
            u["wheel"]["center_in_upright_m"] = serde_json::json!([0.015, 0.02, 0.01]);
            if i < 2 {
                u["steering"]["keys"][0]["displacement_m"] = 0.002.into();
            }
        }
        let bench = load_config(&dir, &c).unwrap();
        let k = bench.system.kinematics().unwrap();
        for (i, u) in k.uprights.iter().enumerate() {
            let expected = DVec3::new(2., 3., 4.)
                + rotation
                    * DVec3::new(
                        if i % 2 == 0 { 0.75 } else { -0.75 },
                        -0.2,
                        if i < 2 { 3.2 * 0.63 } else { -3.2 * 0.37 },
                    );
            assert!(
                (DVec3::from_array(u.wheel.as_ref().unwrap().body.position_m) - expected).length()
                    < 1e-10
            );
            let g = &bench.sources[i]["resolved_wheel_geometry"];
            assert_eq!(g["compound"], 1);
            assert_eq!(g["normal_radius_m"], if i < 2 { 0.32 } else { 0.33 });
            assert!(g["field_sources"]["suspensions.ini"].is_object());
            assert_eq!(g["original_setup_or_frame_mapping_validated"], false);
        }
        assert_eq!(
            std::fs::read(dir.0.join("data/suspensions.ini")).unwrap(),
            original
        );
    }
    #[test]
    fn reflected_source_corner_preserves_coupled_dynamics_contact_and_axial_motor_reactions() {
        use glam::{DQuat, DVec3};
        let mirror = |v: [f64; 3]| [-v[0], v[1], v[2]];
        let axial = |v: [f64; 3]| [v[0], -v[1], -v[2]];
        let near = |a: [f64; 3], b: [f64; 3]| {
            assert!(
                (DVec3::from_array(a) - DVec3::from_array(b)).length() < 1e-7,
                "{a:?} != {b:?}"
            )
        };
        for index in [0, 2] {
            let (dir, mut c) = source_fixture();
            c["uprights"] = serde_json::json!([c["uprights"][index].clone()]);
            c["support_plane"] = serde_json::json!({"point_world_m":[0,0,0],"normal_world":[0,1,0],"friction_scale":1});
            c["gravity_world_m_s2"] = serde_json::json!([0, -9.81, 0]);
            c["chassis"]["position_world_m"] = serde_json::json!([0, 0.515, 0]);
            c["chassis"]["velocity_world_m_s"] = serde_json::json!([0.05, -0.01, 0.1]);
            c["chassis"]["angular_velocity_world_rad_s"] = serde_json::json!([0.01, 0.02, -0.03]);
            c["uprights"][0]["wheel"]["initial_rate_rad_s"] = 2.into();
            c["uprights"][0]["wheel"]["drive_torque_nm"] = 100.into();
            c["uprights"][0]["wheel"]["normal_contact"] = serde_json::json!({"stiffness_n_m":200000,"damping_n_s_m":1500,"max_penetration_m":0.08});
            c["uprights"][0]["wheel"]["tangential_contact"] = serde_json::json!({"longitudinal_viscosity_n_s_m":300,"lateral_viscosity_n_s_m":400,"longitudinal_friction":0.9,"lateral_friction":0.9});
            let mut left = load_config(&dir, &c).unwrap();
            let u = &mut c["uprights"][0];
            u["source_geometry_reflect_x"] = true.into();
            u["source_wheel_geometry"]["corner"] = if index == 0 {
                "front_right"
            } else {
                "rear_right"
            }
            .into();
            let center: [f64; 3] =
                serde_json::from_value(u["center_of_mass_local_m"].clone()).unwrap();
            u["center_of_mass_local_m"] = serde_json::json!(mirror(center));
            for i in 0..3 {
                for j in 0..3 {
                    let sign = if (i == 0) ^ (j == 0) { -1. } else { 1. };
                    u["inertia_rows_kg_m2"][i][j] =
                        (sign * u["inertia_rows_kg_m2"][i][j].as_f64().unwrap()).into();
                }
            }
            for spring in u["springs"].as_array_mut().unwrap() {
                for name in ["fixed_anchor_m", "upright_local_point_m"] {
                    spring[name][0] = (-spring[name][0].as_f64().unwrap()).into();
                }
            }
            if index == 0 {
                u["steering"]["axis_in_source"] = serde_json::json!([-1, 0, 0]);
            }
            c["chassis"]["velocity_world_m_s"] = serde_json::json!([-0.05, -0.01, 0.1]);
            c["chassis"]["angular_velocity_world_rad_s"] = serde_json::json!([0.01, -0.02, 0.03]);
            let mut right = load_config(&dir, &c).unwrap();
            for _ in 0..50 {
                let command = Command {
                    drive: 0.5,
                    brake: 0.1,
                    steering: 0.4,
                };
                let ls = left.live_step(command, actuator()).unwrap();
                let rs = right.live_step(command, actuator()).unwrap();
                near(
                    rs.linear_momentum_world_kg_m_s,
                    mirror(ls.linear_momentum_world_kg_m_s),
                );
                near(
                    rs.angular_momentum_about_world_origin_kg_m2_s,
                    axial(ls.angular_momentum_about_world_origin_kg_m2_s),
                );
                assert!((ls.kinetic_energy_j - rs.kinetic_energy_j).abs() < 1e-7);
                let l = left.system.kinematics().unwrap();
                let r = right.system.kinematics().unwrap();
                near(r.chassis.position_m, mirror(l.chassis.position_m));
                near(
                    r.chassis.angular_velocity_world_rad_s,
                    axial(l.chassis.angular_velocity_world_rad_s),
                );
                let lw = l.uprights[0].wheel.as_ref().unwrap();
                let rw = r.uprights[0].wheel.as_ref().unwrap();
                near(rw.body.position_m, mirror(lw.body.position_m));
                assert!((lw.state.rate_rad_s - rw.state.rate_rad_s).abs() < 1e-8);
                let q = l.chassis.orientation_xyzw;
                let expected = DQuat::from_xyzw(q[0], -q[1], -q[2], q[3]);
                assert!(
                    expected
                        .dot(DQuat::from_array(r.chassis.orientation_xyzw))
                        .abs()
                        > 1. - 1e-12
                );
                let lc = left.environment.contacts(&l).unwrap();
                let rc = right.environment.contacts(&r).unwrap();
                near(
                    rc[0].normal.as_ref().unwrap().force_world_n,
                    mirror(lc[0].normal.as_ref().unwrap().force_world_n),
                );
                assert!(
                    (lc[0].tangential.as_ref().unwrap().dissipated_power_w
                        - rc[0].tangential.as_ref().unwrap().dissipated_power_w)
                        .abs()
                        < 1e-7
                );
            }
        }
    }
    #[test]
    fn source_radius_reaches_contact_law_and_source_changes_are_reloaded() {
        let (dir, mut c) = source_fixture();
        c["support_plane"] = serde_json::json!({"point_world_m":[0,0,0],"normal_world":[0,1,0]});
        c["chassis"]["position_world_m"] = serde_json::json!([0., 0.515, 0.]);
        for u in c["uprights"].as_array_mut().unwrap() {
            u["wheel"]["normal_contact"] = serde_json::json!({"stiffness_n_m":200000,"damping_n_s_m":1500,"max_penetration_m":0.08});
        }
        let bench = load_config(&dir, &c).unwrap();
        let k = bench.system.kinematics().unwrap();
        let normal = bench.environment.contacts(&k).unwrap();
        let first = normal[0].normal.as_ref().unwrap();
        assert!((first.gap_m + 0.005).abs() < 1e-10);
        assert!((first.normal_force_n - 1000.).abs() < 1e-6);
        let file = dir.0.join("data/tyres.ini");
        let tyres = std::fs::read_to_string(&file)
            .unwrap()
            .replace("RADIUS=.32", "RADIUS=.34");
        std::fs::write(file, tyres).unwrap();
        let next = load_config(&dir, &c).unwrap();
        assert_eq!(
            next.sources[0]["resolved_wheel_geometry"]["normal_radius_m"],
            0.34
        );
        let contacts = next
            .environment
            .contacts(&next.system.kinematics().unwrap())
            .unwrap();
        assert!((contacts[0].normal.as_ref().unwrap().normal_force_n - 5000.).abs() < 1e-6);
        let source_file = dir.0.join("data/suspensions.ini");
        let source = std::fs::read_to_string(&source_file)
            .unwrap()
            .replace("TRACK=1.5", "TRACK=1.6");
        std::fs::write(source_file, source).unwrap();
        let changed = load_config(&dir, &c).unwrap();
        assert_eq!(
            changed.sources[0]["resolved_wheel_geometry"]["wheel_center_in_chassis_m"][0],
            0.8
        );
    }
    #[test]
    fn ambiguous_or_incomplete_source_geometry_configuration_is_rejected() {
        let (dir, c) = source_fixture();
        let mut bad = Vec::new();
        let mut v = c.clone();
        v["uprights"][0]["frame_position_in_chassis_m"] = serde_json::json!([0, 0, 0]);
        bad.push(v);
        let mut v = c.clone();
        v["support_plane"] = serde_json::json!({"point_world_m":[0,0,0],"normal_world":[0,1,0]});
        v["uprights"][0]
            .as_object_mut()
            .unwrap()
            .remove("source_wheel_geometry");
        v["uprights"][0]["frame_position_in_chassis_m"] = serde_json::json!([0.8, 0, 1.2]);
        v["uprights"][0]["wheel"]["normal_contact"] = serde_json::json!({"stiffness_n_m":200000,"damping_n_s_m":1500,"max_penetration_m":0.08});
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0]
            .as_object_mut()
            .unwrap()
            .remove("source_wheel_geometry");
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0]["source_wheel_geometry"]["corner"] = "rear_left".into();
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][1]["source_wheel_geometry"]["corner"] = "front_left".into();
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0].as_object_mut().unwrap().remove("wheel");
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0]["source_wheel_geometry"]["corner"] = "guess".into();
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0]["source_wheel_geometry"]["compound"] = 100.into();
        bad.push(v);
        let mut v = c.clone();
        v["uprights"][0]["source_wheel_geometry"]["extra"] = true.into();
        bad.push(v);
        let mut v = c.clone();
        v["support_plane"] = serde_json::json!({"point_world_m":[0,0,0],"normal_world":[0,1,0]});
        v["uprights"][0]["wheel"]["normal_contact"] = serde_json::json!({"radius_m":0.3,"stiffness_n_m":200000,"damping_n_s_m":1500,"max_penetration_m":0.08});
        bad.push(v);
        for v in bad {
            assert!(load_config(&dir, &v).is_err());
        }
    }
    #[test]
    fn live_commands_override_offline_schedule_steer_brake_reverse_and_reset_mechanical_state() {
        let (_dir, mut bench) = fixture();
        let initial = serde_json::to_value(bench.system.kinematics().unwrap()).unwrap();
        let mut peak = 0_f64;
        for i in 0..1600 {
            let input = if i < 800 {
                Command {
                    drive: 1.,
                    steering: 1.,
                    brake: 0.,
                }
            } else {
                Command {
                    drive: 0.,
                    steering: 0.,
                    brake: 1.,
                }
            };
            let sample = bench.live_step(input, actuator()).unwrap();
            peak = peak.max(sample.chassis.velocity_world_m_s[2]);
        }
        let k = bench.system.kinematics().unwrap();
        assert!(peak > 0.1);
        assert!(k.chassis.velocity_world_m_s[2] < 0.8 * peak);
        assert!(bench.time_s() > 0.79);
        assert!(k.uprights[0].rack_motion.position_m[0] > 0.);
        bench.reset();
        assert_eq!(bench.time_s(), 0.);
        assert_eq!(
            serde_json::to_value(bench.system.kinematics().unwrap()).unwrap(),
            initial
        );
        for _ in 0..400 {
            bench
                .live_step(
                    Command {
                        drive: -1.,
                        ..Command::default()
                    },
                    actuator(),
                )
                .unwrap();
        }
        assert!(
            bench
                .system
                .kinematics()
                .unwrap()
                .chassis
                .velocity_world_m_s[2]
                < -0.01
        );
    }
    #[test]
    fn invalid_live_commands_preserve_state_and_clock() {
        let (_dir, mut bench) = fixture();
        let before = serde_json::to_value(bench.system.kinematics().unwrap()).unwrap();
        assert!(
            bench
                .live_step(
                    Command {
                        drive: f64::NAN,
                        ..Command::default()
                    },
                    actuator()
                )
                .is_err()
        );
        assert_eq!(bench.time_s(), 0.);
        assert_eq!(
            serde_json::to_value(bench.system.kinematics().unwrap()).unwrap(),
            before
        );
    }
}
