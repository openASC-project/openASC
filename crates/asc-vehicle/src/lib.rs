//! Typed vehicle configuration loading, not original vehicle physics.
pub mod data;
mod fields;
use fields::Fields;
use serde::Serialize;
use std::{collections::BTreeMap, io::Read, path::Path};
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

#[derive(Serialize)]
pub struct Diagnostic {
    pub code: &'static str,
    pub message: String,
    pub source: String,
    pub line: usize,
    pub severity: &'static str,
}
impl Diagnostic {
    pub fn warning(code: &'static str, source: &str, message: &str) -> Self {
        Self {
            code,
            message: message.into(),
            source: source.into(),
            line: 0,
            severity: "warning",
        }
    }
}
#[derive(Serialize)]
pub struct FieldSource {
    pub source: String,
    pub line: usize,
}
#[derive(Serialize)]
pub struct Axle {
    #[serde(rename = "type")]
    pub kind: String,
    pub track_m: f64,
    pub base_y_m: f64,
    pub tyre_name: String,
    pub tyre_radius_m: f64,
    pub tyre_width_m: f64,
}
#[derive(Serialize)]
pub struct TorqueLut {
    pub source: String,
    pub points: Vec<[f64; 2]>,
    pub x_unit: &'static str,
    pub y_unit: &'static str,
}
#[derive(Serialize)]
pub struct Engine {
    pub idle_rpm: f64,
    pub limiter_rpm: f64,
    pub inertia_kg_m2: f64,
    pub torque_lut: TorqueLut,
}
#[derive(Serialize)]
pub struct Drivetrain {
    pub traction: String,
    pub forward_ratios: Vec<f64>,
    pub reverse_ratio: f64,
    pub final_ratio: f64,
}
#[derive(Serialize)]
pub struct Brakes {
    pub max_torque_nm: f64,
    pub front_share: f64,
    pub handbrake_torque_nm: f64,
}
#[derive(Serialize)]
pub struct Header {
    pub version: u32,
    pub scope: &'static str,
    pub size_bytes: u64,
}
#[derive(Serialize)]
pub struct Model {
    pub file: String,
    pub kn5: Header,
}
#[derive(Serialize)]
pub struct Vehicle {
    pub car: String,
    pub data_source: &'static str,
    pub mass_kg: f64,
    pub inertia_dimensions_m: [f64; 3],
    pub graphics_offset_m: [f64; 3],
    pub graphics_pitch_deg: f64,
    pub steer_lock_deg: f64,
    pub steer_ratio: f64,
    pub wheelbase_m: f64,
    pub front_weight_fraction: f64,
    pub axles: BTreeMap<String, Axle>,
    pub engine: Engine,
    pub drivetrain: Drivetrain,
    pub brakes: Brakes,
    pub compound: u32,
    pub model: Model,
    pub field_sources: BTreeMap<String, BTreeMap<String, FieldSource>>,
    pub diagnostics: Vec<Diagnostic>,
    pub wheel_offsets_m: BTreeMap<String, [f64; 3]>,
    pub scope: &'static str,
    pub simulation_ready: bool,
    pub has_errors: bool,
}
pub fn load(directory: &Path, mode: data::Mode, compound: Option<u32>) -> Result<Vehicle> {
    let mut data = data::CarData::open(directory, mode)?;
    let mut car = Fields::new(data.ini("car.ini")?);
    let mut suspension = Fields::new(data.ini("suspensions.ini")?);
    let mut tyres = Fields::new(data.ini("tyres.ini")?);
    let mut engine = Fields::new(data.ini("engine.ini")?);
    let mut drive = Fields::new(data.ini("drivetrain.ini")?);
    let mut brake = Fields::new(data.ini("brakes.ini")?);
    let mut lod = Fields::new(data.ini("lods.ini")?);
    car.version(&[1])?;
    suspension.version(&[1, 2])?;
    tyres.version(&[7, 10])?;
    engine.version(&[1])?;
    drive.version(&[1, 2, 3])?;
    brake.version(&[1])?;
    let mass_kg = car.number("BASIC", "TOTALMASS", 0., f64::MAX, true)?;
    let inertia_dimensions_m = car.vector("BASIC", "INERTIA", true)?;
    let graphics_offset_m = car.vector("BASIC", "GRAPHICS_OFFSET", false)?;
    let graphics_pitch_deg = car.number(
        "BASIC",
        "GRAPHICS_PITCH_ROTATION",
        -f64::MAX,
        f64::MAX,
        false,
    )?;
    let steer_lock_deg = car.number("CONTROLS", "STEER_LOCK", 0., f64::MAX, true)?;
    let steer_ratio = car.number("CONTROLS", "STEER_RATIO", 0., f64::MAX, true)?;
    let wheelbase_m = suspension.number("BASIC", "WHEELBASE", 0., f64::MAX, true)?;
    let front_weight_fraction = suspension.number("BASIC", "CG_LOCATION", 0., 1., true)?;
    if front_weight_fraction >= 1. {
        return Err(suspension.error(
            "BASIC",
            "CG_LOCATION",
            "vehicle_range",
            "Front weight fraction must be strictly between zero and one.",
        ));
    }
    let compound = match compound {
        Some(v) if v <= 99 => v,
        Some(_) => {
            return Err(asc_config::Error::new(
                "vehicle_compound",
                "<car>",
                0,
                "Compound index must be 0..99.",
            )
            .into());
        }
        None => tyres.integer("COMPOUND_DEFAULT", "INDEX", 0, 99)?,
    };
    let mut axles = BTreeMap::new();
    let mut wheel_offsets_m = BTreeMap::new();
    for name in ["FRONT", "REAR"] {
        let section = if compound == 0 {
            name.into()
        } else {
            format!("{name}_{compound}")
        };
        let axle = Axle {
            kind: suspension.value(name, "TYPE")?,
            track_m: suspension.number(name, "TRACK", 0., f64::MAX, true)?,
            base_y_m: suspension.number(name, "BASEY", -f64::MAX, f64::MAX, false)?,
            tyre_name: tyres.value(&section, "NAME")?,
            tyre_radius_m: tyres.number(&section, "RADIUS", 0., f64::MAX, true)?,
            tyre_width_m: tyres.number(&section, "WIDTH", 0., f64::MAX, true)?,
        };
        let z = if name == "FRONT" {
            wheelbase_m * (1. - front_weight_fraction)
        } else {
            -wheelbase_m * front_weight_fraction
        };
        for (side, sign) in [("left", 1.), ("right", -1.)] {
            wheel_offsets_m.insert(
                format!("{}_{side}", name.to_lowercase()),
                [sign * (axle.track_m / 2.), axle.base_y_m, z],
            );
        }
        axles.insert(name.to_lowercase(), axle);
    }
    let idle_rpm = engine.number("ENGINE_DATA", "MINIMUM", 0., f64::MAX, false)?;
    let limiter_rpm = engine.number("ENGINE_DATA", "LIMITER", 0., f64::MAX, true)?;
    if limiter_rpm <= idle_rpm {
        return Err(engine.error(
            "ENGINE_DATA",
            "LIMITER",
            "vehicle_range",
            "Limiter must exceed idle speed.",
        ));
    }
    let inertia_kg_m2 = engine.number("ENGINE_DATA", "INERTIA", 0., f64::MAX, true)?;
    let curve_name = engine.value("HEADER", "POWER_CURVE")?;
    asc_acd::data_name(&curve_name)
        .map_err(|e| engine.error("HEADER", "POWER_CURVE", e.code, &e.message))?;
    let curve_source = data.source(&curve_name);
    let curve = asc_config::parse_lut(data.text(&curve_name)?, &curve_source)?;
    let points = curve.points();
    if points.len() < 2
        || points[0][0] < 0.
        || points[0][0] > idle_rpm
        || points.last().unwrap()[0] < limiter_rpm
    {
        return Err(asc_config::Error::new(
            "vehicle_power_domain",
            &curve_source,
            0,
            "Torque LUT requires nonnegative RPM coordinates covering idle through limiter.",
        )
        .into());
    }
    let engine_config = Engine {
        idle_rpm,
        limiter_rpm,
        inertia_kg_m2,
        torque_lut: TorqueLut {
            source: curve_source,
            points: points.to_vec(),
            x_unit: "rpm",
            y_unit: "Nm",
        },
    };
    let traction = drive.value("TRACTION", "TYPE")?.to_uppercase();
    if !["RWD", "FWD", "AWD"].contains(&traction.as_str()) {
        return Err(drive.error(
            "TRACTION",
            "TYPE",
            "vehicle_traction",
            "Unknown traction type.",
        ));
    }
    let count = drive.integer("GEARS", "COUNT", 1, 10)?;
    let mut forward_ratios = Vec::new();
    for i in 1..=count {
        forward_ratios.push(drive.number("GEARS", &format!("GEAR_{i}"), 0., f64::MAX, true)?);
    }
    let reverse_ratio = drive.number("GEARS", "GEAR_R", -f64::MAX, f64::MAX, false)?;
    if reverse_ratio >= 0. {
        return Err(drive.error(
            "GEARS",
            "GEAR_R",
            "vehicle_range",
            "Reverse ratio must be negative.",
        ));
    }
    let final_ratio = drive.number("GEARS", "FINAL", 0., f64::MAX, true)?;
    let brakes = Brakes {
        max_torque_nm: brake.number("DATA", "MAX_TORQUE", 0., f64::MAX, true)?,
        front_share: brake.number("DATA", "FRONT_SHARE", 0., 1., false)?,
        handbrake_torque_nm: brake.number("DATA", "HANDBRAKE_TORQUE", 0., f64::MAX, false)?,
    };
    let model_name = lod.value("LOD_0", "FILE")?;
    let path = data::confined(
        &data.root,
        &model_name,
        &lod.document.source,
        lod.line("LOD_0", "FILE"),
    )?;
    if !path
        .extension()
        .is_some_and(|v| v.eq_ignore_ascii_case("kn5"))
        || !path.is_file()
    {
        return Err(lod.error(
            "LOD_0",
            "FILE",
            "vehicle_model",
            "Expected an existing KN5 model.",
        ));
    }
    let mut header = Vec::new();
    std::fs::File::open(&path)?
        .take(14)
        .read_to_end(&mut header)?;
    if header.len() < 10 || &header[..6] != b"sc6969" {
        return Err(lod.error(
            "LOD_0",
            "FILE",
            "kn5_header",
            "Missing or truncated KN5 signature/version.",
        ));
    }
    let version = u32::from_le_bytes(header[6..10].try_into().unwrap());
    if ![5, 6].contains(&version) {
        return Err(lod.error(
            "LOD_0",
            "FILE",
            "kn5_version",
            "Only KN5 v5/v6 headers are supported.",
        ));
    }
    if version == 6 && header.len() < 14 {
        return Err(lod.error("LOD_0", "FILE", "kn5_header", "Truncated KN5 v6 header."));
    }
    let model = Model {
        file: path
            .strip_prefix(&data.root)?
            .components()
            .map(|v| v.as_os_str().to_string_lossy())
            .collect::<Vec<_>>()
            .join("/"),
        kn5: Header {
            version,
            scope: "header_only",
            size_bytes: path.metadata()?.len(),
        },
    };
    if engine
        .document
        .sections
        .iter()
        .any(|s| s.name.starts_with("TURBO_"))
    {
        data.diagnostics.push(Diagnostic::warning("vehicle_turbo_unimplemented",&engine.document.source,"Turbo definitions are present; the base torque curve does not describe boosted output."));
    }
    let mut field_sources = BTreeMap::new();
    for (name, f) in [
        ("car.ini", car),
        ("suspensions.ini", suspension),
        ("tyres.ini", tyres),
        ("engine.ini", engine),
        ("drivetrain.ini", drive),
        ("brakes.ini", brake),
        ("lods.ini", lod),
    ] {
        let total: usize = f.document.sections.iter().map(|s| s.values.len()).sum();
        if total > f.used.len() {
            data.diagnostics.push(Diagnostic::warning(
                "vehicle_fields_uninterpreted",
                &f.document.source,
                &format!(
                    "{name}: {} other fields are not interpreted.",
                    total - f.used.len()
                ),
            ));
        }
        field_sources.insert(name.into(), f.used);
    }
    data.diagnostics.push(Diagnostic::warning("vehicle_configuration_scope",&format!("content/cars/{}",data.car),"Configuration only. Inertia dimensions are not a tensor. Original tyre/suspension/drivetrain/fuel/coast/aero/electronics behavior, audio and FFB are not simulated; model validation is header-only."));
    Ok(Vehicle {
        car: data.car,
        data_source: data.mode,
        mass_kg,
        inertia_dimensions_m,
        graphics_offset_m,
        graphics_pitch_deg,
        steer_lock_deg,
        steer_ratio,
        wheelbase_m,
        front_weight_fraction,
        axles,
        engine: engine_config,
        drivetrain: Drivetrain {
            traction,
            forward_ratios,
            reverse_ratio,
            final_ratio,
        },
        brakes,
        compound,
        model,
        field_sources,
        diagnostics: data.diagnostics,
        wheel_offsets_m,
        scope: "vehicle_configuration_subset",
        simulation_ready: false,
        has_errors: false,
    })
}
