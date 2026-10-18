//! Source-aware DWB/STRUT configuration records, not a kinematic/force reconstruction.
use crate::{
    Diagnostic, FieldSource, Result,
    data::{CarData, Mode},
    fields::Fields,
};
use serde::Serialize;
use std::{collections::BTreeMap, path::Path};

#[derive(Serialize)]
pub struct Axle {
    #[serde(rename = "type")]
    pub kind: String,
    pub track_m: f64,
    pub base_y_m: f64,
    pub rod_length_m: f64,
    pub hub_mass_kg: f64,
    pub hardpoints: BTreeMap<String, [f64; 3]>,
    /// Original numeric names/values; no automatic force/threshold/stop interpretation.
    pub source_parameters: BTreeMap<String, f64>,
}
#[derive(Serialize)]
pub struct Suspension {
    pub scope: &'static str,
    pub car: String,
    pub data_source: &'static str,
    pub header_version: u32,
    pub wheelbase_m: f64,
    pub front_weight_fraction: f64,
    pub axles: BTreeMap<String, Axle>,
    pub antiroll_source_parameters: BTreeMap<String, f64>,
    pub field_sources: BTreeMap<String, FieldSource>,
    pub diagnostics: Vec<Diagnostic>,
    pub simulation_ready: bool,
    pub has_errors: bool,
}
pub fn load(directory: &Path, mode: Mode) -> Result<Suspension> {
    let mut data = CarData::open(directory, mode)?;
    let mut f = Fields::new(data.ini("suspensions.ini")?);
    f.version(&[1, 2])?;
    let header_version = f.integer("HEADER", "VERSION", 0, 10000)?;
    let wheelbase_m = f.number("BASIC", "WHEELBASE", 0., 100., true)?;
    let front_weight_fraction = f.number("BASIC", "CG_LOCATION", 0., 1., true)?;
    if front_weight_fraction >= 1. {
        return Err(f.error(
            "BASIC",
            "CG_LOCATION",
            "suspension_range",
            "Front weight fraction must be strictly between zero and one.",
        ));
    }
    let mut axles = BTreeMap::new();
    for section in ["FRONT", "REAR"] {
        let kind = f.value(section, "TYPE")?;
        if !["STRUT", "DWB"].contains(&kind.as_str()) {
            return Err(f.error(
                section,
                "TYPE",
                "suspension_type",
                "Only the explicit DWB/STRUT configuration subset is supported.",
            ));
        }
        let track_m = f.number(section, "TRACK", 0., 100., true)?;
        let base_y_m = f.number(section, "BASEY", -100., 100., false)?;
        let rod_length_m = f.number(section, "ROD_LENGTH", -100., 100., false)?;
        let hub_mass_kg = f.number(section, "HUB_MASS", 0., 1e6, true)?;
        let mut names = vec![
            "WBCAR_BOTTOM_FRONT",
            "WBCAR_BOTTOM_REAR",
            "WBTYRE_BOTTOM",
            "WBCAR_STEER",
            "WBTYRE_STEER",
        ];
        if kind == "DWB" {
            names.extend(["WBCAR_TOP_FRONT", "WBCAR_TOP_REAR", "WBTYRE_TOP"]);
        } else {
            names.extend(["STRUT_CAR", "STRUT_TYRE"]);
        }
        let mut hardpoints = BTreeMap::new();
        for name in names {
            let v = f.vector(section, name, false)?;
            if v.iter().any(|x| x.abs() > 1000.) {
                return Err(f.error(
                    section,
                    name,
                    "suspension_geometry_range",
                    "Source hardpoint coordinates exceed +/-1000 metres.",
                ));
            }
            hardpoints.insert(name.into(), v);
        }
        let mut parameters = BTreeMap::new();
        for name in [
            "SPRING_RATE",
            "PROGRESSIVE_SPRING_RATE",
            "BUMP_STOP_RATE",
            "BUMPSTOP_UP",
            "BUMPSTOP_DN",
            "PACKER_RANGE",
            "DAMP_BUMP",
            "DAMP_FAST_BUMP",
            "DAMP_FAST_BUMPTHRESHOLD",
            "DAMP_REBOUND",
            "DAMP_FAST_REBOUND",
            "DAMP_FAST_REBOUNDTHRESHOLD",
        ] {
            parameters.insert(name.into(), f.number(section, name, 0., 1e12, false)?);
        }
        for name in ["TOE_OUT", "STATIC_CAMBER"] {
            parameters.insert(name.into(), f.number(section, name, -1e6, 1e6, false)?);
        }
        axles.insert(
            section.to_lowercase(),
            Axle {
                kind,
                track_m,
                base_y_m,
                rod_length_m,
                hub_mass_kg,
                hardpoints,
                source_parameters: parameters,
            },
        );
    }
    let mut antiroll = BTreeMap::new();
    if f.document.sections.iter().any(|s| s.name == "ARB") {
        for name in ["FRONT", "REAR"] {
            antiroll.insert(name.into(), f.number("ARB", name, 0., 1e12, false)?);
        }
    }
    let total: usize = f.document.sections.iter().map(|s| s.values.len()).sum();
    if total > f.used.len() {
        data.diagnostics.push(Diagnostic::warning("suspension_fields_uninterpreted",&f.document.source,&format!("{} other fields are retained only in the original source; heave, damage, graphics and extra force definitions are not solved.",total-f.used.len())));
    }
    data.diagnostics.push(Diagnostic::warning("suspension_configuration_scope",&f.document.source,"Source configuration only. Hardpoint frames/mirroring, kinematic constraints, unsprung-mass assignment, preload, progressive/stop laws and the original damper interpolation are not validated or simulated. No automatic mapping to the independent spring link."));
    Ok(Suspension {
        scope: "native_suspension_configuration",
        car: data.car,
        data_source: data.mode,
        header_version,
        wheelbase_m,
        front_weight_fraction,
        axles,
        antiroll_source_parameters: antiroll,
        field_sources: f.used,
        diagnostics: data.diagnostics,
        simulation_ready: false,
        has_errors: false,
    })
}
