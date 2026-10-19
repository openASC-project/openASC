use std::{error::Error, path::Path};

#[derive(Default)]
pub struct MotionProbe {
    pub rates_m_s: [f64; 4],
    pub force_n: [f64; 3],
    pub torque_about_upright_origin_nm: [f64; 3],
}
impl MotionProbe {
    fn evaluate(
        &self,
        mapping: &asc_physics::LinkDifferential,
    ) -> Result<serde_json::Value, Box<dyn Error>> {
        let load = asc_physics::LinkLoad::new(self.force_n, self.torque_about_upright_origin_nm)?;
        let velocity = mapping.velocity(self.rates_m_s[0], self.rates_m_s[1..].try_into()?)?;
        Ok(serde_json::json!({
            "mapping":mapping,"input_rates_m_s":self.rates_m_s,"explicit_probe_load":load,
            "relative_velocity":velocity,"generalized_loads":mapping.project_load(load)?,
            "held_massless_balance":mapping.hold_load(load)?,
            "velocity_frame":"fixed_source_design_frame",
            "load_torque_origin":"upright_origin",
            "support_reaction_excludes_travel_actuator":true,
            "generated_tyre_or_spring_loads":false,"dynamic_vehicle_simulation":false
        }))
    }
}

pub fn motion(
    directory: &Path,
    axle: &str,
    travel: f64,
    axis: [f64; 3],
    rack: [f64; 3],
    mode: asc_vehicle::data::Mode,
    probe: Option<MotionProbe>,
) -> Result<serde_json::Value, Box<dyn Error>> {
    let source = asc_vehicle::suspension::load(directory, mode)?;
    let selected = source
        .axles
        .get(axle)
        .ok_or("Choose an explicit front or rear axle")?;
    let (scope, lengths, solution, strut_reference, differential) = match selected.kind.as_str() {
        "DWB" => {
            let mechanism = asc_session::dwb_geometry(selected, axis)?;
            let solution = mechanism.solve(travel, rack, None)?;
            let differential = probe
                .as_ref()
                .map(|p| p.evaluate(&mechanism.differential(travel, rack, solution.pose()?)?))
                .transpose()?;
            (
                "native_dwb_raw_design_geometry",
                serde_json::to_value(mechanism.rest_lengths_m())?,
                serde_json::to_value(solution)?,
                None,
                differential,
            )
        }
        "STRUT" => {
            let mechanism = asc_session::strut_geometry(selected, axis)?;
            let solution = mechanism.solve(travel, rack, None)?;
            let differential = probe
                .as_ref()
                .map(|p| p.evaluate(&mechanism.differential(travel, rack, solution.pose()?)?))
                .transpose()?;
            (
                "native_strut_raw_design_geometry",
                serde_json::to_value(mechanism.rest_lengths_m())?,
                serde_json::to_value(solution)?,
                Some(mechanism.reference_strut_length_m()),
                differential,
            )
        }
        _ => return Err("Unsupported suspension geometry type".into()),
    };
    let mut report = serde_json::json!({
        "scope": scope, "type": selected.kind, "car": source.car,
        "data_source": source.data_source, "axle": axle,
        "travel_axis_in_source_coordinates": axis, "travel_m": travel,
        "steering_anchor_displacement_in_source_coordinates_m": rack,
        "assumed_design_pose": { "position_m": [0.,0.,0.], "orientation_xyzw": [0.,0.,0.,1.] },
        "rest_link_lengths_m": lengths, "solution": solution,
        "simulation_ready": false, "original_kinematics_validated": false,
        "source_diagnostics": source.diagnostics,
        "limitations": ["Raw common hardpoint coordinates are assumed to describe a rigid design upright",
            "No game-side frame/mirroring, static camber/toe or BASEY/track/rod corrections",
            "Only explicitly requested ideal held-load probes; no tyre/spring force laws, inertia or chassis equilibrium",
            "Local assembly branch only; singular/unreachable configurations fail explicitly"]
    });
    if let Some(length) = strut_reference {
        report["reference_strut_length_m"] = length.into();
    }
    if let Some(differential) = differential {
        report["differential_probe"] = differential;
    }
    Ok(report)
}
