use std::{error::Error, path::Path};

pub fn motion(
    directory: &Path,
    axle: &str,
    travel: f64,
    axis: [f64; 3],
    rack: [f64; 3],
    mode: asc_vehicle::data::Mode,
) -> Result<serde_json::Value, Box<dyn Error>> {
    let source = asc_vehicle::suspension::load(directory, mode)?;
    let selected = source
        .axles
        .get(axle)
        .ok_or("Choose an explicit front or rear axle")?;
    let mechanism = asc_session::dwb_geometry(selected, axis)?;
    let solution = mechanism.solve(travel, rack, None)?;
    Ok(serde_json::json!({
        "scope": "native_dwb_raw_design_geometry", "car": source.car,
        "data_source": source.data_source, "axle": axle,
        "travel_axis_in_source_coordinates": axis, "travel_m": travel,
        "steering_anchor_displacement_in_source_coordinates_m": rack,
        "assumed_design_pose": { "position_m": [0.,0.,0.], "orientation_xyzw": [0.,0.,0.,1.] },
        "rest_link_lengths_m": mechanism.rest_lengths_m(), "solution": solution,
        "simulation_ready": false, "original_kinematics_validated": false,
        "source_diagnostics": source.diagnostics,
        "limitations": ["Raw common hardpoint coordinates are assumed to describe a rigid design upright",
            "No game-side frame/mirroring, static camber/toe or BASEY/track/rod corrections",
            "No strut constraints, forces, unsprung mass or chassis equilibrium",
            "Local assembly branch only; singular/unreachable configurations fail explicitly"]
    }))
}
