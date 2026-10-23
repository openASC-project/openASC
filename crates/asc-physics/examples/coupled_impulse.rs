//! Original synthetic point-impact fixture. No geometry collision detector or game assets.
use asc_physics::{
    Body, ChassisSystem, FiveLink, LinkPose, MassProperties, MountedUpright, TravelMechanism,
    TravelState, Wheel, WheelState,
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mechanism = FiveLink::new(
        [
            [-0.3, 0.2, 0.12],
            [-0.3, 0.2, -0.12],
            [-0.3, -0.2, 0.12],
            [-0.3, -0.2, -0.12],
            [-0.3, 0., 0.15],
        ],
        [
            [0., 0.2, 0.],
            [0., 0.2, 0.],
            [0., -0.2, 0.],
            [0., -0.2, 0.],
            [0., 0., 0.15],
        ],
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        [0., 1., 0.],
    )?;
    let chassis = Body::new(
        MassProperties::new(300., [[220., 0., 0.], [0., 400., 0.], [0., 0., 350.]])?,
        [0.; 3],
        [0., 0., 0., 1.],
        [-2., 0., 0.],
        [0.; 3],
    )?;
    let mut upright = MountedUpright::new(
        TravelMechanism::Dwb(mechanism),
        MassProperties::new(35., [[0.4, 0., 0.], [0., 0.5, 0.], [0., 0., 0.6]])?,
        [0.; 3],
        [0.; 3],
        LinkPose::new([0.; 3], [0., 0., 0., 1.])?,
        TravelState {
            travel_m: 0.,
            rate_m_s: 0.,
        },
    )?;
    upright.attach_wheel(Wheel::new(
        [0.; 3],
        [1., 0., 0.],
        20.,
        1.2,
        0.7,
        WheelState {
            angle_rad: 0.,
            rate_rad_s: 0.,
        },
    )?)?;
    let mut system = ChassisSystem::new(chassis, vec![upright], [0.; 3])?;
    let initial = system.kinematics()?;
    let point = [0., 0.2, 0.3];
    let direction = [1., 0., 0.];
    let restitution = 0.4;
    let response = system.chassis_point_response(point, direction)?;
    let impulse = (1. + restitution) * 2. / response.inverse_effective_mass_kg_inv;
    let sample = system.impulse_at_world_point([impulse, 0., 0.], point)?;
    let after = system.kinematics()?;
    assert!((sample.point_velocity_after_world_m_s[0] - 0.8).abs() < 1e-10);
    assert!(sample.kinetic_energy_after_j < sample.kinetic_energy_before_j);
    assert!(sample.energy_balance_residual_j.abs() < 1e-8);
    assert_eq!(initial.chassis.position_m, after.chassis.position_m);
    assert_eq!(
        initial.chassis.orientation_xyzw,
        after.chassis.orientation_xyzw
    );
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "scope":"native_coupled_chassis_impulse_example","point_world_m":point,"normal_world":direction,"restitution":restitution,
            "response":response,"impulse_world_ns":[impulse,0.,0.],"sample":sample,"initial":initial,"after":after,
            "geometry_collision_detection":false,"original_physics_validated":false,"simulation_ready":false
        }))?
    );
    Ok(())
}
