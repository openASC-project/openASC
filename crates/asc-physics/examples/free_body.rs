//! Original numerical fixture, not a car/track simulation or an original-engine comparison.
use asc_physics::{Body, FixedClock, MassProperties, Wrench};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let properties = MassProperties::new(2., [[2., 0., 0.], [0., 3., 0.], [0., 0., 4.]])?;
    let mut body = Body::new(
        properties,
        [0., 10., 0.],
        [0., 0., 0., 1.],
        [0.; 3],
        [0.; 3],
    )?;
    let initial = body.state();
    let load = Wrench::new([2., -19.62, 3.], [1., 0.5, 0.25])?;
    let mut clock = FixedClock::new(120, 8)?;
    let first = clock.advance(Duration::from_secs(1), |tick| {
        body.step(tick.dt_seconds, load)
    })?;
    assert_eq!(first.executed_steps, 8);
    assert_eq!(first.pending_steps, 112);
    while clock.pending_steps() > 0 {
        clock.advance(Duration::ZERO, |tick| body.step(tick.dt_seconds, load))?;
    }
    let state = body.state();
    for (actual, expected) in state.position_m.into_iter().zip([0.5, 5.095, 0.75]) {
        assert!((actual - expected).abs() < 1e-10);
    }
    for (actual, expected) in state
        .angular_momentum_world_kg_m2_s
        .into_iter()
        .zip([1., 0.5, 0.25])
    {
        assert!((actual - expected).abs() < 1e-12);
    }
    assert_eq!(clock.completed_ticks(), 120);
    println!(
        "{}",
        serde_json::to_string_pretty(&serde_json::json!({
            "scope":"native_unconstrained_body_example","mass_kg":2.,"inertia_rows_kg_m2":[[2.,0.,0.],[0.,3.,0.],[0.,0.,4.]],
            "hz":120,"completed_ticks":clock.completed_ticks(),"pending_steps":clock.pending_steps(),
            "initial":initial,"final":state,"original_physics_validated":false,"simulation_ready":false
        }))?
    );
    Ok(())
}
