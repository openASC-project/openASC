//! Original coupled-oscillator fixture; no AC geometry, force curves or vehicle data.
use asc_physics::{Body, Damper, DamperBranch, FixedClock, MassProperties, SpringLink, Wrench};
use std::time::Duration;
fn body(mass: f64, x: f64) -> Result<Body, asc_physics::Error> {
    Body::new(
        MassProperties::new(mass, [[2., 0., 0.], [0., 2., 0.], [0., 0., 2.]])?,
        [x, 0., 0.],
        [0., 0., 0., 1.],
        [0.; 3],
        [0.; 3],
    )
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut a = body(2., 0.)?;
    let mut b = body(3., 1.2)?;
    let branch = DamperBranch::new(4.8, 4.8, 1.)?;
    let link = SpringLink::new([0.; 3], [0.; 3], 1., 60., Damper::new(branch, branch))?;
    let mut clock = FixedClock::new(240, 16)?;
    clock.advance(Duration::from_secs(1), |tick| {
        link.step_pair(
            &mut a,
            &mut b,
            tick.dt_seconds,
            Wrench::default(),
            Wrench::default(),
        )
        .map(|_| ())
    })?;
    while clock.pending_steps() > 0 {
        clock.advance(Duration::ZERO, |tick| {
            link.step_pair(
                &mut a,
                &mut b,
                tick.dt_seconds,
                Wrench::default(),
                Wrench::default(),
            )
            .map(|_| ())
        })?;
    }
    let state_a = a.state();
    let state_b = b.state();
    let sample = link.loads(&a, &b)?.2;
    let omega = 46f64.sqrt();
    let exact = 0.2 * (-2f64).exp() * (omega.cos() + 2. / omega * omega.sin());
    let error = (sample.extension_m - exact).abs();
    assert!(error < 1e-5);
    assert!(((2. * state_a.position_m[0] + 3. * state_b.position_m[0]) / 5. - 0.72).abs() < 1e-12);
    assert!(
        (state_a.linear_momentum_world_kg_m_s[0] + state_b.linear_momentum_world_kg_m_s[0]).abs()
            < 1e-12
    );
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"scope":"native_two_body_spring_fixture","completed_ticks":clock.completed_ticks(),"body_a":state_a,"body_b":state_b,"final_link":sample,"analytic_extension_m":exact,"extension_error_m":error,"original_suspension_validated":false,"simulation_ready":false})
        )?
    );
    Ok(())
}
