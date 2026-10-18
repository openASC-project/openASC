use asc_physics::{Body, Error, FixedClock, MassProperties, Wrench};
use std::time::Duration;

fn body() -> Body {
    Body::new(
        MassProperties::new(2., [[2., 0., 0.], [0., 3., 0.], [0., 0., 4.]]).unwrap(),
        [0.; 3],
        [0., 0., 0., 1.],
        [0.; 3],
        [0.; 3],
    )
    .unwrap()
}
#[test]
fn frame_grouping_changes_neither_tick_sequence_nor_body_result() {
    let load = Wrench::new([1., 2., 3.], [0., 0.2, 0.]).unwrap();
    let mut results = Vec::new();
    for frames in [
        vec![Duration::from_secs(1)],
        vec![Duration::from_millis(10); 100],
    ] {
        let mut clock = FixedClock::new(120, 1000).unwrap();
        let mut b = body();
        let mut indices = Vec::new();
        for elapsed in frames {
            clock
                .advance(elapsed, |tick| {
                    indices.push(tick.index);
                    b.step(tick.dt_seconds, load)
                })
                .unwrap();
        }
        assert_eq!(clock.completed_ticks(), 120);
        assert_eq!(indices, (0..120).collect::<Vec<_>>());
        results.push(b.state());
    }
    assert_eq!(results[0], results[1]);
}
#[test]
fn nanosecond_credits_do_not_round_the_120_hz_step_and_backlog_is_retained() {
    let mut clock = FixedClock::new(120, 8).unwrap();
    let first = clock
        .advance(Duration::from_nanos(8_333_333), |_| panic!("not due"))
        .unwrap();
    assert_eq!(first.executed_steps, 0);
    let second = clock
        .advance(Duration::from_nanos(1), |tick| {
            assert_eq!(tick.index, 0);
            assert_eq!(tick.dt_seconds, 1. / 120.);
            Ok(())
        })
        .unwrap();
    assert_eq!(second.executed_steps, 1);
    assert!((second.interpolation_alpha.unwrap() - 8e-8).abs() < 1e-16);
    let mut clock = FixedClock::new(120, 8).unwrap();
    let first = clock.advance(Duration::from_secs(1), |_| Ok(())).unwrap();
    assert_eq!(first.executed_steps, 8);
    assert_eq!(first.pending_steps, 112);
    assert_eq!(first.interpolation_alpha, None);
    while clock.pending_steps() > 0 {
        clock.advance(Duration::ZERO, |_| Ok(())).unwrap();
    }
    assert_eq!(clock.completed_ticks(), 120);
    let end = clock
        .advance(Duration::ZERO, |_| panic!("not due"))
        .unwrap();
    assert_eq!(end.interpolation_alpha, Some(0.));
}
#[test]
fn callback_failure_retains_the_failed_tick_and_only_prior_ticks_are_committed() {
    let mut clock = FixedClock::new(120, 8).unwrap();
    let mut b = body();
    let mut attempted = Vec::new();
    let force = Wrench::new([2., 0., 0.], [0.; 3]).unwrap();
    let error = clock
        .advance(Duration::from_millis(25), |tick| {
            attempted.push(tick.index);
            if tick.index == 1 {
                return Err(Error::new("fixture_load", "reject before mutation"));
            }
            b.step(tick.dt_seconds, force)
        })
        .unwrap_err();
    assert_eq!(error.code, "fixture_load");
    assert_eq!(attempted, [0, 1]);
    assert_eq!(clock.completed_ticks(), 1);
    assert_eq!(clock.pending_steps(), 2);
    assert!((b.state().velocity_world_m_s[0] - 1. / 120.).abs() < 1e-14);
    let retry = clock
        .advance(Duration::ZERO, |tick| {
            attempted.push(tick.index);
            b.step(tick.dt_seconds, force)
        })
        .unwrap();
    assert_eq!(attempted, [0, 1, 1, 2]);
    assert_eq!(retry.completed_ticks, 3);
    assert_eq!(retry.pending_steps, 0);
    assert!((b.state().velocity_world_m_s[0] - 3. / 120.).abs() < 1e-14);
}
#[test]
fn invalid_config_and_time_overflow_are_explicit_without_mutating_the_clock() {
    for (hz, budget) in [(0, 8), (29, 8), (10001, 8), (120, 0), (120, 10001)] {
        assert_eq!(
            FixedClock::new(hz, budget).unwrap_err().code,
            "physics_clock_config"
        );
    }
    let mut clock = FixedClock::new(120, 8).unwrap();
    assert_eq!(
        clock
            .advance(Duration::MAX, |_| panic!("not due"))
            .unwrap_err()
            .code,
        "physics_clock_range"
    );
    assert_eq!(clock.completed_ticks(), 0);
    assert_eq!(clock.pending_steps(), 0);
}
