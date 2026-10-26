//! Transactional motion to the first WALL event, followed by an inelastic coupled impulse.
use crate::ColliderBox;
use asc_ground::{BoxMotion, WallContact, WallIndex, WallSweep, WallSweepSettings};
use asc_physics::{
    ChassisContact, ChassisContactSample, ChassisContactSettings, ChassisKinematics, ChassisLoads,
    ChassisPartialStep, ChassisSystem, RackMotion,
};
use asc_vehicle::Result;
use glam::DVec3;
use serde::Serialize;
#[derive(Clone, Copy, Debug)]
pub struct WallStepSettings {
    pub sweep: WallSweepSettings,
    pub contacts: ChassisContactSettings,
    pub friction: f64,
    pub allow_certified_departures: bool,
}
#[derive(Debug, Serialize)]
pub struct WallStep<'a> {
    pub integration: ChassisPartialStep,
    pub sweep: WallSweep<'a>,
    pub impact: Option<ChassisContactSample>,
    pub remaining_s: f64,
}
/// Merge only matching point/normal pairs; never discard an overflowing manifold.
pub fn chassis_wall_contacts(
    patches: &[WallContact<'_>],
    friction: f64,
) -> Result<Vec<ChassisContact>> {
    if !friction.is_finite() || !(0. ..=2.).contains(&friction) {
        return Err("WALL friction must lie in 0..2".into());
    }
    let mut contacts: Vec<ChassisContact> = Vec::new();
    for patch in patches {
        for &point in &patch.geometry.points_world_m {
            let normal = patch.geometry.normal_world;
            if contacts.iter().any(|c| {
                (DVec3::from_array(c.point_world_m) - DVec3::from_array(point)).length() <= 1e-6
                    && (DVec3::from_array(c.normal_world) - DVec3::from_array(normal)).length()
                        <= 1e-8
            }) {
                continue;
            }
            if contacts.len() == 64 {
                return Err(
                    "WALL manifold exceeds 64 distinct contact points; no truncation is applied"
                        .into(),
                );
            }
            contacts.push(ChassisContact {
                point_world_m: point,
                normal_world: normal,
                friction,
            });
        }
    }
    Ok(contacts)
}
/// Advance the retained midpoint path only to its first conservative WALL proximity event.
/// This commits both integration and impact atomically. The caller owns remaining-time/event loops.
/// Initial overlap is not repaired. A zero elapsed time is legitimate and must not be spun on.
pub fn step_to_wall<'a>(
    system: &mut ChassisSystem,
    dt_s: f64,
    collider: &ColliderBox,
    walls: &'a WallIndex,
    settings: WallStepSettings,
    racks: impl FnMut(f64) -> asc_physics::Result<Vec<RackMotion>>,
    forces: impl FnMut(f64, &ChassisKinematics) -> asc_physics::Result<ChassisLoads>,
) -> Result<WallStep<'a>> {
    // Validate friction even on a clear path.
    chassis_wall_contacts(&[], settings.friction)?;
    let mut candidate = system.clone();
    let mut sweep = None;
    let integration = candidate.step_with_racks_and_forces_until(dt_s, racks, forces, |path| {
        let query = || -> Result<_> {
            let motion = BoxMotion::new(
                path.start.position_m,
                path.end.position_m,
                path.start.orientation_xyzw,
                path.end.orientation_xyzw,
                collider.center_in_chassis_m,
                collider.half_extent_m,
            )?;
            if settings.allow_certified_departures {
                walls
                    .sweep_allowing_certified_departures(&motion, settings.sweep)
                    .map_err(Into::into)
            } else {
                walls.sweep(&motion, settings.sweep).map_err(Into::into)
            }
        };
        let result =
            query().map_err(|e| asc_physics::Error::new("session_wall_sweep", &e.to_string()))?;
        let fraction = result.fraction.unwrap_or(1.);
        sweep = Some(result);
        Ok(fraction)
    })?;
    let sweep = sweep.ok_or("WALL selector did not run")?;
    let impact = if sweep.fraction.is_some() {
        let contacts = chassis_wall_contacts(&sweep.contacts, settings.friction)?;
        Some(candidate.resolve_chassis_contacts(&contacts, settings.contacts)?)
    } else {
        None
    };
    let remaining_s = dt_s - integration.elapsed_s;
    *system = candidate;
    Ok(WallStep {
        integration,
        sweep,
        impact,
        remaining_s,
    })
}
