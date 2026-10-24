//! One fixed-pose, inelastic chassis/WALL velocity solve. No time advance or overlap repair.
use asc_physics::{ChassisContact, ChassisContactSettings};
use asc_vehicle::Result;
use glam::DVec3;
use serde::{Deserialize, Serialize};
use std::{io::Read, path::Path};
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    dynamics_config: String,
    model_origin_in_chassis_m: [f64; 3],
    model_orientation_in_chassis_xyzw: [f64; 4],
    compound: Option<u32>,
    margin_m: f64,
    friction: f64,
    #[serde(default)]
    settings: ChassisContactSettings,
}
fn manifold(patches: &[asc_ground::WallContact<'_>], friction: f64) -> Result<Vec<ChassisContact>> {
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
                    "wall-impact exceeds 64 distinct contact points; no truncation is applied"
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
pub fn run(path: &Path) -> Result<serde_json::Value> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(8193)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 8192 {
        return Err("wall-impact config exceeds 8192 bytes".into());
    }
    let c: Configuration = serde_json::from_slice(&bytes)?;
    if !c.friction.is_finite() || !(0. ..=2.).contains(&c.friction) {
        return Err("wall-impact requires explicit friction in 0..2".into());
    }
    let parent = path.parent().unwrap_or(Path::new("."));
    let mut bench = super::chassis_run::load(&parent.join(&c.dynamics_config))?;
    let (car, track, mode) = bench.source_paths()?;
    let collider = asc_session::ColliderBox::load(
        &car,
        mode,
        c.compound,
        c.model_origin_in_chassis_m,
        c.model_orientation_in_chassis_xyzw,
    )?;
    let walls = asc_ground::WallIndex::load(&track)?;
    let before = bench.system.kinematics()?;
    let body = collider.at_pose(before.chassis.position_m, before.chassis.orientation_xyzw)?;
    let patches = walls.contacts(&body, c.margin_m)?;
    let contacts = manifold(&patches, c.friction)?;
    let response = bench
        .system
        .resolve_chassis_contacts(&contacts, c.settings)?;
    let after = bench.system.kinematics()?;
    Ok(serde_json::json!({
        "scope":"fixed_pose_inelastic_chassis_wall_impact", "explicit_configuration":c,
        "body_shape":"enclosing_chassis_frame_box", "collider_proxy":collider,
        "walls":walls.summary(), "source_diagnostics":bench.diagnostics(), "patches":patches,
        "before":before, "after":after, "response":response,
        "time_advanced_s":0, "position_correction":false, "restitution":0,
        "continuous_collision_detection":false, "native_driving_collision_response":false,
        "original_collision_shape_validated":false, "original_vehicle_dynamics_validated":false,
        "simulation_ready":false
    }))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn patch_points_merge_only_matching_positions_and_normals_and_never_truncate() {
        let patch = |normal, points| asc_ground::WallContact {
            geometry: asc_ground::BoxTriangleContact {
                normal_world: normal,
                separation_m: -0.01,
                points_world_m: points,
            },
            triangle: 0,
            mesh: "1WALL",
            model: "original.kn5",
        };
        let a = patch([1., 0., 0.], vec![[0.; 3], [0., 1., 0.]]);
        let b = patch([1., 0., 0.], vec![[0., 1. + 1e-7, 0.]]);
        let c = patch([0., 0., 1.], vec![[0.; 3]]);
        let result = manifold(&[a, b, c], 0.3).unwrap();
        assert_eq!(result.len(), 3);
        assert!(result.iter().all(|c| c.friction == 0.3));
        let points = (0..64).map(|i| [0., i as f64, 0.]).collect();
        assert_eq!(
            manifold(&[patch([1., 0., 0.], points)], 0.).unwrap().len(),
            64
        );
        let points = (0..65).map(|i| [0., i as f64, 0.]).collect();
        assert!(manifold(&[patch([1., 0., 0.], points)], 0.).is_err());
    }
}
