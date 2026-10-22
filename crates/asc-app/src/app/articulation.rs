//! Explicit KN5 subtree-to-physics bindings. No car naming or axle-order inference.
use asc_kn5::NodeTransform;
use asc_physics::ChassisKinematics;
use asc_render::{
    gpu::scene::{InstanceDescriptor, Result},
    scene::pose::RigidPose,
};
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use std::ops::Range;

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Wheel,
    Upright,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Binding {
    pub node: String,
    pub upright: usize,
    pub kind: Kind,
}
pub struct Articulation {
    pub descriptors: Vec<InstanceDescriptor>,
    targets: Vec<Option<Target>>,
}
struct Target {
    binding: Binding,
    initial_local: RigidPose,
    pivot: DVec3,
}
fn physical(binding: &Binding, state: &ChassisKinematics) -> Result<RigidPose> {
    let upright = state
        .uprights
        .get(binding.upright)
        .ok_or("Articulation upright slot is absent")?;
    let wheel = &upright
        .wheel
        .as_ref()
        .ok_or("Articulation requires a physical wheel centre")?
        .body;
    let orientation = match binding.kind {
        Kind::Upright => upright.body.orientation_xyzw,
        Kind::Wheel => wheel.orientation_xyzw,
    };
    RigidPose::new(wheel.position_m, orientation)
}
impl Articulation {
    pub fn new(
        nodes: &[NodeTransform],
        car: Range<usize>,
        initial_model: RigidPose,
        state: &ChassisKinematics,
        bindings: &[Binding],
    ) -> Result<Self> {
        if bindings.len() > 16 {
            return Err("At most 16 articulation bindings are supported".into());
        }
        let mut ranges = Vec::new();
        let body_inverse =
            RigidPose::new(state.chassis.position_m, state.chassis.orientation_xyzw)?.inverse()?;
        let correction = body_inverse.compose(initial_model)?;
        for binding in bindings {
            if binding.node.is_empty() || binding.node.len() > 1024 {
                return Err("Articulation node name must contain 1..1024 bytes".into());
            }
            let mut matches = nodes.iter().filter(|n| n.name == binding.node);
            let node = matches.next().ok_or("Articulation source node is absent")?;
            if matches.next().is_some()
                || !node.active
                || node.draws.is_empty()
                || node.draws.start < car.start
                || node.draws.end > car.end
            {
                return Err(
                    "Articulation node must be unique, active and contain supported visible draws"
                        .into(),
                );
            }
            let initial_local = body_inverse.compose(physical(binding, state)?)?;
            let pivot = correction.compose(RigidPose::new(
                [node.world[12], node.world[13], node.world[14]],
                [0., 0., 0., 1.],
            )?)?;
            ranges.push((
                node.draws.clone(),
                Target {
                    binding: binding.clone(),
                    initial_local,
                    pivot: DVec3::from_array(pivot.position_m()),
                },
            ));
        }
        ranges.sort_by_key(|v| v.0.start);
        let mut result = Self {
            descriptors: Vec::new(),
            targets: Vec::new(),
        };
        let mut cursor = car.start;
        for (range, target) in ranges {
            if range.start < cursor {
                return Err("Articulation subtrees must not overlap or repeat".into());
            }
            if cursor < range.start {
                result.push(cursor..range.start, initial_model, None);
            }
            cursor = range.end;
            result.push(range, initial_model, Some(target));
        }
        if cursor < car.end {
            result.push(cursor..car.end, initial_model, None);
        }
        if result.descriptors.is_empty() {
            return Err("Articulation car draw range is empty".into());
        }
        Ok(result)
    }
    fn push(&mut self, draws: Range<usize>, pose: RigidPose, target: Option<Target>) {
        self.descriptors.push(InstanceDescriptor { draws, pose });
        self.targets.push(target);
    }
    pub fn metadata(&self) -> Vec<serde_json::Value> {
        self.descriptors.iter().zip(&self.targets).enumerate().map(|(i,(d,t))| {
            serde_json::json!({"root":i+1,"draws":d.draws,"binding":t.as_ref().map(|t|&t.binding),
                "source_pivot_in_chassis_m":t.as_ref().map(|t|t.pivot.to_array())})
        }).collect()
    }
    /// Apply chassis-relative physical travel and orientation changes about the source node pivot.
    /// Preserve the source rest placement even when the synthetic physical layout differs.
    pub fn poses(
        &self,
        state: &ChassisKinematics,
        correction: RigidPose,
    ) -> Result<Vec<RigidPose>> {
        let body = RigidPose::new(state.chassis.position_m, state.chassis.orientation_xyzw)?;
        let inverse = body.inverse()?;
        self.targets
            .iter()
            .map(|target| match target {
                None => body.compose(correction),
                Some(target) => {
                    let local = inverse.compose(physical(&target.binding, state)?)?;
                    let delta = DQuat::from_array(local.orientation_xyzw())
                        * DQuat::from_array(target.initial_local.orientation_xyzw()).conjugate();
                    let travel = DVec3::from_array(local.position_m())
                        - DVec3::from_array(target.initial_local.position_m());
                    let translation = target.pivot + travel - delta * target.pivot;
                    body.compose(RigidPose::new(translation.to_array(), delta.to_array())?)?
                        .compose(correction)
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use asc_physics::{
        RackMotion, State, TravelState, UprightKinematics, WheelKinematics, WheelState,
    };
    fn body(p: [f64; 3], q: DQuat) -> State {
        State {
            position_m: p,
            orientation_xyzw: q.to_array(),
            linear_momentum_world_kg_m_s: [0.; 3],
            angular_momentum_world_kg_m2_s: [0.; 3],
            velocity_world_m_s: [0.; 3],
            angular_velocity_world_rad_s: [0.; 3],
            kinetic_energy_j: 0.,
        }
    }
    fn state() -> ChassisKinematics {
        ChassisKinematics {
            chassis: body([0.; 3], DQuat::IDENTITY),
            uprights: vec![UprightKinematics {
                state: TravelState {
                    travel_m: 0.,
                    rate_m_s: 0.,
                },
                rack_motion: RackMotion::fixed([0.; 3]).unwrap(),
                source_position_m: [0.; 3],
                source_orientation_xyzw: DQuat::IDENTITY.to_array(),
                body: body([0.8, 0.3, 1.2], DQuat::IDENTITY),
                wheel: Some(WheelKinematics {
                    state: WheelState {
                        angle_rad: 0.,
                        rate_rad_s: 0.,
                    },
                    axis_world: [1., 0., 0.],
                    body: body([0.8, 0.3, 1.2], DQuat::IDENTITY),
                }),
            }],
        }
    }
    fn node(name: &str, draws: Range<usize>) -> NodeTransform {
        NodeTransform {
            name: name.into(),
            kind: 1,
            active: true,
            parent: None,
            world: glam::DMat4::from_translation(DVec3::new(0.7, 0.3, 1.3)).to_cols_array(),
            draws,
        }
    }
    fn binding(name: &str, kind: Kind) -> Binding {
        Binding {
            node: name.into(),
            upright: 0,
            kind,
        }
    }
    fn near(a: [f32; 3], b: [f32; 3]) {
        assert!(
            (glam::Vec3::from_array(a) - glam::Vec3::from_array(b)).length() < 1e-5,
            "{a:?} != {b:?}"
        );
    }
    #[test]
    fn source_pivot_stays_centered_under_spin_and_travel_with_rotated_moving_chassis() {
        let mut state = state();
        let initial_rotation = DQuat::from_rotation_z(0.3);
        state.uprights[0].body.orientation_xyzw = initial_rotation.to_array();
        state.uprights[0]
            .wheel
            .as_mut()
            .unwrap()
            .body
            .orientation_xyzw = initial_rotation.to_array();
        let correction =
            RigidPose::new([0., 0.2, 0.], DQuat::from_rotation_x(0.1).to_array()).unwrap();
        let nodes = [node("wheel", 3..5), node("suspension", 6..7)];
        let a = Articulation::new(
            &nodes,
            2..9,
            correction,
            &state,
            &[
                binding("wheel", Kind::Wheel),
                binding("suspension", Kind::Upright),
            ],
        )
        .unwrap();
        assert_eq!(
            a.descriptors
                .iter()
                .map(|d| d.draws.clone())
                .collect::<Vec<_>>(),
            [2..3, 3..5, 5..6, 6..7, 7..9]
        );
        let pivot = [0.7, 0.3, 1.3];
        let rest = a.poses(&state, correction).unwrap();
        for p in rest {
            near(p.point(pivot).unwrap(), correction.point(pivot).unwrap());
        }
        let rotate = DQuat::from_rotation_y(0.7);
        let shift = DVec3::new(10., 2., -5.);
        state.chassis = body(shift.to_array(), rotate);
        let local = DVec3::new(0.8, 0.34, 1.2);
        let steer = DQuat::from_rotation_y(0.12);
        // An offset upright COM must not translate the source wheel-centred suspension pivot.
        state.uprights[0].body = body(
            (shift + rotate * (local + DVec3::X * 0.02)).to_array(),
            rotate * steer * initial_rotation,
        );
        state.uprights[0].wheel.as_mut().unwrap().body = body(
            (shift + rotate * local).to_array(),
            rotate * steer * initial_rotation * DQuat::from_rotation_x(1.),
        );
        let poses = a.poses(&state, correction).unwrap();
        let pivot_local = DVec3::from_array(correction.point(pivot).unwrap().map(f64::from));
        let expected = (shift + rotate * (pivot_local + DVec3::Y * 0.04))
            .as_vec3()
            .to_array();
        near(poses[1].point(pivot).unwrap(), expected);
        near(poses[3].point(pivot).unwrap(), expected);
        let point = [0.7, 0.5, 1.3];
        let offset =
            DVec3::from_array(correction.point(point).unwrap().map(f64::from)) - pivot_local;
        near(
            poses[1].point(point).unwrap(),
            (DVec3::from_array(expected.map(f64::from))
                + rotate
                    * steer
                    * initial_rotation
                    * DQuat::from_rotation_x(1.)
                    * initial_rotation.conjugate()
                    * offset)
                .as_vec3()
                .to_array(),
        );
        near(
            poses[3].point(point).unwrap(),
            (DVec3::from_array(expected.map(f64::from)) + rotate * steer * offset)
                .as_vec3()
                .to_array(),
        );
    }
    #[test]
    fn invalid_or_ambiguous_bindings_fail_before_gpu_creation() {
        let mut s = state();
        let p = RigidPose::default();
        let nodes = [node("wheel", 2..4), node("child", 3..4)];
        let build = |ns: &[NodeTransform], bs: &[Binding]| Articulation::new(ns, 1..5, p, &s, bs);
        assert!(build(&nodes, &[binding("missing", Kind::Wheel)]).is_err());
        assert!(
            build(
                &nodes,
                &[binding("wheel", Kind::Wheel), binding("child", Kind::Wheel)]
            )
            .is_err()
        );
        assert!(
            build(
                &[node("wheel", 2..3), node("wheel", 3..4)],
                &[binding("wheel", Kind::Wheel)]
            )
            .is_err()
        );
        assert!(build(&[node("wheel", 3..3)], &[binding("wheel", Kind::Wheel)]).is_err());
        assert!(
            build(
                &nodes,
                &[Binding {
                    upright: 4,
                    ..binding("wheel", Kind::Wheel)
                }]
            )
            .is_err()
        );
        assert!(build(&nodes, &vec![binding("wheel", Kind::Wheel); 17]).is_err());
        let mut inactive = node("wheel", 2..3);
        inactive.active = false;
        assert!(build(&[inactive], &[binding("wheel", Kind::Wheel)]).is_err());
        s.uprights[0].wheel = None;
        assert!(Articulation::new(&nodes, 1..5, p, &s, &[binding("wheel", Kind::Wheel)]).is_err());
        assert!(
            serde_json::from_str::<Binding>(r#"{"node":"wheel","upright":0,"kind":"guess"}"#)
                .is_err()
        );
        assert!(
            serde_json::from_str::<Binding>(
                r#"{"node":"wheel","upright":0,"kind":"wheel","extra":0}"#
            )
            .is_err()
        );
    }
}
