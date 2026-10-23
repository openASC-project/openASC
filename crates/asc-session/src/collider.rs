//! Enclosing box of supported collider.kn5 geometry in an explicitly mapped chassis frame.
use asc_vehicle::{Result, data::Mode};
use glam::{DQuat, DVec3};
use serde::Serialize;
use std::path::Path;
#[derive(Debug, Serialize)]
pub struct ColliderBox {
    pub car: String,
    pub data_source: &'static str,
    pub compound: u32,
    pub source: String,
    pub center_in_chassis_m: [f64; 3],
    pub half_extent_m: [f64; 3],
    pub indexed_vertex_references: usize,
    pub graphics_offset_m: [f64; 3],
    pub graphics_pitch_deg: f64,
    pub model_origin_in_chassis_m: [f64; 3],
    pub model_orientation_in_chassis_xyzw: [f64; 4],
    pub diagnostics: Vec<String>,
}
impl ColliderBox {
    pub fn load(
        car: &Path,
        mode: Mode,
        compound: Option<u32>,
        model_origin_in_chassis_m: [f64; 3],
        model_orientation_in_chassis_xyzw: [f64; 4],
    ) -> Result<Self> {
        let origin = DVec3::from_array(model_origin_in_chassis_m);
        let rotation = DQuat::from_array(model_orientation_in_chassis_xyzw);
        if !origin.is_finite()
            || origin.abs().max_element() > 20.
            || !rotation.is_finite()
            || (rotation.length_squared() - 1.).abs() > 1e-8
        {
            return Err("Collider model mapping requires finite translation within +/-20 m and a unit quaternion".into());
        }
        let rotation = rotation.normalize();
        let vehicle = asc_vehicle::load(car, mode, compound)?;
        let root = car.canonicalize()?;
        let path = asc_vehicle::data::confined(&root, "collider.kn5", "<native collider>", 0)?;
        if std::fs::metadata(&path)?.len() > 4 * 1024 * 1024 {
            return Err("collider.kn5 exceeds the 4 MiB proxy input limit".into());
        }
        let scene = asc_kn5::load(&path)?;
        if scene.summary.skinned_meshes > 0 {
            return Err("A skinned collider cannot define the static native box proxy".into());
        }
        let pitch = DQuat::from_rotation_x(vehicle.graphics_pitch_deg.to_radians());
        let graphics = DVec3::from_array(vehicle.graphics_offset_m);
        let mut low = DVec3::splat(f64::INFINITY);
        let mut high = DVec3::splat(f64::NEG_INFINITY);
        for &index in &scene.indices {
            let p = origin
                + rotation
                    * (graphics
                        + pitch
                            * DVec3::from_array(
                                scene.vertices[index as usize].position.map(f64::from),
                            ));
            if !p.is_finite() || p.abs().max_element() > 20. {
                return Err(
                    "Mapped collider geometry exceeds +/-20 m in chassis coordinates".into(),
                );
            }
            low = low.min(p);
            high = high.max(p);
        }
        let half = (high - low) * 0.5;
        if scene.indices.is_empty()
            || !half.is_finite()
            || half.min_element() < 0.01
            || half.max_element() > 10.
        {
            return Err("Collider proxy requires supported visible near-LOD geometry with three-dimensional half extents in 0.01..10 m".into());
        }
        let mut diagnostics = scene.summary.diagnostics;
        diagnostics.extend(
            vehicle
                .diagnostics
                .into_iter()
                .map(|d| format!("{}:{}: {}", d.source, d.line, d.message)),
        );
        diagnostics.push("Enclosing chassis-frame box of supported collider geometry; original convex shape, auxiliary colliders and collision semantics are not reproduced".into());
        Ok(Self {
            car: vehicle.car,
            data_source: vehicle.data_source,
            compound: vehicle.compound,
            source: "collider.kn5".into(),
            center_in_chassis_m: ((low + high) * 0.5).to_array(),
            half_extent_m: half.to_array(),
            indexed_vertex_references: scene.indices.len(),
            graphics_offset_m: vehicle.graphics_offset_m,
            graphics_pitch_deg: vehicle.graphics_pitch_deg,
            model_origin_in_chassis_m,
            model_orientation_in_chassis_xyzw: rotation.to_array(),
            diagnostics,
        })
    }
    pub fn at_pose(
        &self,
        position_world_m: [f64; 3],
        orientation_xyzw: [f64; 4],
    ) -> Result<asc_ground::OrientedBox> {
        let p = DVec3::from_array(position_world_m);
        let q = DQuat::from_array(orientation_xyzw);
        if !p.is_finite()
            || p.abs().max_element() > 1e7
            || !q.is_finite()
            || (q.length_squared() - 1.).abs() > 1e-8
        {
            return Err("Collider pose requires a bounded position and unit quaternion".into());
        }
        Ok(asc_ground::OrientedBox::new(
            (p + q.normalize() * DVec3::from_array(self.center_in_chassis_m)).to_array(),
            self.half_extent_m,
            q.to_array(),
        )?)
    }
}
