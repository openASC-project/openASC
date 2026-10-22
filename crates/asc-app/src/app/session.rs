//! Native car/track scene assembly for static previews and explicit physical body poses.
use asc_kn5::Scene;
use asc_render::{
    gpu::scene::{InstanceDescriptor, Result},
    scene::pose::RigidPose,
};
use asc_scene::{Origin, Track};
use asc_vehicle::{Vehicle, data::Mode};
use glam::DQuat;
use std::{ops::Range, path::Path};

pub struct Preview {
    pub track: Track,
    pub instance: InstanceDescriptor,
    pub focus: [[f32; 3]; 2],
    pub metadata: serde_json::Value,
}
fn metadata_bytes(scene: &Scene) -> usize {
    scene.textures.iter().map(|t| t.name.len()).sum::<usize>()
        + scene
            .materials
            .iter()
            .map(|m| {
                m.name.len()
                    + m.shader.len()
                    + m.properties
                        .iter()
                        .map(|(k, v)| k.len() + size_of_val(v))
                        .sum::<usize>()
                    + m.mappings
                        .iter()
                        .map(|v| v.sampler.len() + v.texture.len())
                        .sum::<usize>()
                    + m.diffuse_refs.iter().map(String::len).sum::<usize>()
            })
            .sum::<usize>()
        + scene
            .summary
            .diagnostics
            .iter()
            .map(String::len)
            .sum::<usize>()
}
fn append(
    track: &mut Track,
    mut car: Scene,
    path: std::path::PathBuf,
    pose: RigidPose,
) -> Result<(Range<usize>, [[f32; 3]; 2])> {
    let scene = &mut track.scene;
    if (scene.vertices.len() + car.vertices.len()) * size_of::<asc_kn5::Vertex>()
        + (scene.indices.len() + car.indices.len()) * 4
        > 256 * 1024 * 1024
        || scene.summary.nodes + car.summary.nodes > 200_000
        || scene.materials.len() + car.materials.len() > 100_000
        || scene.textures.len() + car.textures.len() > 100_000
        || metadata_bytes(scene) + metadata_bytes(&car) > 32 * 1024 * 1024
        || track.manifest.models.len() >= 256
    {
        return Err(
            "Combined track/car exceeds existing scene geometry, metadata or count budgets.".into(),
        );
    }
    let bounds = pose.bounds(
        car.summary
            .bounds
            .ok_or("Car has no supported visible near-LOD geometry.")?,
    )?;
    let origin = Origin {
        section: "VEHICLE_0".into(),
        file: path
            .file_name()
            .ok_or("Car model has no filename.")?
            .to_string_lossy()
            .into_owned(),
        path,
        line: 0,
        position: [0.; 3],
        rotation: [0.; 3],
        version: car.summary.version,
        materials: scene.materials.len()..scene.materials.len() + car.materials.len(),
        textures: scene.textures.len()..scene.textures.len() + car.textures.len(),
    };
    let vertices = scene.vertices.len() as u32;
    let indices = scene.indices.len() as u32;
    let start = scene.draws.len();
    for draw in &mut car.draws {
        draw.start += indices;
        draw.material += origin.materials.start;
    }
    scene.summary.nodes += car.summary.nodes;
    scene.summary.static_meshes += car.summary.static_meshes;
    scene.summary.skinned_meshes += car.summary.skinned_meshes;
    scene.summary.structural_triangles += car.summary.structural_triangles;
    scene.summary.drawn_meshes += car.summary.drawn_meshes;
    scene.summary.drawn_triangles += car.summary.drawn_triangles;
    if let Some(world) = &mut scene.summary.bounds {
        for i in 0..3 {
            world[0][i] = world[0][i].min(bounds[0][i]);
            world[1][i] = world[1][i].max(bounds[1][i]);
        }
    } else {
        scene.summary.bounds = Some(bounds);
    }
    scene.summary.diagnostics.extend(
        car.summary
            .diagnostics
            .into_iter()
            .map(|v| format!("{}: {v}", origin.file)),
    );
    scene.vertices.extend(car.vertices);
    scene
        .indices
        .extend(car.indices.into_iter().map(|i| i + vertices));
    scene.draws.extend(car.draws);
    scene.materials.extend(car.materials);
    scene.textures.extend(car.textures);
    scene.summary.materials = scene.materials.len();
    scene.summary.textures = scene.textures.len();
    track.manifest.scope = "native_track_vehicle_instances";
    track.manifest.models.push(origin);
    Ok((start..scene.draws.len(), bounds))
}
pub fn load(
    car_dir: &Path,
    config: &Path,
    spawn_name: &str,
    mode: Mode,
    compound: Option<u32>,
) -> Result<Preview> {
    let vehicle: Vehicle = asc_vehicle::load(car_dir, mode, compound)?;
    let spawns = asc_scene::spawns::load(config)?;
    let spawn = spawns
        .spawns
        .iter()
        .find(|s| s.name == spawn_name)
        .ok_or("Selected preview spawn is absent, ambiguous or unsupported.")?;
    let root = RigidPose::from_basis(spawn.position, spawn.left, spawn.up, spawn.forward)?;
    let graphics = RigidPose::new(
        vehicle.graphics_offset_m,
        DQuat::from_rotation_x(vehicle.graphics_pitch_deg.to_radians()).to_array(),
    )?;
    let pose = root.compose(graphics)?;
    let car_root = car_dir.canonicalize()?;
    let path = asc_vehicle::data::confined(&car_root, &vehicle.model.file, "<native preview>", 0)?;
    let car = asc_kn5::load(&path)?;
    let mut track = asc_scene::load(config)?;
    let (draws, focus) = append(&mut track, car, path, pose)?;
    let metadata = serde_json::json!({"scope":"native_static_session_preview","car":vehicle.car,"spawn":spawn,"body_pose_policy":"source marker plane, not solved chassis CG","graphics_offset_m":vehicle.graphics_offset_m,"graphics_pitch_deg":vehicle.graphics_pitch_deg,"root_pose":pose,"car_draws":draws,"car_world_bounds":focus,"spawn_diagnostics":spawns.diagnostics,"vehicle_diagnostics":vehicle.diagnostics,"has_errors":spawns.has_errors,"simulation_ready":false,"driving":false,"limitations":"Static inspection pose. Chassis equilibrium, input-driven movement, wheel animation, original car materials/skins and physics are not implemented here."});
    Ok(Preview {
        track,
        instance: InstanceDescriptor { draws, pose },
        focus,
        metadata,
    })
}

/// Compose explicit physical COM-to-model mapping with retained source graphics correction.
pub(super) fn load_body(
    car_dir: &Path,
    config: &Path,
    mode: Mode,
    compound: Option<u32>,
    body: RigidPose,
    model_in_body: RigidPose,
) -> Result<(Preview, RigidPose, Vec<asc_kn5::NodeTransform>)> {
    let vehicle = asc_vehicle::load(car_dir, mode, compound)?;
    let graphics = RigidPose::new(
        vehicle.graphics_offset_m,
        DQuat::from_rotation_x(vehicle.graphics_pitch_deg.to_radians()).to_array(),
    )?;
    let correction = model_in_body.compose(graphics)?;
    let pose = body.compose(correction)?;
    let root = car_dir.canonicalize()?;
    let path = asc_vehicle::data::confined(&root, &vehicle.model.file, "<native drive>", 0)?;
    let (car, mut nodes) = asc_kn5::load_hierarchy(&path)?;
    let mut track = asc_scene::load(config)?;
    let (draws, focus) = append(&mut track, car, path, pose)?;
    for node in &mut nodes {
        node.draws.start += draws.start;
        node.draws.end += draws.start;
    }
    let metadata = serde_json::json!({"scope":"native_driving_visual_source","car":vehicle.car,"graphics_offset_m":vehicle.graphics_offset_m,
        "graphics_pitch_deg":vehicle.graphics_pitch_deg,"body_to_model":correction,"vehicle_diagnostics":vehicle.diagnostics,
        "original_vehicle_dynamics_validated":false,"wheel_mesh_animation":false,"material_parity":false});
    Ok((
        Preview {
            track,
            instance: InstanceDescriptor { draws, pose },
            focus,
            metadata,
        },
        correction,
        nodes,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use asc_test_support as f;
    use glam::Mat4;
    struct Directory(std::path::PathBuf);
    impl Drop for Directory {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    fn fixture() -> Directory {
        let root = f::temp_path("native-preview");
        let car = root.join("content/cars/fixture");
        let track = root.join("content/tracks/fixture");
        std::fs::create_dir_all(car.join("data")).unwrap();
        std::fs::create_dir_all(&track).unwrap();
        for (name, text) in f::vehicle_files() {
            std::fs::write(car.join("data").join(name), text).unwrap();
        }
        let mesh = f::mesh(
            &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
            &[0, 2, 1],
            2,
            1,
            1,
            0.,
            0,
        );
        std::fs::write(
            car.join("original.kn5"),
            f::file_texture(
                f::base(
                    std::slice::from_ref(&mesh),
                    Mat4::IDENTITY.to_cols_array(),
                    1,
                ),
                6,
                &f::bc1(0xf800),
            ),
        )
        .unwrap();
        let pose = [
            0., 0., -1., 0., 0., 1., 0., 0., 1., 0., 0., 0., 10., 2., 0., 1.,
        ];
        let marker = f::named(f::base(&[], pose, 1), "AC_PIT_0");
        std::fs::write(
            track.join("track.kn5"),
            f::file_texture(
                f::base(&[mesh, marker], Mat4::IDENTITY.to_cols_array(), 1),
                6,
                &f::bc1(0x001f),
            ),
        )
        .unwrap();
        std::fs::write(
            track.join("models.ini"),
            "[MODEL_0]\nFILE=track.kn5\nPOSITION=5,3,0\nROTATION=0,0,0\n",
        )
        .unwrap();
        Directory(root)
    }
    #[test]
    fn preview_rebases_geometry_and_keeps_source_texture_names_isolated_without_baking_car_pose() {
        let dir = fixture();
        let result = load(
            &dir.0.join("content/cars/fixture"),
            &dir.0.join("content/tracks/fixture/models.ini"),
            "AC_PIT_0",
            Mode::Auto,
            None,
        )
        .unwrap();
        assert_eq!(result.instance.draws, 1..2);
        assert_eq!(result.track.scene.indices, [0, 2, 1, 3, 5, 4]);
        assert_eq!(result.track.scene.vertices[0].position, [5., 3., 0.]);
        assert_eq!(result.track.scene.vertices[3].position, [0.; 3]);
        assert_eq!(result.track.scene.draws[1].material, 1);
        assert_eq!(result.track.manifest.models[1].materials, 1..2);
        assert_eq!(result.track.manifest.models[1].textures, 1..2);
        let p = result.instance.pose.point([0.; 3]).unwrap();
        for (a, b) in p.into_iter().zip([15.1, 4.7, 0.]) {
            assert!((a - b).abs() < 1e-5);
        }
        let resources = asc_render::load_track_textures(&result.track);
        assert_eq!(resources.material_textures, [Some(0), Some(1)]);
        assert_eq!(resources.textures.len(), 2);
        assert_eq!(&resources.textures[0].levels[0].rgba[..3], &[0, 0, 255]);
        assert_eq!(&resources.textures[1].levels[0].rgba[..3], &[255, 0, 0]);
        assert_eq!(result.metadata["driving"], false);
        assert_eq!(result.metadata["simulation_ready"], false);
    }
    #[test]
    fn preview_rejects_ambiguous_selection_and_source_model_escape() {
        let dir = fixture();
        let car = dir.0.join("content/cars/fixture");
        let config = dir.0.join("content/tracks/fixture/models.ini");
        assert!(load(&car, &config, "AC_PIT_99", Mode::Auto, None).is_err());
        std::fs::write(car.join("data/lods.ini"), "[LOD_0]\nFILE=../outside.kn5\n").unwrap();
        assert!(load(&car, &config, "AC_PIT_0", Mode::Auto, None).is_err());
    }
}
