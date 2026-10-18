use super::scene::Result;
use crate::scene::pose::RigidPose;
use asc_kn5::Scene;
use std::ops::Range;
use wgpu::util::DeviceExt;

#[derive(Clone)]
pub struct InstanceDescriptor {
    pub draws: Range<usize>,
    pub pose: RigidPose,
}
pub(super) struct Instance {
    pub pose: RigidPose,
    pub buffer: wgpu::Buffer,
    pub bind: wgpu::BindGroup,
    pub bounds: Option<[[f32; 3]; 2]>,
}
pub(super) fn create(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    scene: &Scene,
    descriptors: &[InstanceDescriptor],
) -> Result<(Vec<Instance>, Vec<usize>)> {
    if descriptors.len() > 255 {
        return Err("At most 256 root instances including the default root are supported.".into());
    }
    let mut map = vec![0; scene.draws.len()];
    for (i, d) in descriptors.iter().enumerate() {
        if d.draws.is_empty()
            || d.draws.end > map.len()
            || map[d.draws.clone()].iter().any(|&id| id != 0)
        {
            return Err(
                "Instance draw ranges must be nonempty, disjoint and inside the scene.".into(),
            );
        }
        map[d.draws.clone()].fill(i + 1);
    }
    let mut bounds = vec![None; descriptors.len() + 1];
    for (draw, &id) in scene.draws.iter().zip(&map) {
        let end = draw
            .start
            .checked_add(draw.count)
            .ok_or("Draw index range overflow.")? as usize;
        for &index in scene
            .indices
            .get(draw.start as usize..end)
            .ok_or("Draw index range exceeds geometry.")?
        {
            let p = scene
                .vertices
                .get(index as usize)
                .ok_or("Draw vertex index exceeds geometry.")?
                .position;
            if p.iter().any(|v| !v.is_finite()) {
                return Err("Instance geometry must be finite.".into());
            }
            let b = bounds[id].get_or_insert([p, p]);
            for i in 0..3 {
                b[0][i] = b[0][i].min(p[i]);
                b[1][i] = b[1][i].max(p[i]);
            }
        }
    }
    let mut result = Vec::new();
    for (id, bounds) in bounds.into_iter().enumerate() {
        let pose = if id == 0 {
            RigidPose::default()
        } else {
            descriptors[id - 1].pose
        };
        if let Some(b) = bounds {
            pose.bounds(b)?;
        }
        let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Persistent view/root uniform"),
            contents: bytemuck::cast_slice(&[0f32; 32]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Root camera"),
            layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            }],
        });
        result.push(Instance {
            pose,
            buffer,
            bind,
            bounds,
        });
    }
    Ok((result, map))
}
