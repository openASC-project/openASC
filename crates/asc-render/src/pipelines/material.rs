//! Approximate native diffuse/alpha/depth preview; not original AC shader execution.
use crate::resources::texture::Resources;
use asc_kn5::{Draw, Material, Scene, Vertex};
use asc_texture as dds;
use wgpu::util::DeviceExt;

pub struct GpuMaterial {
    pub bind: wgpu::BindGroup,
    pub diagnostic_bind: wgpu::BindGroup,
    pub pipeline: usize,
    pub textured: bool,
    pub cutout: bool,
    pub blend: bool,
}
pub struct Prepared {
    pub camera_layout: wgpu::BindGroupLayout,
    pub pipelines: Vec<wgpu::RenderPipeline>,
    pub materials: Vec<GpuMaterial>,
    pub uploaded: usize,
    pub compressed: usize,
    pub upload_bytes: usize,
    pub diagnostics: Vec<String>,
}

fn state(material: &Material) -> (bool, bool, u32) {
    if material.blend > 2 || material.depth > 2 {
        (false, false, 0)
    } else {
        (
            material.blend == 1,
            material.alpha_test || material.blend == 2,
            material.depth,
        )
    }
}

fn upload(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &dds::Texture,
) -> Result<(wgpu::TextureView, usize, bool), Box<dyn std::error::Error>> {
    let compressed = texture.encoded.is_some();
    let format = if compressed {
        match texture.info.format {
            "BC1" => wgpu::TextureFormat::Bc1RgbaUnormSrgb,
            "BC2" => wgpu::TextureFormat::Bc2RgbaUnormSrgb,
            "BC3" => wgpu::TextureFormat::Bc3RgbaUnormSrgb,
            _ => return Err("Unsupported compressed upload format.".into()),
        }
    } else {
        wgpu::TextureFormat::Rgba8UnormSrgb
    };
    let blocks = if compressed {
        texture.block_levels()?
    } else {
        Vec::new()
    };
    let gpu = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Embedded diffuse"),
        size: wgpu::Extent3d {
            width: texture.info.width,
            height: texture.info.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: if compressed {
            blocks.len()
        } else {
            texture.levels.len()
        } as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let mut bytes = 0;
    let stride = if texture.info.format == "BC1" { 8 } else { 16 };
    for (mip, level) in blocks.iter().enumerate() {
        bytes += level.bytes.len();
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &gpu,
                mip_level: mip as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            level.bytes,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(level.width.div_ceil(4) * stride),
                rows_per_image: Some(level.height.div_ceil(4)),
            },
            wgpu::Extent3d {
                width: level.width,
                height: level.height,
                depth_or_array_layers: 1,
            }
            .physical_size(format),
        );
    }
    for (mip, level) in texture.levels.iter().enumerate() {
        bytes += level.rgba.len();
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &gpu,
                mip_level: mip as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &level.rgba,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(level.width * 4),
                rows_per_image: Some(level.height),
            },
            wgpu::Extent3d {
                width: level.width,
                height: level.height,
                depth_or_array_layers: 1,
            },
        );
    }
    Ok((gpu.create_view(&Default::default()), bytes, compressed))
}

pub fn prepare(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    shader: &wgpu::ShaderModule,
    scene: &Scene,
    resources: Option<&Resources>,
    format: wgpu::TextureFormat,
) -> Result<Prepared, Box<dyn std::error::Error>> {
    let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("Camera layout"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    let material_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("Diffuse material layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ],
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("Native diffuse pipeline layout"),
        bind_group_layouts: &[Some(&camera_layout), Some(&material_layout)],
        ..Default::default()
    });
    let attributes =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2, 3 => Float32x3];
    let mut pipelines = Vec::new();
    for depth in 0..3 {
        for blend in [false, true] {
            pipelines.push(
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("Approximate diffuse material pipeline"),
                    layout: Some(&layout),
                    vertex: wgpu::VertexState {
                        module: shader,
                        entry_point: Some("vertex"),
                        compilation_options: Default::default(),
                        buffers: &[Some(wgpu::VertexBufferLayout {
                            array_stride: size_of::<Vertex>() as u64,
                            step_mode: wgpu::VertexStepMode::Vertex,
                            attributes: &attributes,
                        })],
                    },
                    primitive: wgpu::PrimitiveState {
                        cull_mode: None,
                        ..Default::default()
                    },
                    depth_stencil: Some(wgpu::DepthStencilState {
                        format: wgpu::TextureFormat::Depth32Float,
                        depth_write_enabled: Some(depth == 0),
                        depth_compare: Some(if depth == 2 {
                            wgpu::CompareFunction::Always
                        } else {
                            wgpu::CompareFunction::LessEqual
                        }),
                        stencil: Default::default(),
                        bias: if depth == 1 {
                            wgpu::DepthBiasState {
                                constant: -1,
                                slope_scale: -1.,
                                clamp: 0.,
                            }
                        } else {
                            Default::default()
                        },
                    }),
                    multisample: Default::default(),
                    fragment: Some(wgpu::FragmentState {
                        module: shader,
                        entry_point: Some("fragment"),
                        compilation_options: Default::default(),
                        targets: &[Some(wgpu::ColorTargetState {
                            format,
                            blend: if blend {
                                Some(wgpu::BlendState::ALPHA_BLENDING)
                            } else {
                                None
                            },
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    multiview_mask: None,
                    cache: None,
                }),
            );
        }
    }
    let fallback = dds::Texture {
        encoded: None,
        info: dds::Info {
            width: 1,
            height: 1,
            format: "RGB",
            source_mips: 1,
            decoded_bytes: 4,
            generated_mips: false,
        },
        levels: vec![dds::Level {
            width: 1,
            height: 1,
            rgba: vec![255; 4],
        }],
    };
    let mut views = vec![upload(device, queue, &fallback)?.0];
    let (mut upload_bytes, mut compressed) = (0, 0);
    let mut diagnostics = resources.map(|r| r.diagnostics.clone()).unwrap_or_default();
    let mut ids = Vec::new();
    if let Some(resources) = resources {
        if resources.material_textures.len() != scene.materials.len() {
            return Err("Material texture mapping length differs from scene.".into());
        }
        for (texture_index, texture) in resources.textures.iter().enumerate() {
            if (texture.levels.is_empty() && texture.encoded.is_none())
                || texture.info.width.max(texture.info.height)
                    > device.limits().max_texture_dimension_2d
            {
                diagnostics.push("Decoded texture exceeds device dimensions or has no mip levels; diagnostic colour.".into());
                ids.push(None);
            } else {
                let decoded;
                let remaining = 256 * 1024 * 1024 - upload_bytes;
                let texture = if let Some(data) = &texture.encoded {
                    if !device
                        .features()
                        .contains(wgpu::Features::TEXTURE_COMPRESSION_BC)
                    {
                        match dds::decode(data, remaining) {
                            Ok(value) => {
                                decoded = value;
                                &decoded
                            }
                            Err(error) => {
                                diagnostics.push(format!(
                                    "Texture resource {texture_index}, BC software fallback: {error} Diagnostic colour."
                                ));
                                ids.push(None);
                                continue;
                            }
                        }
                    } else {
                        texture
                    }
                } else {
                    texture
                };
                let cost = if texture.encoded.is_some() {
                    texture.block_levels()?.iter().map(|l| l.bytes.len()).sum()
                } else {
                    texture.retained_bytes()
                };
                if cost > remaining {
                    diagnostics.push("Texture upload payload exceeds the 256 MiB scene budget; diagnostic colour.".into());
                    ids.push(None);
                    continue;
                }
                let (view, bytes, bc) = upload(device, queue, texture)?;
                upload_bytes += bytes;
                compressed += usize::from(bc);
                ids.push(Some(views.len()));
                views.push(view);
            }
        }
    }
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("Repeat trilinear diffuse"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::Repeat,
        address_mode_w: wgpu::AddressMode::Repeat,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });
    let mut materials = Vec::new();
    for (index, material) in scene.materials.iter().enumerate() {
        let slot = if let Some(resources) = resources {
            match resources.material_textures[index] {
                Some(id) => *ids
                    .get(id)
                    .ok_or("Material references an unknown decoded texture.")?,
                None => None,
            }
        } else {
            None
        };
        let (blend, cutout, depth) = state(material);
        let params = [
            if slot.is_some() { 1_f32 } else { 0. },
            if cutout { 0.5 } else { 0. },
            if cutout || blend { 1. } else { 0. },
            material.opacity,
            material.diffuse,
            0.,
            0.,
            0.,
        ];
        let make_bind = |params: &[f32; 8]| {
            let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Material parameters"),
                contents: bytemuck::cast_slice(params),
                usage: wgpu::BufferUsages::UNIFORM,
            });
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Diffuse material"),
                layout: &material_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&views[slot.unwrap_or(0)]),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: buffer.as_entire_binding(),
                    },
                ],
            })
        };
        let bind = make_bind(&params);
        let mut diagnostic_params = params;
        diagnostic_params[0] = 0.;
        let diagnostic_bind = make_bind(&diagnostic_params);
        materials.push(GpuMaterial {
            bind,
            diagnostic_bind,
            pipeline: depth as usize * 2 + usize::from(blend),
            textured: slot.is_some(),
            cutout,
            blend,
        });
    }
    Ok(Prepared {
        camera_layout,
        pipelines,
        materials,
        uploaded: views.len() - 1,
        compressed,
        upload_bytes,
        diagnostics,
    })
}

pub fn draw_order(draws: &[Draw], prepared: &Prepared, direction: [f64; 3]) -> Vec<usize> {
    order_with(
        draws,
        |i| {
            let m = &prepared.materials[i];
            (m.blend, m.pipeline / 2)
        },
        direction,
    )
}

fn order_with(
    draws: &[Draw],
    state: impl Fn(usize) -> (bool, usize),
    direction: [f64; 3],
) -> Vec<usize> {
    let mut order: Vec<_> = (0..draws.len()).collect();
    order.sort_by(|&a, &b| {
        let da = &draws[a];
        let db = &draws[b];
        let ma = state(da.material);
        let mb = state(db.material);
        ma.0.cmp(&mb.0)
            .then_with(|| {
                if ma.0 && mb.0 {
                    let depth = |p: [f32; 3]| {
                        p[0] as f64 * direction[0]
                            + p[1] as f64 * direction[1]
                            + p[2] as f64 * direction[2]
                    };
                    depth(da.center).total_cmp(&depth(db.center))
                } else {
                    ma.1.cmp(&mb.1)
                }
            })
            .then(a.cmp(&b))
    });
    order
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transparency_order_tracks_camera_direction_and_retains_opaque_depth_groups() {
        let mut draws = vec![
            Draw {
                start: 0,
                count: 3,
                material: 1,
                center: [-2., 0., 0.],
            },
            Draw {
                start: 3,
                count: 3,
                material: 1,
                center: [2., 0., 0.],
            },
            Draw {
                start: 6,
                count: 3,
                material: 0,
                center: [0.; 3],
            },
            Draw {
                start: 9,
                count: 3,
                material: 2,
                center: [0.; 3],
            },
        ];
        let states = [(false, 0), (true, 0), (false, 1)];
        assert_eq!(
            order_with(&draws, |i| states[i], [1., 0., 0.]),
            [2, 3, 0, 1]
        );
        assert_eq!(
            order_with(&draws, |i| states[i], [-1., 0., 0.]),
            [2, 3, 1, 0]
        );
        draws[1].center = draws[0].center;
        assert_eq!(
            order_with(&draws, |i| states[i], [-1., 0., 0.]),
            [2, 3, 0, 1]
        );
    }
}
