//! Approximate native diffuse/alpha/depth preview; not original AC shader execution.
use crate::resources::texture::Resources;
use asc_kn5::{Material, Scene, Vertex};
use asc_texture as dds;
use wgpu::util::DeviceExt;

pub struct GpuMaterial {
    pub bind: wgpu::BindGroup,
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

fn upload(device: &wgpu::Device, queue: &wgpu::Queue, texture: &dds::Texture) -> wgpu::TextureView {
    let base = &texture.levels[0];
    let gpu = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Decoded embedded diffuse"),
        size: wgpu::Extent3d {
            width: base.width,
            height: base.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: texture.levels.len() as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    for (mip, level) in texture.levels.iter().enumerate() {
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
    gpu.create_view(&Default::default())
}

pub fn prepare(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    shader: &wgpu::ShaderModule,
    scene: &Scene,
    resources: Option<&Resources>,
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
                            format: wgpu::TextureFormat::Rgba8UnormSrgb,
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
    let mut views = vec![upload(device, queue, &fallback)];
    let mut diagnostics = resources.map(|r| r.diagnostics.clone()).unwrap_or_default();
    let mut ids = Vec::new();
    if let Some(resources) = resources {
        if resources.material_textures.len() != scene.materials.len() {
            return Err("Material texture mapping length differs from scene.".into());
        }
        for texture in &resources.textures {
            if texture.levels.is_empty()
                || texture.info.width.max(texture.info.height)
                    > device.limits().max_texture_dimension_2d
            {
                diagnostics.push("Decoded texture exceeds device dimensions or has no mip levels; diagnostic colour.".into());
                ids.push(None);
            } else {
                ids.push(Some(views.len()));
                views.push(upload(device, queue, texture));
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
        let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Material parameters"),
            contents: bytemuck::cast_slice(&params),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
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
        });
        materials.push(GpuMaterial {
            bind,
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
        diagnostics,
    })
}

pub fn draw_order(scene: &Scene, prepared: &Prepared) -> Vec<usize> {
    let mut order: Vec<_> = (0..scene.draws.len()).collect();
    order.sort_by(|&a, &b| {
        let da = &scene.draws[a];
        let db = &scene.draws[b];
        let ma = &prepared.materials[da.material];
        let mb = &prepared.materials[db.material];
        ma.blend
            .cmp(&mb.blend)
            .then_with(|| {
                if ma.blend && mb.blend {
                    let depth = |p: [f32; 3]| p[0] as f64 * 0.7 + p[1] as f64 + p[2] as f64 * 0.8;
                    depth(da.center).total_cmp(&depth(db.center))
                } else {
                    (ma.pipeline / 2).cmp(&(mb.pipeline / 2))
                }
            })
            .then(a.cmp(&b))
    });
    order
}
