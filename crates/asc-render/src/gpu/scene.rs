//! Persistent GPU scene shared by offscreen captures and native presentation.
use crate::{pipelines::material as materials, resources::texture::Resources};
use asc_kn5::{Draw, Scene};
use serde::Serialize;
use wgpu::util::DeviceExt;

pub const BACKGROUND: [u8; 4] = [29, 35, 44, 255];
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct Context {
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub info: wgpu::AdapterInfo,
}
impl Context {
    pub async fn request(
        instance: &wgpu::Instance,
        surface: Option<&wgpu::Surface<'_>>,
        fallback: bool,
        allow_bc: bool,
    ) -> Result<Self> {
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: fallback,
                compatible_surface: surface,
                ..Default::default()
            })
            .await?;
        let info = adapter.get_info();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("openASC native device"),
                required_features: if allow_bc {
                    adapter.features() & wgpu::Features::TEXTURE_COMPRESSION_BC
                } else {
                    wgpu::Features::empty()
                },
                ..Default::default()
            })
            .await?;
        Ok(Self {
            adapter,
            device,
            queue,
            info,
        })
    }
}

#[derive(Clone, Serialize)]
pub struct Statistics {
    pub triangles: usize,
    pub uploaded_textures: usize,
    pub compressed_textures: usize,
    pub texture_upload_bytes: usize,
    pub bc_enabled: bool,
    pub textured_draws: usize,
    pub alpha_test_draws: usize,
    pub blended_draws: usize,
    pub diagnostics: Vec<String>,
}
pub struct GpuScene {
    prepared: materials::Prepared,
    camera_buffer: wgpu::Buffer,
    camera_bind: wgpu::BindGroup,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    draws: Vec<Draw>,
    pub statistics: Statistics,
}
impl GpuScene {
    pub async fn new(
        context: &Context,
        scene: &Scene,
        resources: Option<&Resources>,
        format: wgpu::TextureFormat,
    ) -> Result<Self> {
        let device = &context.device;
        let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Original diffuse preview shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/diffuse.wgsl").into()),
        });
        let prepared =
            materials::prepare(device, &context.queue, &shader, scene, resources, format)?;
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Mutable camera"),
            contents: bytemuck::cast_slice(&[0_f32; 16]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Camera"),
            layout: &prepared.camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });
        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Static world vertices"),
            contents: bytemuck::cast_slice(&scene.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Static indices"),
            contents: bytemuck::cast_slice(&scene.indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let statistics = Statistics {
            triangles: scene.indices.len() / 3,
            uploaded_textures: prepared.uploaded,
            compressed_textures: prepared.compressed,
            texture_upload_bytes: prepared.upload_bytes,
            bc_enabled: context
                .device
                .features()
                .contains(wgpu::Features::TEXTURE_COMPRESSION_BC),
            textured_draws: scene
                .draws
                .iter()
                .filter(|d| prepared.materials[d.material].textured)
                .count(),
            alpha_test_draws: scene
                .draws
                .iter()
                .filter(|d| prepared.materials[d.material].cutout)
                .count(),
            blended_draws: scene
                .draws
                .iter()
                .filter(|d| prepared.materials[d.material].blend)
                .count(),
            diagnostics: prepared.diagnostics.clone(),
        };
        if let Some(error) = scope.pop().await {
            return Err(error.into());
        }
        Ok(Self {
            prepared,
            camera_buffer,
            camera_bind,
            vertices,
            indices,
            draws: scene.draws.clone(),
            statistics,
        })
    }
    pub fn encode(
        &self,
        context: &Context,
        encoder: &mut wgpu::CommandEncoder,
        target: &wgpu::TextureView,
        depth: &wgpu::TextureView,
        view: &View,
    ) {
        context
            .queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&view.matrix));
        let linear = |byte: u8| {
            let c = byte as f64 / 255.;
            if c <= 0.04045 {
                c / 12.92
            } else {
                ((c + 0.055) / 1.055).powf(2.4)
            }
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Shared native scene pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: linear(BACKGROUND[0]),
                        g: linear(BACKGROUND[1]),
                        b: linear(BACKGROUND[2]),
                        a: 1.,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_bind_group(0, &self.camera_bind, &[]);
        pass.set_vertex_buffer(0, self.vertices.slice(..));
        pass.set_index_buffer(self.indices.slice(..), wgpu::IndexFormat::Uint32);
        for index in materials::draw_order(&self.draws, &self.prepared, view.direction) {
            let draw = &self.draws[index];
            let material = &self.prepared.materials[draw.material];
            pass.set_pipeline(&self.prepared.pipelines[material.pipeline]);
            pass.set_bind_group(
                1,
                if view.textures {
                    &material.bind
                } else {
                    &material.diagnostic_bind
                },
                &[],
            );
            pass.draw_indexed(draw.start..draw.start + draw.count, 0, 0..1);
        }
    }
}
pub struct View {
    pub matrix: [f32; 16],
    pub direction: [f64; 3],
    pub textures: bool,
}

pub fn depth(device: &wgpu::Device, width: u32, height: u32) -> wgpu::Texture {
    device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Scene depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    })
}
