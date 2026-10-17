//! Offscreen native wgpu diagnostic rendering; no original shader/material parity.
use crate::{
    pipelines::material as materials, resources::texture::Resources, scene::camera::camera,
};
use asc_kn5::Scene;
use serde::Serialize;
use std::{error::Error, fs::OpenOptions, io::BufWriter, path::Path, sync::mpsc, time::Duration};
use wgpu::util::DeviceExt;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const BACKGROUND: [u8; 4] = [29, 35, 44, 255];

fn linear(byte: u8) -> f64 {
    let c = byte as f64 / 255.;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

#[derive(Serialize)]
pub struct Report {
    pub adapter: String,
    pub backend: String,
    pub device_type: String,
    pub width: u32,
    pub height: u32,
    pub non_background_pixels: usize,
    pub background_rgba: [u8; 4],
    pub triangles: usize,
    pub uploaded_textures: usize,
    pub textured_draws: usize,
    pub alpha_test_draws: usize,
    pub blended_draws: usize,
    pub diagnostics: Vec<String>,
    pub scope: &'static str,
    pub camera_focus: &'static str,
}
pub struct Frame {
    pub rgba: Vec<u8>,
    pub report: Report,
}

pub async fn render(scene: &Scene, width: u32, height: u32, fallback: bool) -> Result<Frame> {
    render_with_textures(scene, None, width, height, fallback).await
}

pub async fn render_with_textures(
    scene: &Scene,
    resources: Option<&Resources>,
    width: u32,
    height: u32,
    fallback: bool,
) -> Result<Frame> {
    let scene_bounds = scene
        .summary
        .bounds
        .ok_or("No visible indexed static geometry to render.")?;
    let bounds = scene.summary.road_focus_bounds.unwrap_or(scene_bounds);
    let matrix = camera(bounds, width, height)?;
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: fallback,
            compatible_surface: None,
            ..Default::default()
        })
        .await?;
    let info = adapter.get_info();
    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor {
            label: Some("openASC native diagnostic device"),
            ..Default::default()
        })
        .await?;
    let scope = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("Original diagnostic shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/diffuse.wgsl").into()),
    });
    let prepared = materials::prepare(&device, &queue, &shader, scene, resources)?;
    let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Camera"),
        contents: bytemuck::cast_slice(&matrix),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("Camera"),
        layout: &prepared.camera_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: camera_buffer.as_entire_binding(),
        }],
    });
    let order = materials::draw_order(scene, &prepared);
    let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Static world vertices"),
        contents: bytemuck::cast_slice(&scene.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Static triangle indices"),
        contents: bytemuck::cast_slice(&scene.indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    let extent = wgpu::Extent3d {
        width,
        height,
        depth_or_array_layers: 1,
    };
    let colour = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Offscreen colour"),
        size: extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Offscreen depth"),
        size: extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let row_bytes = (width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
        * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let readback = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("Padded RGBA readback"),
        size: row_bytes as u64 * height as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&Default::default());
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("Native scene capture"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &colour.create_view(&Default::default()),
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
                view: &depth.create_view(&Default::default()),
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
        pass.set_bind_group(0, &camera_bind, &[]);
        pass.set_vertex_buffer(0, vertices.slice(..));
        pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
        for index in &order {
            let draw = &scene.draws[*index];
            let material = &prepared.materials[draw.material];
            pass.set_pipeline(&prepared.pipelines[material.pipeline]);
            pass.set_bind_group(1, &material.bind, &[]);
            pass.draw_indexed(draw.start..draw.start + draw.count, 0, 0..1);
        }
    }
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &colour,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &readback,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row_bytes),
                rows_per_image: Some(height),
            },
        },
        extent,
    );
    let submission = queue.submit([encoder.finish()]);
    let (tx, rx) = mpsc::sync_channel(1);
    readback
        .slice(..)
        .map_async(wgpu::MapMode::Read, move |result| {
            let _ = tx.send(result);
        });
    device.poll(wgpu::PollType::Wait {
        submission_index: Some(submission),
        timeout: Some(Duration::from_secs(30)),
    })?;
    rx.recv_timeout(Duration::from_secs(1))??;
    if let Some(error) = scope.pop().await {
        return Err(error.into());
    }
    let mapped = readback.slice(..).get_mapped_range()?;
    let mut rgba = Vec::with_capacity((width * height * 4) as usize);
    for row in mapped.chunks_exact(row_bytes as usize) {
        rgba.extend_from_slice(&row[..(width * 4) as usize]);
    }
    drop(mapped);
    readback.unmap();
    // The corner may contain terrain when ROAD-focused. Compare with the known
    // clear colour, allowing one quantization unit across native backends.
    let non_background_pixels = rgba
        .chunks_exact(4)
        .filter(|p| p.iter().zip(BACKGROUND).any(|(&v, b)| v.abs_diff(b) > 1))
        .count();
    Ok(Frame {
        rgba,
        report: Report {
            adapter: info.name,
            backend: format!("{:?}", info.backend),
            device_type: format!("{:?}", info.device_type),
            width,
            height,
            non_background_pixels,
            background_rgba: BACKGROUND,
            triangles: scene.indices.len() / 3,
            uploaded_textures: prepared.uploaded,
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
            diagnostics: prepared.diagnostics,
            scope: if resources.is_some() {
                "native_static_diffuse_preview"
            } else {
                "native_static_material_diagnostics"
            },
            camera_focus: if scene.summary.road_focus_bounds.is_some() {
                "road_bounds"
            } else {
                "scene_bounds"
            },
        },
    })
}

pub fn save_png(frame: &Frame, path: &Path) -> Result<()> {
    // Create only after a successful GPU readback; never overwrite an existing file.
    let file = OpenOptions::new().write(true).create_new(true).open(path)?;
    let mut encoder = png::Encoder::new(
        BufWriter::new(file),
        frame.report.width,
        frame.report.height,
    );
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(&frame.rgba)?;
    writer.finish()?;
    Ok(())
}
