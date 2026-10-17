//! Offscreen native wgpu diagnostic rendering; no original shader/material parity.
use crate::kn5::{Scene, Vertex};
use glam::{DMat4, DVec3};
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
    pub scope: &'static str,
    pub camera_focus: &'static str,
}
pub struct Frame {
    pub rgba: Vec<u8>,
    pub report: Report,
}

pub fn camera(bounds: [[f32; 3]; 2], width: u32, height: u32) -> Result<[f32; 16]> {
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err("Image dimensions must be in 1..=4096.".into());
    }
    let lo = DVec3::from_array(bounds[0].map(f64::from));
    let hi = DVec3::from_array(bounds[1].map(f64::from));
    if !lo.is_finite() || !hi.is_finite() || (hi - lo).min_element() < 0. {
        return Err("Invalid scene bounds.".into());
    }
    let center = (lo + hi) * 0.5;
    let radius = ((hi - lo) * 0.5).length().max(0.1);
    let z = DVec3::new(0.7, 1., 0.8).normalize();
    let eye = center + z * radius * 3.;
    let right = DVec3::Y.cross(z).normalize();
    let up = z.cross(right);
    let view = DMat4::from_cols_array(&[
        right.x,
        up.x,
        z.x,
        0.,
        right.y,
        up.y,
        z.y,
        0.,
        right.z,
        up.z,
        z.z,
        0.,
        -right.dot(eye),
        -up.dot(eye),
        -z.dot(eye),
        1.,
    ]);
    let (mut x, mut y) = (0.01_f64, 0.01_f64);
    for i in 0..8 {
        let p = DVec3::new(
            if i & 1 == 0 { lo.x } else { hi.x },
            if i & 2 == 0 { lo.y } else { hi.y },
            if i & 4 == 0 { lo.z } else { hi.z },
        );
        let v = view.transform_point3(p);
        x = x.max(v.x.abs());
        y = y.max(v.y.abs());
    }
    let aspect = width as f64 / height as f64;
    let half_height = y.max(x / aspect) * 1.1;
    let half_width = half_height * aspect;
    // Right-handed orthographic projection with WebGPU depth in [0, 1].
    let near = radius;
    let far = radius * 5.;
    let projection = DMat4::from_cols_array(&[
        1. / half_width,
        0.,
        0.,
        0.,
        0.,
        1. / half_height,
        0.,
        0.,
        0.,
        0.,
        1. / (near - far),
        0.,
        0.,
        0.,
        near / (near - far),
        1.,
    ]);
    let matrix = (projection * view).to_cols_array().map(|v| v as f32);
    if matrix.iter().any(|v| !v.is_finite()) {
        return Err("Camera exceeds float32 range.".into());
    }
    Ok(matrix)
}

pub async fn render(scene: &Scene, width: u32, height: u32, fallback: bool) -> Result<Frame> {
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
        source: wgpu::ShaderSource::Wgsl(include_str!("diagnostic.wgsl").into()),
    });
    let attributes = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("Diagnostic static mesh pipeline"),
        layout: None,
        vertex: wgpu::VertexState {
            module: &shader,
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
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: Default::default(),
            bias: Default::default(),
        }),
        multisample: Default::default(),
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fragment"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    });
    let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("Camera"),
        contents: bytemuck::cast_slice(&matrix),
        usage: wgpu::BufferUsages::UNIFORM,
    });
    let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("Camera"),
        layout: &pipeline.get_bind_group_layout(0),
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
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(0, &camera_bind, &[]);
        pass.set_vertex_buffer(0, vertices.slice(..));
        pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..scene.indices.len() as u32, 0, 0..1);
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
            scope: "native_static_diagnostic_colours",
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
