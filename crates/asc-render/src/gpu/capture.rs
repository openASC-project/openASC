//! Offscreen native wgpu diagnostic rendering; no original shader/material parity.
use super::{
    readback::Readback,
    scene::{self, BACKGROUND, Context, GpuScene, InstanceDescriptor, Result, View},
};
use crate::{resources::texture::Resources, scene::camera::camera};
use asc_kn5::Scene;
use serde::Serialize;
use std::{fs::OpenOptions, io::BufWriter, path::Path};

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
    pub compressed_textures: usize,
    pub texture_upload_bytes: usize,
    pub bc_enabled: bool,
    pub textured_draws: usize,
    pub multilayer_draws: usize,
    pub alpha_test_draws: usize,
    pub blended_draws: usize,
    pub diagnostics: Vec<String>,
    pub diffuse_enabled: bool,
    pub scope: &'static str,
    pub camera_focus: &'static str,
    pub root_instances: usize,
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
    let matrix = camera(
        scene.summary.road_focus_bounds.unwrap_or(scene_bounds),
        width,
        height,
    )?;
    render_with_view(
        scene,
        resources,
        &[],
        View {
            matrix,
            direction: [0.7, 1., 0.8],
            textures: true,
        },
        (width, height),
        fallback,
    )
    .await
}

pub async fn render_with_view(
    scene: &Scene,
    resources: Option<&Resources>,
    instances: &[InstanceDescriptor],
    view: View,
    size: (u32, u32),
    fallback: bool,
) -> Result<Frame> {
    let (width, height) = size;
    if width == 0
        || height == 0
        || width > 4096
        || height > 4096
        || view.matrix.iter().any(|v| !v.is_finite())
        || view.direction.iter().any(|v| !v.is_finite())
    {
        return Err("Invalid explicit capture view or dimensions.".into());
    }
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    let context = Context::request(
        &instance,
        None,
        fallback,
        resources.is_some_and(|r| r.allow_bc),
    )
    .await?;
    let gpu = GpuScene::with_instances(
        &context,
        scene,
        resources,
        wgpu::TextureFormat::Rgba8UnormSrgb,
        instances,
    )
    .await?;
    let target = context.device.create_texture(&wgpu::TextureDescriptor {
        label: Some("Offscreen colour"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let depth = scene::depth(&context.device, width, height);
    let readback = Readback::new(
        &context.device,
        width,
        height,
        wgpu::TextureFormat::Rgba8UnormSrgb,
    );
    let scope = context
        .device
        .push_error_scope(wgpu::ErrorFilter::Validation);
    let mut encoder = context.device.create_command_encoder(&Default::default());
    gpu.encode(
        &context,
        &mut encoder,
        &target.create_view(&Default::default()),
        &depth.create_view(&Default::default()),
        &view,
    );
    readback.encode(&mut encoder, &target);
    let rgba = readback.finish(&context.device, context.queue.submit([encoder.finish()]))?;
    if let Some(error) = scope.pop().await {
        return Err(error.into());
    }
    Ok(frame(
        &context,
        &gpu,
        rgba,
        width,
        height,
        (
            if resources.is_some() {
                "native_static_diffuse_preview"
            } else {
                "native_static_material_diagnostics"
            },
            view.textures,
        ),
        if !instances.is_empty() {
            "explicit_instance_view"
        } else if scene.summary.road_focus_bounds.is_some() {
            "road_bounds"
        } else {
            "scene_bounds"
        },
    ))
}

pub(super) fn frame(
    context: &Context,
    gpu: &GpuScene,
    rgba: Vec<u8>,
    width: u32,
    height: u32,
    scope: (&'static str, bool),
    focus: &'static str,
) -> Frame {
    let non_background_pixels = rgba
        .chunks_exact(4)
        .filter(|p| p.iter().zip(BACKGROUND).any(|(&v, b)| v.abs_diff(b) > 1))
        .count();
    let stats = &gpu.statistics;
    Frame {
        rgba,
        report: Report {
            adapter: context.info.name.clone(),
            backend: format!("{:?}", context.info.backend),
            device_type: format!("{:?}", context.info.device_type),
            width,
            height,
            non_background_pixels,
            background_rgba: BACKGROUND,
            triangles: stats.triangles,
            uploaded_textures: stats.uploaded_textures,
            compressed_textures: stats.compressed_textures,
            texture_upload_bytes: stats.texture_upload_bytes,
            bc_enabled: stats.bc_enabled,
            textured_draws: if scope.1 { stats.textured_draws } else { 0 },
            multilayer_draws: if scope.1 { stats.multilayer_draws } else { 0 },
            alpha_test_draws: stats.alpha_test_draws,
            blended_draws: stats.blended_draws,
            diagnostics: stats.diagnostics.clone(),
            diffuse_enabled: scope.1 && stats.uploaded_textures > 0,
            scope: scope.0,
            camera_focus: focus,
            root_instances: gpu.instance_count(),
        },
    }
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
