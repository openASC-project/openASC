//! Native SDR surface presentation with persistent geometry and optional readback.
use super::{
    capture::{self, Frame},
    readback::Readback,
    scene::{self, Context, GpuScene, Result, View},
};
use crate::resources::texture::Resources;
use asc_kn5::Scene;

pub enum Outcome {
    Presented(Option<Box<Frame>>),
    Reconfigured,
    Timeout,
    Occluded,
    Lost,
}
pub struct Presenter {
    pub instance: wgpu::Instance,
    pub context: Context,
    pub scene: GpuScene,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    view_format: wgpu::TextureFormat,
    depth: Option<wgpu::Texture>,
    active: bool,
    pub configurations: u32,
}
impl Presenter {
    pub async fn new(
        instance: wgpu::Instance,
        surface: wgpu::Surface<'static>,
        scene: &Scene,
        resources: Option<&Resources>,
        size: (u32, u32),
        fallback: bool,
    ) -> Result<Self> {
        let context = Context::request(&instance, Some(&surface), fallback).await?;
        let caps = surface.get_capabilities(&context.adapter);
        let format = caps
            .formats
            .iter()
            .copied()
            .find(|f| {
                matches!(
                    f,
                    wgpu::TextureFormat::Rgba8UnormSrgb | wgpu::TextureFormat::Bgra8UnormSrgb
                )
            })
            .or_else(|| {
                caps.formats.iter().copied().find(|f| {
                    matches!(
                        f,
                        wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Bgra8Unorm
                    )
                })
            })
            .ok_or("No SDR RGBA/BGRA surface format.")?;
        let view_format = format.add_srgb_suffix();
        let gpu = GpuScene::new(&context, scene, resources, view_format).await?;
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | if caps.usages.contains(wgpu::TextureUsages::COPY_SRC) {
                    wgpu::TextureUsages::COPY_SRC
                } else {
                    wgpu::TextureUsages::empty()
                },
            format,
            color_space: wgpu::SurfaceColorSpace::Auto,
            view_formats: vec![view_format],
            alpha_mode: if caps.alpha_modes.contains(&wgpu::CompositeAlphaMode::Opaque) {
                wgpu::CompositeAlphaMode::Opaque
            } else {
                wgpu::CompositeAlphaMode::Auto
            },
            width: size.0.max(1),
            height: size.1.max(1),
            desired_maximum_frame_latency: 2,
            present_mode: wgpu::PresentMode::AutoVsync,
        };
        let mut presenter = Self {
            instance,
            context,
            scene: gpu,
            surface,
            config,
            view_format,
            depth: None,
            active: false,
            configurations: 0,
        };
        presenter.resize(size.0, size.1)?;
        Ok(presenter)
    }
    pub fn size(&self) -> (u32, u32) {
        (self.config.width, self.config.height)
    }
    pub fn format(&self) -> wgpu::TextureFormat {
        self.view_format
    }
    pub fn resize(&mut self, width: u32, height: u32) -> Result<()> {
        self.active = false;
        self.depth = None;
        if width == 0 || height == 0 {
            return Ok(());
        }
        if width > 4096
            || height > 4096
            || width.max(height) > self.context.device.limits().max_texture_dimension_2d
        {
            return Err("Native viewport exceeds the 4096-pixel/device limit.".into());
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.context.device, &self.config);
        self.depth = Some(scene::depth(&self.context.device, width, height));
        self.active = true;
        self.configurations += 1;
        Ok(())
    }
    pub fn replace_surface(&mut self, surface: wgpu::Surface<'static>) -> Result<()> {
        let caps = surface.get_capabilities(&self.context.adapter);
        if !caps.formats.contains(&self.config.format) {
            return Err("Recovered surface changed format; restart the preview.".into());
        }
        self.surface = surface;
        self.resize(self.config.width, self.config.height)
    }
    pub async fn present(&mut self, view: &View, capture: bool) -> Result<Outcome> {
        if !self.active {
            return Ok(Outcome::Occluded);
        }
        if capture && !self.config.usage.contains(wgpu::TextureUsages::COPY_SRC) {
            return Err("This surface cannot be captured by GPU readback.".into());
        }
        let scope = self
            .context
            .device
            .push_error_scope(wgpu::ErrorFilter::Validation);
        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame) => frame,
            wgpu::CurrentSurfaceTexture::Suboptimal(frame) => {
                drop(frame);
                self.resize(self.config.width, self.config.height)?;
                return Ok(Outcome::Reconfigured);
            }
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.resize(self.config.width, self.config.height)?;
                return Ok(Outcome::Reconfigured);
            }
            wgpu::CurrentSurfaceTexture::Lost => return Ok(Outcome::Lost),
            wgpu::CurrentSurfaceTexture::Timeout => return Ok(Outcome::Timeout),
            wgpu::CurrentSurfaceTexture::Occluded => return Ok(Outcome::Occluded),
            wgpu::CurrentSurfaceTexture::Validation => {
                return Err("Surface acquisition failed validation.".into());
            }
        };
        let target = frame.texture.create_view(&wgpu::TextureViewDescriptor {
            format: Some(self.view_format),
            ..Default::default()
        });
        let mut encoder = self
            .context
            .device
            .create_command_encoder(&Default::default());
        self.scene.encode(
            &self.context,
            &mut encoder,
            &target,
            &self
                .depth
                .as_ref()
                .unwrap()
                .create_view(&Default::default()),
            view,
        );
        let readback = if capture {
            let r = Readback::new(
                &self.context.device,
                self.config.width,
                self.config.height,
                self.view_format,
            );
            r.encode(&mut encoder, &frame.texture);
            Some(r)
        } else {
            None
        };
        let submission = self.context.queue.submit([encoder.finish()]);
        let pixels = if let Some(readback) = readback {
            Some(readback.finish(&self.context.device, submission)?)
        } else {
            None
        };
        if let Some(error) = scope.pop().await {
            return Err(error.into());
        }
        self.context.queue.present(frame);
        Ok(Outcome::Presented(pixels.map(|rgba| {
            Box::new(capture::frame(
                &self.context,
                &self.scene,
                rgba,
                self.config.width,
                self.config.height,
                (
                    if view.textures {
                        "native_window_diffuse_preview"
                    } else {
                        "native_window_material_diagnostics"
                    },
                    view.textures,
                ),
                "interactive_orbit",
            ))
        })))
    }
}
