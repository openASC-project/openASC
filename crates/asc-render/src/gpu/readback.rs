//! Row-aligned RGBA/BGRA readback shared by captures and presented-frame checks.
use super::scene::Result;
use std::{sync::mpsc, time::Duration};
pub struct Readback {
    buffer: wgpu::Buffer,
    width: u32,
    height: u32,
    row_bytes: u32,
    bgra: bool,
}
impl Readback {
    pub fn new(
        device: &wgpu::Device,
        width: u32,
        height: u32,
        format: wgpu::TextureFormat,
    ) -> Self {
        let row_bytes = (width * 4).div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
            * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Padded frame readback"),
            size: row_bytes as u64 * height as u64,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        Self {
            buffer,
            width,
            height,
            row_bytes,
            bgra: matches!(
                format,
                wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
            ),
        }
    }
    pub fn encode(&self, encoder: &mut wgpu::CommandEncoder, texture: &wgpu::Texture) {
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &self.buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(self.row_bytes),
                    rows_per_image: Some(self.height),
                },
            },
            wgpu::Extent3d {
                width: self.width,
                height: self.height,
                depth_or_array_layers: 1,
            },
        );
    }
    pub fn finish(
        self,
        device: &wgpu::Device,
        submission: wgpu::SubmissionIndex,
    ) -> Result<Vec<u8>> {
        let (tx, rx) = mpsc::sync_channel(1);
        self.buffer
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |result| {
                let _ = tx.send(result);
            });
        device.poll(wgpu::PollType::Wait {
            submission_index: Some(submission),
            timeout: Some(Duration::from_secs(30)),
        })?;
        rx.recv_timeout(Duration::from_secs(1))??;
        let mapped = self.buffer.slice(..).get_mapped_range()?;
        let mut rgba = Vec::with_capacity((self.width * self.height * 4) as usize);
        for row in mapped.chunks_exact(self.row_bytes as usize) {
            rgba.extend_from_slice(&row[..(self.width * 4) as usize]);
        }
        drop(mapped);
        self.buffer.unmap();
        if self.bgra {
            for pixel in rgba.chunks_exact_mut(4) {
                pixel.swap(0, 2);
            }
        }
        Ok(rgba)
    }
}
