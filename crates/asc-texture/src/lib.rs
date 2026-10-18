//! Original legacy DDS decoder: BC1/2/3 and masked RGB/luminance.
//! No source data is written to disk. All allocations follow validated budgets.
use serde::Serialize;
use std::fmt;

#[derive(Debug)]
pub struct Error {
    pub code: &'static str,
    pub message: &'static str,
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}
impl std::error::Error for Error {}
fn fail<T>(code: &'static str, message: &'static str) -> Result<T, Error> {
    Err(Error { code, message })
}

#[derive(Debug, Serialize)]
pub struct Info {
    pub width: u32,
    pub height: u32,
    pub format: &'static str,
    pub source_mips: u32,
    /// Required bytes for the complete RGBA mip chain, even when blocks stay encoded.
    pub decoded_bytes: usize,
    pub generated_mips: bool,
}
pub struct Level {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}
pub struct Texture {
    pub info: Info,
    pub levels: Vec<Level>,
    /// Validated legacy BC DDS retained for upload; mutually exclusive with RGBA levels.
    pub encoded: Option<Vec<u8>>,
}
pub struct BlockLevel<'a> {
    pub width: u32,
    pub height: u32,
    pub bytes: &'a [u8],
}
impl Texture {
    pub fn retained_bytes(&self) -> usize {
        self.encoded
            .as_ref()
            .map_or_else(|| self.levels.iter().map(|l| l.rgba.len()).sum(), Vec::len)
    }
    pub fn block_levels(&self) -> Result<Vec<BlockLevel<'_>>, Error> {
        let data = self.encoded.as_deref().ok_or(Error {
            code: "dds_storage",
            message: "Texture has no encoded BC data.",
        })?;
        let h = header(data)?;
        if !self.levels.is_empty()
            || !bc_aligned(&h.info)
            || h.info.width != self.info.width
            || h.info.height != self.info.height
            || h.info.format != self.info.format
            || h.info.source_mips != self.info.source_mips
        {
            return fail("dds_storage", "Inconsistent compressed texture storage.");
        }
        Ok(h.ranges
            .into_iter()
            .map(|(width, height, offset, length)| BlockLevel {
                width,
                height,
                bytes: &data[offset..offset + length],
            })
            .collect())
    }
}
fn bc_aligned(info: &Info) -> bool {
    matches!(info.format, "BC1" | "BC2" | "BC3")
        && info.width.is_multiple_of(4)
        && info.height.is_multiple_of(4)
}

/// Retain validated, block-aligned BC sources without expanding RGBA. Raw/unaligned
/// images use the software decoder; adapters lacking BC decode retained sources later.
pub fn prepare(data: &[u8], remaining_bytes: usize) -> Result<Texture, Error> {
    prepare_inner(data, remaining_bytes, false)
}
pub fn prepare_data(data: &[u8], remaining_bytes: usize) -> Result<Texture, Error> {
    prepare_inner(data, remaining_bytes, true)
}
fn prepare_inner(data: &[u8], remaining_bytes: usize, data_map: bool) -> Result<Texture, Error> {
    let h = header(data)?;
    if !bc_aligned(&h.info) {
        return decode_inner(data, remaining_bytes, data_map);
    }
    let &(_, _, offset, length) = h.ranges.last().unwrap();
    let end = offset + length;
    if end > remaining_bytes {
        return fail(
            "dds_limit",
            "Retained scene texture budget would be exceeded.",
        );
    }
    Ok(Texture {
        info: h.info,
        levels: Vec::new(),
        encoded: Some(data[..end].to_vec()),
    })
}
struct Header {
    info: Info,
    bits: u32,
    masks: [u32; 4],
    ranges: Vec<(u32, u32, usize, usize)>,
}

fn header(data: &[u8]) -> Result<Header, Error> {
    if data.len() < 128 || &data[..4] != b"DDS " {
        return fail("dds_header", "Missing legacy DDS header.");
    }
    let mut h = [0; 31];
    for (v, bytes) in h.iter_mut().zip(data[4..128].chunks_exact(4)) {
        *v = u32::from_le_bytes(bytes.try_into().unwrap());
    }
    if h[0] != 124 || h[18] != 32 {
        return fail("dds_header", "Invalid DDS structure sizes.");
    }
    let (width, height) = (h[3], h[2]);
    if width == 0
        || height == 0
        || width.max(height) > 8192
        || width as u64 * height as u64 > 16_777_216
    {
        return fail("dds_limit", "Dimensions exceed the bounded 2D subset.");
    }
    if h[5] > 1 || h[27] & (0xfe00 | 0x200000) != 0 {
        return fail("dds_surface", "Cube/volume textures are unsupported.");
    }
    let source_mips = h[6].max(1);
    let full_mips = 32 - width.max(height).leading_zeros();
    if source_mips > full_mips {
        return fail("dds_mips", "Mip count exceeds the 2D chain.");
    }
    let mut masks: [u32; 4] = h[22..26].try_into().unwrap();
    let bits = h[21];
    let (format, block) = if h[19] & 4 != 0 {
        match &h[20].to_le_bytes() {
            b"DXT1" => ("BC1", 8),
            b"DXT3" => ("BC2", 16),
            b"DXT5" => ("BC3", 16),
            _ => {
                return fail(
                    "dds_format",
                    "Unsupported FourCC, including DX10/BC4-7 and premultiplied DXT2/4.",
                );
            }
        }
    } else if h[19] & (0x40 | 0x20000) != 0 {
        let luminance = h[19] & 0x20000 != 0;
        if ![8, 16, 24, 32].contains(&bits) || (!luminance && bits == 8) {
            return fail("dds_format", "Unsupported raw bit count.");
        }
        if h[19] & 1 == 0 {
            masks[3] = 0;
        }
        let mut used = 0;
        for mask in masks {
            if mask != 0 {
                let normalized = mask >> mask.trailing_zeros();
                if normalized & normalized.wrapping_add(1) != 0
                    || (bits < 32 && mask as u64 >= (1u64 << bits))
                    || used & mask != 0
                {
                    return fail(
                        "dds_masks",
                        "Masks must be contiguous, in range and nonoverlapping.",
                    );
                }
                used |= mask;
            }
        }
        if masks[0] == 0
            || (!luminance && (masks[1] == 0 || masks[2] == 0))
            || (luminance && (masks[1] != 0 || masks[2] != 0))
            || (h[19] & 1 != 0 && masks[3] == 0)
        {
            return fail("dds_masks", "Missing or inconsistent channel masks.");
        }
        (if luminance { "LUMINANCE" } else { "RGB" }, 0)
    } else {
        return fail("dds_format", "Unsupported pixel format.");
    };
    let generated_mips = block == 0 && source_mips == 1 && full_mips > 1;
    let raw_ranges = |aligned: bool| -> Result<Vec<(u32, u32, usize, usize)>, Error> {
        let (mut w, mut ht, mut offset) = (width, height, 128);
        let mut ranges = Vec::new();
        for level in 0..source_mips {
            let tight = w as usize * (bits / 8) as usize;
            let pitch = if level == 0 && h[1] & 8 != 0 {
                h[4] as usize
            } else if aligned && level > 0 {
                tight.div_ceil(4) * 4
            } else {
                tight
            };
            if pitch < tight || pitch > tight + 65536 {
                return fail("dds_pitch", "Invalid/unsupported raw row pitch.");
            }
            ranges.push((w, ht, offset, pitch));
            offset += pitch * ht as usize;
            w = (w / 2).max(1);
            ht = (ht / 2).max(1);
        }
        Ok(ranges)
    };
    let mut ranges = Vec::new();
    let mut decoded_bytes = 0;
    if block == 0 {
        ranges = raw_ranges(false)?;
        let end = |ranges: &[(u32, u32, usize, usize)]| {
            let &(_, h, o, p) = ranges.last().unwrap();
            o + p * h as usize
        };
        if end(&ranges) > data.len() {
            return fail("dds_truncated", "Raw mip chain is incomplete.");
        }
        if source_mips > 1 && end(&ranges) != data.len() {
            let aligned = raw_ranges(true)?;
            if end(&aligned) != data.len() {
                return fail(
                    "dds_pitch",
                    "Raw lower mip layout is neither exact tight nor DWORD-aligned storage.",
                );
            }
            ranges = aligned;
        }
    } else {
        let (mut w, mut ht, mut offset) = (width, height, 128);
        for _ in 0..source_mips {
            let length = w.div_ceil(4) as usize * ht.div_ceil(4) as usize * block;
            if length > data.len().saturating_sub(offset) {
                return fail("dds_truncated", "Compressed mip chain is incomplete.");
            }
            ranges.push((w, ht, offset, length));
            offset += length;
            w = (w / 2).max(1);
            ht = (ht / 2).max(1);
        }
    }
    let (mut w, mut ht) = (width, height);
    for _ in 0..if generated_mips {
        full_mips
    } else {
        source_mips
    } {
        decoded_bytes += w as usize * ht as usize * 4;
        w = (w / 2).max(1);
        ht = (ht / 2).max(1);
    }
    Ok(Header {
        info: Info {
            width,
            height,
            format,
            source_mips,
            decoded_bytes,
            generated_mips,
        },
        bits,
        masks,
        ranges,
    })
}

pub fn inspect(data: &[u8]) -> Result<Info, Error> {
    Ok(header(data)?.info)
}

/// Budget is checked before allocating any decoded pixels.
pub fn decode(data: &[u8], remaining_bytes: usize) -> Result<Texture, Error> {
    decode_inner(data, remaining_bytes, false)
}
pub fn decode_data(data: &[u8], remaining_bytes: usize) -> Result<Texture, Error> {
    decode_inner(data, remaining_bytes, true)
}
fn decode_inner(data: &[u8], remaining_bytes: usize, data_map: bool) -> Result<Texture, Error> {
    let h = header(data)?;
    if h.info.decoded_bytes > remaining_bytes {
        return fail(
            "dds_limit",
            "Decoded scene texture budget would be exceeded.",
        );
    }
    let mut levels = Vec::new();
    if h.info.format == "RGB" || h.info.format == "LUMINANCE" {
        for &(width, height, offset, pitch) in &h.ranges {
            let stride = (h.bits / 8) as usize;
            let mut rgba = vec![0; width as usize * height as usize * 4];
            for y in 0..height as usize {
                for x in 0..width as usize {
                    let start = offset + y * pitch + x * stride;
                    let packed = data[start..start + stride]
                        .iter()
                        .enumerate()
                        .fold(0u32, |v, (i, &b)| v | (b as u32) << (8 * i));
                    let out = &mut rgba[(y * width as usize + x) * 4..][..4];
                    for (c, channel) in out.iter_mut().enumerate() {
                        let mask = if h.info.format == "LUMINANCE" && c < 3 {
                            h.masks[0]
                        } else {
                            h.masks[c]
                        };
                        *channel = if mask == 0 {
                            if c == 3 { 255 } else { 0 }
                        } else {
                            let shift = mask.trailing_zeros();
                            let max = (mask >> shift) as u64;
                            ((((packed & mask) >> shift) as u64 * 255 + max / 2) / max) as u8
                        };
                    }
                }
            }
            levels.push(Level {
                width,
                height,
                rgba,
            });
        }
        while h.info.generated_mips && levels.last().is_some_and(|l| l.width > 1 || l.height > 1) {
            levels.push(downsample(levels.last().unwrap(), data_map));
        }
    } else {
        for &(width, height, offset, length) in &h.ranges {
            let mut rgba = vec![0; width as usize * height as usize * 4];
            let stride = if h.info.format == "BC1" { 8 } else { 16 };
            let mut blocks = data[offset..offset + length].chunks_exact(stride);
            for by in 0..height.div_ceil(4) {
                for bx in 0..width.div_ceil(4) {
                    let block = blocks.next().unwrap();
                    let pixels = decode_block(block, h.info.format);
                    for (i, pixel) in pixels.iter().enumerate() {
                        let x = bx * 4 + i as u32 % 4;
                        let y = by * 4 + i as u32 / 4;
                        if x < width && y < height {
                            rgba[((y * width + x) * 4) as usize..][..4].copy_from_slice(pixel);
                        }
                    }
                }
            }
            levels.push(Level {
                width,
                height,
                rgba,
            });
        }
    }
    Ok(Texture {
        info: h.info,
        levels,
        encoded: None,
    })
}

fn rgb565(value: u16) -> [u8; 4] {
    let r = (value >> 11) as u8;
    let g = ((value >> 5) & 63) as u8;
    let b = (value & 31) as u8;
    [
        (r << 3) | (r >> 2),
        (g << 2) | (g >> 4),
        (b << 3) | (b >> 2),
        255,
    ]
}
fn decode_block(block: &[u8], format: &str) -> [[u8; 4]; 16] {
    let offset = if format == "BC1" { 0 } else { 8 };
    let c0 = u16::from_le_bytes(block[offset..offset + 2].try_into().unwrap());
    let c1 = u16::from_le_bytes(block[offset + 2..offset + 4].try_into().unwrap());
    let (a, b) = (rgb565(c0), rgb565(c1));
    let mut colours = [a, b, [0, 0, 0, 255], [0, 0, 0, 255]];
    for c in 0..3 {
        if format == "BC1" && c0 <= c1 {
            colours[2][c] = ((a[c] as u16 + b[c] as u16) / 2) as u8;
        } else {
            colours[2][c] = ((2 * a[c] as u16 + b[c] as u16) / 3) as u8;
            colours[3][c] = ((a[c] as u16 + 2 * b[c] as u16) / 3) as u8;
        }
    }
    if format == "BC1" && c0 <= c1 {
        colours[3] = [0; 4];
    }
    let colour_bits = u32::from_le_bytes(block[offset + 4..offset + 8].try_into().unwrap());
    let mut alpha = [0; 8];
    let mut alpha_bits = 0u64;
    if format == "BC3" {
        alpha[0] = block[0];
        alpha[1] = block[1];
        let steps = if alpha[0] > alpha[1] { 7 } else { 5 };
        for i in 1..steps {
            alpha[i + 1] = (((steps - i) as u16 * alpha[0] as u16 + i as u16 * alpha[1] as u16)
                / steps as u16) as u8;
        }
        if steps == 5 {
            alpha[6] = 0;
            alpha[7] = 255;
        }
        for i in 0..6 {
            alpha_bits |= (block[i + 2] as u64) << (i * 8);
        }
    }
    let mut pixels = [[0; 4]; 16];
    for (i, pixel) in pixels.iter_mut().enumerate() {
        *pixel = colours[((colour_bits >> (i * 2)) & 3) as usize];
        if format == "BC2" {
            pixel[3] = ((block[i / 2] >> ((i % 2) * 4)) & 15) * 17;
        }
        if format == "BC3" {
            pixel[3] = alpha[((alpha_bits >> (i * 3)) & 7) as usize];
        }
    }
    pixels
}

fn srgb_to_linear(v: u8) -> f64 {
    let c = v as f64 / 255.;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}
fn linear_to_srgb(v: f64) -> u8 {
    let c = if v <= 0.0031308 {
        v * 12.92
    } else {
        1.055 * v.powf(1. / 2.4) - 0.055
    };
    (c * 255.).round().clamp(0., 255.) as u8
}
fn downsample(source: &Level, data_map: bool) -> Level {
    let linear: [f64; 256] = std::array::from_fn(|i| srgb_to_linear(i as u8));
    let (width, height) = ((source.width / 2).max(1), (source.height / 2).max(1));
    let mut rgba = vec![0; (width * height * 4) as usize];
    // Area averaging includes all source texels for odd/one-dimensional sizes.
    for y in 0..height {
        for x in 0..width {
            let (x0, x1) = (x * source.width / width, (x + 1) * source.width / width);
            let (y0, y1) = (y * source.height / height, (y + 1) * source.height / height);
            let mut sum = [0.; 4];
            for sy in y0..y1 {
                for sx in x0..x1 {
                    let p = &source.rgba[((sy * source.width + sx) * 4) as usize..][..4];
                    for c in 0..3 {
                        sum[c] += if data_map {
                            p[c] as f64
                        } else {
                            linear[p[c] as usize]
                        };
                    }
                    sum[3] += p[3] as f64;
                }
            }
            let count = ((x1 - x0) * (y1 - y0)) as f64;
            let out = &mut rgba[((y * width + x) * 4) as usize..][..4];
            for c in 0..3 {
                out[c] = if data_map {
                    (sum[c] / count).round() as u8
                } else {
                    linear_to_srgb(sum[c] / count)
                };
            }
            out[3] = (sum[3] / count).round() as u8;
        }
    }
    Level {
        width,
        height,
        rgba,
    }
}
