//! Bounded, independent KN5 v5/v6 structural reader and static render geometry.
//! Texture payloads are skipped in place; no source assets are extracted.
use glam::{DMat4, DVec3};
use serde::Serialize;
use std::{
    collections::HashSet,
    fmt,
    fs::File,
    io::{BufReader, Read, Seek, SeekFrom},
    path::Path,
};

const MAX_GEOMETRY: usize = 256 * 1024 * 1024;

#[derive(Debug, Serialize)]
pub struct Error {
    pub code: &'static str,
    pub byte_offset: u64,
    pub message: String,
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} at byte {}: {}",
            self.code, self.byte_offset, self.message
        )
    }
}
impl std::error::Error for Error {}

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub colour: [f32; 3],
}

#[derive(Debug, Serialize)]
pub struct Summary {
    pub version: u32,
    pub nodes: usize,
    pub static_meshes: usize,
    pub skinned_meshes: usize,
    pub textures: usize,
    pub materials: usize,
    pub structural_triangles: usize,
    pub drawn_meshes: usize,
    pub drawn_triangles: usize,
    pub bounds: Option<[[f32; 3]; 2]>,
    pub road_focus_bounds: Option<[[f32; 3]; 2]>,
    pub diagnostics: Vec<String>,
}

pub struct Scene {
    pub summary: Summary,
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

struct Reader<R> {
    stream: R,
    offset: u64,
    size: u64,
    strings: usize,
    geometry: usize,
}
impl<R: Read + Seek> Reader<R> {
    fn new(mut stream: R) -> Result<Self, Error> {
        let size = stream.seek(SeekFrom::End(0)).map_err(|e| Error {
            code: "kn5_io",
            byte_offset: 0,
            message: e.to_string(),
        })?;
        let mut r = Self {
            stream,
            offset: 0,
            size,
            strings: 0,
            geometry: 0,
        };
        if size > 2 * 1024 * 1024 * 1024 {
            return r.fail("kn5_limit", "File exceeds 2 GiB.");
        }
        r.stream
            .rewind()
            .map_err(|e| r.error("kn5_io", &e.to_string()))?;
        Ok(r)
    }
    fn error(&self, code: &'static str, message: &str) -> Error {
        Error {
            code,
            byte_offset: self.offset,
            message: message.into(),
        }
    }
    fn fail<T>(&self, code: &'static str, message: &str) -> Result<T, Error> {
        Err(self.error(code, message))
    }
    fn require(&self, n: usize) -> Result<(), Error> {
        if n as u64 > self.size.saturating_sub(self.offset) {
            self.fail("kn5_truncated", "Field exceeds remaining input.")
        } else {
            Ok(())
        }
    }
    fn bytes<const N: usize>(&mut self) -> Result<[u8; N], Error> {
        self.require(N)?;
        let mut bytes = [0; N];
        self.stream
            .read_exact(&mut bytes)
            .map_err(|e| self.error("kn5_io", &e.to_string()))?;
        self.offset += N as u64;
        Ok(bytes)
    }
    fn skip(&mut self, n: usize) -> Result<(), Error> {
        self.require(n)?;
        self.stream
            .seek(SeekFrom::Current(n as i64))
            .map_err(|e| self.error("kn5_io", &e.to_string()))?;
        self.offset += n as u64;
        Ok(())
    }
    fn u32(&mut self) -> Result<u32, Error> {
        Ok(u32::from_le_bytes(self.bytes()?))
    }
    fn count(&mut self, max: usize) -> Result<usize, Error> {
        let n = self.u32()? as usize;
        if n > max {
            self.fail("kn5_limit", "Count exceeds its supported budget.")
        } else {
            Ok(n)
        }
    }
    fn flag(&mut self) -> Result<bool, Error> {
        match self.bytes::<1>()?[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => self.fail("kn5_flag", "Expected boolean 0 or 1."),
        }
    }
    fn string(&mut self) -> Result<String, Error> {
        let n = self.count(1_048_576)?;
        self.strings += n;
        if self.strings > 32 * 1024 * 1024 {
            return self.fail("kn5_limit", "Combined strings exceed 32 MiB.");
        }
        self.require(n)?;
        let mut bytes = vec![0; n];
        self.stream
            .read_exact(&mut bytes)
            .map_err(|e| self.error("kn5_io", &e.to_string()))?;
        self.offset += n as u64;
        String::from_utf8(bytes).map_err(|_| self.error("kn5_string", "Invalid UTF-8 string."))
    }
    fn floats<const N: usize>(&mut self) -> Result<[f32; N], Error> {
        let mut values = [0.; N];
        for v in &mut values {
            *v = f32::from_le_bytes(self.bytes()?);
            if !v.is_finite() {
                return self.fail("kn5_nonfinite", "NaN or infinity in numeric field.");
            }
        }
        Ok(values)
    }
    fn geometry(&mut self, n: usize) -> Result<(), Error> {
        self.geometry += n;
        if self.geometry > MAX_GEOMETRY {
            return self.fail("kn5_limit", "Geometry exceeds 256 MiB.");
        }
        self.require(n)
    }
}

pub fn load(path: &Path) -> Result<Scene, Error> {
    let file = File::open(path).map_err(|e| Error {
        code: "kn5_io",
        byte_offset: 0,
        message: e.to_string(),
    })?;
    parse(BufReader::with_capacity(256 * 1024, file))
}

pub fn parse<R: Read + Seek>(stream: R) -> Result<Scene, Error> {
    let mut r = Reader::new(stream)?;
    if &r.bytes::<6>()? != b"sc6969" {
        return r.fail("kn5_signature", "Unsupported/protected KN5 signature.");
    }
    let version = r.u32()?;
    if version != 5 && version != 6 {
        return r.fail("kn5_version", "Only KN5 v5/v6 are supported.");
    }
    let mut diagnostics = vec!["Diagnostic opaque material colours only: textures, original shaders, alpha/depth modes, shadows, skinning and distance LOD are not implemented.".into()];
    if version == 6 && r.u32()? != 0 {
        diagnostics.push("Nonzero v6 header field is unsupported.".into());
    }
    let textures = r.count(100_000)?;
    for _ in 0..textures {
        r.u32()?;
        r.string()?;
        let length = r.u32()? as usize;
        r.skip(length)?;
    }
    let materials = r.count(10_000)?;
    let mut colours = Vec::with_capacity(materials);
    let (mut properties, mut mappings) = (0, 0);
    for _ in 0..materials {
        let name = r.string()?;
        r.string()?;
        r.bytes::<1>()?;
        r.flag()?;
        r.u32()?;
        let count = r.count(10_000)?;
        properties += count;
        if properties > 100_000 {
            return r.fail("kn5_limit", "Combined shader properties exceed 100000.");
        }
        let mut names = HashSet::new();
        for _ in 0..count {
            if !names.insert(r.string()?) {
                return r.fail("kn5_property", "Duplicate shader property.");
            }
            r.floats::<10>()?;
        }
        let count = r.count(10_000)?;
        mappings += count;
        if mappings > 100_000 {
            return r.fail("kn5_limit", "Combined texture mappings exceed 100000.");
        }
        for _ in 0..count {
            r.string()?;
            r.u32()?;
            r.string()?;
        }
        // Original diagnostic palette, deliberately unrelated to source shader execution.
        let hash = name
            .bytes()
            .fold(0u32, |h, b| h.wrapping_mul(31).wrapping_add(b.into()));
        colours.push(
            [
                [0.34, 0.39, 0.43],
                [0.27, 0.38, 0.22],
                [0.48, 0.37, 0.25],
                [0.44, 0.28, 0.24],
                [0.25, 0.36, 0.42],
                [0.44, 0.43, 0.30],
            ][hash as usize % 6],
        );
    }
    let mut scene = Scene {
        summary: Summary {
            version,
            nodes: 0,
            static_meshes: 0,
            skinned_meshes: 0,
            textures,
            materials,
            structural_triangles: 0,
            drawn_meshes: 0,
            drawn_triangles: 0,
            bounds: None,
            road_focus_bounds: None,
            diagnostics,
        },
        vertices: Vec::new(),
        indices: Vec::new(),
    };
    node(&mut r, &mut scene, &colours, DMat4::IDENTITY, true, 0)?;
    if r.offset != r.size {
        return r.fail("kn5_trailing_data", "Unexplained trailing bytes.");
    }
    if scene.summary.skinned_meshes > 0 {
        scene
            .summary
            .diagnostics
            .push("Skinned meshes validated structurally but omitted from rendering.".into());
    }
    Ok(scene)
}

fn node<R: Read + Seek>(
    r: &mut Reader<R>,
    scene: &mut Scene,
    colours: &[[f32; 3]],
    parent: DMat4,
    ancestor_active: bool,
    depth: usize,
) -> Result<(), Error> {
    scene.summary.nodes += 1;
    if scene.summary.nodes > 200_000 || depth > 128 {
        return r.fail("kn5_limit", "Node count/depth exceeds budget.");
    }
    let kind = r.u32()?;
    let name = r.string()?;
    let children = r.count(100_000)?;
    let active = r.flag()? && ancestor_active;
    let mut world = parent;
    match kind {
        1 => {
            let local = r.floats::<16>()?;
            if [3, 7, 11].iter().any(|&i| local[i].abs() > 1e-5) || (local[15] - 1.).abs() > 1e-5 {
                return r.fail("kn5_transform", "Non-affine node transform.");
            }
            world = parent * DMat4::from_cols_array(&local.map(f64::from));
            if world
                .to_cols_array()
                .iter()
                .any(|v| !v.is_finite() || v.abs() > f32::MAX as f64)
            {
                return r.fail("kn5_transform", "Composed transform exceeds float32 range.");
            }
        }
        2 | 3 => {
            r.flag()?;
            let visible = r.flag()?;
            r.flag()?;
            if kind == 3 {
                scene.summary.skinned_meshes += 1;
                for _ in 0..r.count(100_000)? {
                    r.string()?;
                    r.floats::<16>()?;
                }
            } else {
                scene.summary.static_meshes += 1;
            }
            let count = r.count(2_000_000)?;
            let stride = if kind == 2 { 44 } else { 76 };
            r.geometry(count * stride)?;
            let mut positions = Vec::with_capacity(if kind == 2 { count } else { 0 });
            for _ in 0..count {
                let v = r.floats::<11>()?;
                if kind == 3 {
                    r.floats::<8>()?;
                } else {
                    let p =
                        world.transform_point3(DVec3::new(v[0].into(), v[1].into(), v[2].into()));
                    if !p.is_finite() || p.abs().max_element() > f32::MAX as f64 {
                        return r
                            .fail("kn5_transform", "Transformed vertex exceeds float32 range.");
                    }
                    positions.push(p.as_vec3().to_array());
                }
            }
            let index_count = r.count(12_000_000)?;
            if index_count % 3 != 0 {
                return r.fail(
                    "kn5_indices",
                    "Triangle index count is not divisible by three.",
                );
            }
            r.geometry(index_count * 2)?;
            scene.summary.structural_triangles += index_count / 3;
            let mut indices = Vec::with_capacity(if kind == 2 { index_count } else { 0 });
            for _ in 0..index_count {
                let index = u16::from_le_bytes(r.bytes()?) as usize;
                if index >= count {
                    return r.fail("kn5_indices", "Index outside vertex array.");
                }
                if kind == 2 {
                    indices.push(index as u32);
                }
            }
            let material = r.u32()? as usize;
            r.u32()?;
            let lod = r.floats::<2>()?;
            let renderable = if kind == 2 {
                r.floats::<4>()?;
                r.flag()?
            } else {
                false
            };
            if material >= colours.len() {
                return r.fail("kn5_material", "Material index outside table.");
            }
            if kind == 2
                && active
                && lod[0] <= 0.
                && !indices.is_empty()
                && name.to_ascii_uppercase().contains("ROAD")
            {
                for &position in &positions {
                    grow_bounds(&mut scene.summary.road_focus_bounds, position);
                }
            }
            if kind == 2 && active && visible && renderable && lod[0] <= 0. && !indices.is_empty() {
                if (scene.vertices.len() + count) * size_of::<Vertex>()
                    + (scene.indices.len() + indices.len()) * 4
                    > MAX_GEOMETRY
                {
                    return r.fail("kn5_limit", "Native render buffers exceed 256 MiB.");
                }
                let base = scene.vertices.len() as u32;
                for position in positions {
                    grow_bounds(&mut scene.summary.bounds, position);
                    scene.vertices.push(Vertex {
                        position,
                        colour: colours[material],
                    });
                }
                scene.summary.drawn_meshes += 1;
                scene.summary.drawn_triangles += indices.len() / 3;
                scene.indices.extend(indices.into_iter().map(|i| base + i));
            }
        }
        _ => {
            return r.fail(
                "kn5_node_type",
                "Unknown node type; refusing to guess its size.",
            );
        }
    }
    for _ in 0..children {
        node(r, scene, colours, world, active, depth + 1)?;
    }
    Ok(())
}

fn grow_bounds(bounds: &mut Option<[[f32; 3]; 2]>, position: [f32; 3]) {
    let bounds = bounds.get_or_insert([position, position]);
    for (i, &v) in position.iter().enumerate() {
        bounds[0][i] = bounds[0][i].min(v);
        bounds[1][i] = bounds[1][i].max(v);
    }
}
