//! Bounded, independent KN5 v5/v6 structural reader and static render geometry.
//! Texture payloads are skipped in place; no source assets are extracted.
use glam::{DMat3, DMat4, DVec3};
use serde::Serialize;
use std::{
    collections::{BTreeMap, HashSet},
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
    pub normal: [f32; 3],
    pub uv: [f32; 2],
    pub colour: [f32; 3],
}

#[derive(Debug, Serialize)]
pub struct Embedded {
    pub name: String,
    pub active: bool,
    pub offset: u64,
    pub length: usize,
}
#[derive(Clone, Debug, Serialize)]
pub struct Mapping {
    pub sampler: String,
    pub slot: u32,
    pub texture: String,
}
#[derive(Clone, Debug, Serialize)]
pub struct Material {
    pub name: String,
    pub shader: String,
    pub blend: u8,
    pub alpha_test: bool,
    pub depth: u32,
    pub opacity: f32,
    pub diffuse: f32,
    pub diffuse_refs: Vec<String>,
    pub properties: BTreeMap<String, [f32; 10]>,
    pub mappings: Vec<Mapping>,
}
#[derive(Clone, Debug)]
pub struct Draw {
    pub start: u32,
    pub count: u32,
    pub material: usize,
    pub center: [f32; 3],
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
    pub materials: Vec<Material>,
    pub textures: Vec<Embedded>,
    pub draws: Vec<Draw>,
}

struct Reader<R> {
    stream: R,
    offset: u64,
    size: u64,
    strings: usize,
    geometry: usize,
    physical_geometry: usize,
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
            physical_geometry: 0,
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

/// Active static meshes with a nonzero numeric physical prefix, independent of render flags.
pub struct PhysicalMesh {
    pub name: String,
    pub lod: [f32; 2],
    pub positions: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}
pub struct Collision {
    pub meshes: Vec<PhysicalMesh>,
    pub summary: Summary,
}
pub fn load_collision(path: &Path) -> Result<Collision, Error> {
    let file = File::open(path).map_err(|e| Error {
        code: "kn5_io",
        byte_offset: 0,
        message: e.to_string(),
    })?;
    parse_collision(BufReader::with_capacity(256 * 1024, file))
}
pub fn parse_collision<R: Read + Seek>(stream: R) -> Result<Collision, Error> {
    let (scene, traversal) = parse_inner(stream, ReadMode::Collision)?;
    Ok(Collision {
        meshes: traversal.physical.unwrap_or_default(),
        summary: scene.summary,
    })
}
pub fn parse<R: Read + Seek>(stream: R) -> Result<Scene, Error> {
    Ok(parse_inner(stream, ReadMode::Render)?.0)
}
#[derive(Clone, Debug, Serialize)]
pub struct NodeTransform {
    pub name: String,
    pub kind: u32,
    pub active: bool,
    pub parent: Option<usize>,
    pub world: [f64; 16],
}
pub struct NodeScene {
    pub nodes: Vec<NodeTransform>,
    pub summary: Summary,
}
/// Full structural validation and node transforms, without retaining mesh buffers.
pub fn load_nodes(path: &Path) -> Result<NodeScene, Error> {
    if !std::fs::metadata(path)
        .map_err(|e| Error {
            code: "kn5_io",
            byte_offset: 0,
            message: e.to_string(),
        })?
        .is_file()
    {
        return Err(Error {
            code: "kn5_io",
            byte_offset: 0,
            message: "Expected a regular KN5 file.".into(),
        });
    }
    let file = File::open(path).map_err(|e| Error {
        code: "kn5_io",
        byte_offset: 0,
        message: e.to_string(),
    })?;
    parse_nodes(BufReader::with_capacity(256 * 1024, file))
}
pub fn parse_nodes<R: Read + Seek>(stream: R) -> Result<NodeScene, Error> {
    let (scene, traversal) = parse_inner(stream, ReadMode::Nodes)?;
    Ok(NodeScene {
        nodes: traversal.nodes.unwrap_or_default(),
        summary: scene.summary,
    })
}
enum ReadMode {
    Render,
    Collision,
    Nodes,
}
struct Traversal {
    render_geometry: bool,
    physical: Option<Vec<PhysicalMesh>>,
    nodes: Option<Vec<NodeTransform>>,
    parent_index: Option<usize>,
}
fn parse_inner<R: Read + Seek>(stream: R, mode: ReadMode) -> Result<(Scene, Traversal), Error> {
    let mut r = Reader::new(stream)?;
    if &r.bytes::<6>()? != b"sc6969" {
        return r.fail("kn5_signature", "Unsupported/protected KN5 signature.");
    }
    let version = r.u32()?;
    if version != 5 && version != 6 {
        return r.fail("kn5_version", "Only KN5 v5/v6 are supported.");
    }
    let mut diagnostics = vec!["Structural inspection indexes embedded textures without decoding. Native rendering loads supported diffuse resources and reconstructs a limited multilayer base colour with approximate lighting/alpha/depth; original normal/Fresnel/specular lighting, shadows, skinning and distance LOD remain incomplete.".into()];
    if version == 6 && r.u32()? != 0 {
        diagnostics.push("Nonzero v6 header field is unsupported.".into());
    }
    let textures = r.count(100_000)?;
    let mut embedded = Vec::with_capacity(textures);
    for _ in 0..textures {
        let active = r.u32()? == 1;
        let name = r.string()?;
        let length = r.u32()? as usize;
        embedded.push(Embedded {
            name,
            active,
            offset: r.offset,
            length,
        });
        r.skip(length)?;
    }
    let materials = r.count(10_000)?;
    let mut colours = Vec::with_capacity(materials);
    let mut material_records = Vec::with_capacity(materials);
    let (mut properties, mut mappings) = (0, 0);
    for _ in 0..materials {
        let name = r.string()?;
        let shader = r.string()?;
        let blend = r.bytes::<1>()?[0];
        let alpha_test = r.flag()?;
        let depth = r.u32()?;
        let (mut opacity, mut diffuse) = (1_f32, 0.7_f32);
        let count = r.count(10_000)?;
        properties += count;
        if properties > 100_000 {
            return r.fail("kn5_limit", "Combined shader properties exceed 100000.");
        }
        let mut names = HashSet::new();
        let mut property_records = BTreeMap::new();
        for _ in 0..count {
            let property = r.string()?;
            if !names.insert(property.clone()) {
                return r.fail("kn5_property", "Duplicate shader property.");
            }
            let values = r.floats::<10>()?;
            if property == "alpha" {
                opacity = values[0].clamp(0., 1.);
            }
            if property == "ksDiffuse" {
                diffuse = values[0].clamp(0., 2.);
            }
            property_records.insert(property, values);
        }
        let count = r.count(10_000)?;
        mappings += count;
        if mappings > 100_000 {
            return r.fail("kn5_limit", "Combined texture mappings exceed 100000.");
        }
        let mut diffuse_refs = Vec::new();
        let mut texture_mappings = Vec::new();
        for _ in 0..count {
            let sampler = r.string()?;
            let slot = r.u32()?;
            let texture = r.string()?;
            if sampler == "txDiffuse" {
                diffuse_refs.push(texture.clone());
            }
            texture_mappings.push(Mapping {
                sampler,
                slot,
                texture,
            });
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
        if blend > 2 || depth > 2 {
            diagnostics.push(format!(
                "Material {name}: unsupported blend/depth state uses opaque depth-writing fallback."
            ));
        }
        material_records.push(Material {
            name,
            shader,
            blend,
            alpha_test,
            depth,
            opacity,
            diffuse,
            diffuse_refs,
            properties: property_records,
            mappings: texture_mappings,
        });
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
        materials: material_records,
        textures: embedded,
        draws: Vec::new(),
    };
    let mut traversal = Traversal {
        render_geometry: matches!(mode, ReadMode::Render),
        physical: matches!(mode, ReadMode::Collision).then(Vec::new),
        nodes: matches!(mode, ReadMode::Nodes).then(Vec::new),
        parent_index: None,
    };
    node(
        &mut r,
        &mut scene,
        &colours,
        DMat4::IDENTITY,
        true,
        0,
        &mut traversal,
    )?;
    if r.offset != r.size {
        return r.fail("kn5_trailing_data", "Unexplained trailing bytes.");
    }
    if scene.summary.skinned_meshes > 0 {
        scene
            .summary
            .diagnostics
            .push("Skinned meshes validated structurally but omitted from rendering.".into());
    }
    Ok((scene, traversal))
}

fn node<R: Read + Seek>(
    r: &mut Reader<R>,
    scene: &mut Scene,
    colours: &[[f32; 3]],
    parent: DMat4,
    ancestor_active: bool,
    depth: usize,
    traversal: &mut Traversal,
) -> Result<(), Error> {
    let parent_index = traversal.parent_index;
    let node_index = scene.summary.nodes;
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
            let prefix: String = name.chars().take_while(char::is_ascii_digit).collect();
            let capture = traversal.physical.is_some()
                && kind == 2
                && active
                && prefix.bytes().any(|c| c != b'0');
            let mut collision_positions = Vec::new();
            let mut positions = Vec::with_capacity(if kind == 2 && traversal.render_geometry {
                count
            } else {
                0
            });
            let mut attributes = Vec::with_capacity(if kind == 2 && traversal.render_geometry {
                count
            } else {
                0
            });
            let linear = DMat3::from_cols(
                world.x_axis.truncate(),
                world.y_axis.truncate(),
                world.z_axis.truncate(),
            );
            let determinant = linear.determinant();
            let invertible = determinant.is_finite() && determinant != 0.;
            let normal_matrix = if invertible {
                linear.inverse().transpose()
            } else {
                DMat3::ZERO
            };
            let normals_valid = invertible && normal_matrix.is_finite();
            if kind == 2 && !normals_valid {
                scene.summary.diagnostics.push(format!(
                    "Mesh {name}: singular normal transform uses geometric lighting."
                ));
            }
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
                    if capture {
                        if p.abs().max_element() > 10_000_000. {
                            return r.fail(
                                "ground_range",
                                "Physical coordinates exceed +/-10000000 metres.",
                            );
                        }
                        collision_positions.push(p.to_array());
                    }
                    if traversal.render_geometry {
                        positions.push(p.as_vec3().to_array());
                    }
                    let normal = if normals_valid {
                        (normal_matrix * DVec3::new(v[3].into(), v[4].into(), v[5].into()))
                            .normalize_or_zero()
                    } else {
                        DVec3::ZERO
                    };
                    if traversal.render_geometry {
                        attributes.push((normal.as_vec3().to_array(), [v[6], v[7]]));
                    }
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
            let mut indices =
                Vec::with_capacity(if kind == 2 && (capture || traversal.render_geometry) {
                    index_count
                } else {
                    0
                });
            for _ in 0..index_count {
                let index = u16::from_le_bytes(r.bytes()?) as usize;
                if index >= count {
                    return r.fail("kn5_indices", "Index outside vertex array.");
                }
                if kind == 2 && (capture || traversal.render_geometry) {
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
            if capture {
                r.physical_geometry +=
                    collision_positions.len() * size_of::<[f64; 3]>() + indices.len() * 4;
                if r.physical_geometry > MAX_GEOMETRY {
                    return r.fail("kn5_limit", "Retained physical geometry exceeds 256 MiB.");
                }
                traversal.physical.as_mut().unwrap().push(PhysicalMesh {
                    name: name.clone(),
                    lod,
                    positions: collision_positions,
                    indices,
                });
            } else if traversal.render_geometry
                && kind == 2
                && active
                && visible
                && renderable
                && lod[0] <= 0.
                && !indices.is_empty()
            {
                if (scene.vertices.len() + count) * size_of::<Vertex>()
                    + (scene.indices.len() + indices.len()) * 4
                    > MAX_GEOMETRY
                {
                    return r.fail("kn5_limit", "Native render buffers exceed 256 MiB.");
                }
                let base = scene.vertices.len() as u32;
                let mut bounds = None;
                for (position, (normal, uv)) in positions.into_iter().zip(attributes) {
                    grow_bounds(&mut scene.summary.bounds, position);
                    grow_bounds(&mut bounds, position);
                    scene.vertices.push(Vertex {
                        position,
                        normal,
                        uv,
                        colour: colours[material],
                    });
                }
                scene.summary.drawn_meshes += 1;
                scene.summary.drawn_triangles += indices.len() / 3;
                let b = bounds.unwrap();
                scene.draws.push(Draw {
                    start: scene.indices.len() as u32,
                    count: indices.len() as u32,
                    material,
                    center: std::array::from_fn(|a| {
                        ((b[0][a] as f64 + b[1][a] as f64) * 0.5) as f32
                    }),
                });
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
    if let Some(nodes) = &mut traversal.nodes {
        nodes.push(NodeTransform {
            name,
            kind,
            active,
            parent: parent_index,
            world: world.to_cols_array(),
        });
    }
    traversal.parent_index = Some(node_index);
    for _ in 0..children {
        node(r, scene, colours, world, active, depth + 1, traversal)?;
    }
    traversal.parent_index = parent_index;
    Ok(())
}

fn grow_bounds(bounds: &mut Option<[[f32; 3]; 2]>, position: [f32; 3]) {
    let bounds = bounds.get_or_insert([position, position]);
    for (i, &v) in position.iter().enumerate() {
        bounds[0][i] = bounds[0][i].min(v);
        bounds[1][i] = bounds[1][i].max(v);
    }
}
