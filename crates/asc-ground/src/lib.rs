//! Static vertical probes and bounded finite face views, independent of rendering and force laws.
mod faces;
pub mod surfaces;
use asc_config::{Error, Result};
pub use faces::Face;
use serde::Serialize;
use std::path::Path;
use surfaces::Surface;

const MAX_COORDINATE: f64 = 10_000_000.;
const MAX_TRIANGLES: usize = 1_000_000;
type Point = [f64; 3];
#[derive(Clone, Copy)]
struct Bounds {
    lo: Point,
    hi: Point,
}
impl Bounds {
    fn union(self, b: Self) -> Self {
        Self {
            lo: std::array::from_fn(|i| self.lo[i].min(b.lo[i])),
            hi: std::array::from_fn(|i| self.hi[i].max(b.hi[i])),
        }
    }
    fn intersects(&self, p: Point, distance: f64) -> bool {
        p[0] >= self.lo[0] - 1e-7
            && p[0] <= self.hi[0] + 1e-7
            && p[2] >= self.lo[2] - 1e-7
            && p[2] <= self.hi[2] + 1e-7
            && self.lo[1] <= p[1] + 1e-7
            && self.hi[1] >= p[1] - distance - 1e-7
    }
}
struct Triangle {
    points: [Point; 3],
    mesh: usize,
    bounds: Bounds,
}
struct Node {
    bounds: Bounds,
    start: usize,
    end: usize,
    children: Option<[usize; 2]>,
}
#[derive(Serialize)]
pub struct Mesh {
    pub name: String,
    pub model: String,
    pub surface: Surface,
    pub triangles: usize,
}
pub struct Ground {
    triangles: Vec<Triangle>,
    meshes: Vec<Mesh>,
    order: Vec<usize>,
    nodes: Vec<Node>,
    input_triangles: usize,
    skipped_degenerate: usize,
    diagnostics: Vec<Error>,
    metadata_bytes: usize,
}
#[derive(Debug, Serialize)]
pub struct Hit<'a> {
    pub position: Point,
    pub distance: f64,
    pub normal: Point,
    pub mesh: &'a str,
    pub model: &'a str,
    pub triangle: usize,
    pub surface: &'a Surface,
}

impl Ground {
    pub fn diagnostics(&self) -> &[Error] {
        &self.diagnostics
    }
    fn retain_metadata(&mut self, bytes: usize) -> Result<()> {
        self.metadata_bytes += bytes;
        if self.metadata_bytes > 32 * 1024 * 1024 {
            return Err(Error::new(
                "ground_limit",
                "<ground>",
                0,
                "Retained mesh/surface/diagnostic metadata payload exceeds 32 MiB.",
            ));
        }
        Ok(())
    }
    fn warning(&mut self, value: Error) -> Result<()> {
        self.retain_metadata(value.code.len() + value.source.len() + value.message.len())?;
        self.diagnostics.push(value);
        Ok(())
    }
    fn build_node(&mut self, start: usize, end: usize) -> usize {
        let bounds = self.order[start + 1..end]
            .iter()
            .fold(self.triangles[self.order[start]].bounds, |a, &i| {
                a.union(self.triangles[i].bounds)
            });
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            start,
            end,
            children: None,
        });
        if end - start > 8 {
            let axis = if bounds.hi[0] - bounds.lo[0] >= bounds.hi[2] - bounds.lo[2] {
                0
            } else {
                2
            };
            let triangles = &self.triangles;
            self.order[start..end].sort_unstable_by(|&a, &b| {
                let centre = |i: usize| triangles[i].bounds.lo[axis] + triangles[i].bounds.hi[axis];
                centre(a).total_cmp(&centre(b)).then(a.cmp(&b))
            });
            let middle = start + (end - start) / 2;
            let left = self.build_node(start, middle);
            let right = self.build_node(middle, end);
            self.nodes[index].children = Some([left, right]);
        }
        index
    }
    pub fn summary(&self) -> serde_json::Value {
        serde_json::json!({"scope":"native_static_ground_query","triangles":self.triangles.len(),"input_triangles":self.input_triangles,"meshes":self.meshes,"bvh_ready":!self.nodes.is_empty(),"skipped_vertical_or_degenerate":self.skipped_degenerate,"diagnostics":self.diagnostics,"simulation_ready":false})
    }
    /// Nearest double-sided triangle below origin; inclusive shared edges, upward geometric normal.
    pub fn probe(&self, origin: Point, max_distance: f64) -> Result<Option<Hit<'_>>> {
        if origin
            .iter()
            .any(|v| !v.is_finite() || v.abs() > MAX_COORDINATE)
            || !max_distance.is_finite()
            || !(0. ..=MAX_COORDINATE).contains(&max_distance)
        {
            return Err(Error::new(
                "ground_query_range",
                "<ground query>",
                0,
                "Expected finite coordinates within +/-10000000 metres and distance in 0..10000000.",
            ));
        }
        let mut closest = max_distance;
        let mut result: Option<Hit<'_>> = None;
        let mut stack = Vec::new();
        if !self.nodes.is_empty() {
            stack.push(0);
        }
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index];
            if !node.bounds.intersects(origin, closest) {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            for &index in &self.order[node.start..node.end] {
                let triangle = &self.triangles[index];
                let [a, b, c] = triangle.points;
                let u: Point = std::array::from_fn(|i| b[i] - a[i]);
                let v: Point = std::array::from_fn(|i| c[i] - a[i]);
                let area = u[0] * v[2] - u[2] * v[0];
                let px = origin[0] - a[0];
                let pz = origin[2] - a[2];
                let beta = (px * v[2] - pz * v[0]) / area;
                let gamma = (u[0] * pz - u[2] * px) / area;
                if beta.min(gamma).min(1. - beta - gamma) < -1e-9 {
                    continue;
                }
                let height = a[1] + beta * u[1] + gamma * v[1];
                let travel = origin[1] - height;
                if travel < -1e-7 || travel > closest + 1e-7 {
                    continue;
                }
                if let Some(hit) = &result {
                    if travel > closest - 1e-7 && index > hit.triangle {
                        continue;
                    }
                }
                let normal = [u[1] * v[2] - u[2] * v[1], -area, u[0] * v[1] - u[1] * v[0]];
                let magnitude = normal[0].hypot(normal[1]).hypot(normal[2]);
                let sign = if normal[1] > 0. { 1. } else { -1. };
                let mesh = &self.meshes[triangle.mesh];
                closest = travel.max(0.);
                result = Some(Hit {
                    position: [origin[0], height, origin[2]],
                    distance: closest,
                    normal: normal.map(|n| sign * n / magnitude),
                    mesh: &mesh.name,
                    model: &mesh.model,
                    triangle: index,
                    surface: &mesh.surface,
                });
            }
        }
        Ok(result)
    }
}

pub fn load(config: &Path, surface_files: &[&Path]) -> Result<Ground> {
    let declared = asc_scene::sources(config)?;
    let (surfaces, diagnostics) = surfaces::load(surface_files)?;
    let mut ground = Ground {
        triangles: Vec::new(),
        meshes: Vec::new(),
        order: Vec::new(),
        nodes: Vec::new(),
        input_triangles: 0,
        skipped_degenerate: 0,
        diagnostics,
        metadata_bytes: 0,
    };
    ground.retain_metadata(
        ground
            .diagnostics
            .iter()
            .map(|d| d.source.len() + d.code.len() + d.message.len())
            .sum(),
    )?;
    let source = declared.config.to_string_lossy();
    for warning in declared.diagnostics {
        ground.warning(Error::new("scene_declaration", &source, 0, &warning))?;
    }
    let mut nodes = 0;
    for entry in declared.models {
        let collision = asc_kn5::load_collision(&entry.path).map_err(|e| {
            Error::new(
                "ground_model",
                &source,
                entry.line,
                &format!("{}: {e}", entry.file),
            )
        })?;
        nodes += collision.summary.nodes;
        if nodes > 200_000 {
            return Err(Error::new(
                "ground_limit",
                &source,
                entry.line,
                "Combined models exceed 200000 nodes.",
            ));
        }
        if collision.summary.skinned_meshes > 0 {
            ground.warning(Error::new(
                "ground_skinning_unsupported",
                &entry.file,
                0,
                "Skinned meshes are structurally checked and omitted from static support.",
            ))?;
        }
        for mesh in collision.meshes {
            if mesh.indices.is_empty() {
                continue;
            }
            if mesh.lod[0] > 0. {
                ground.warning(Error::new(
                    "ground_lod_unsupported",
                    &entry.file,
                    0,
                    &format!(
                        "{}: positive near-LOD support is unvalidated and omitted.",
                        mesh.name
                    ),
                ))?;
                continue;
            }
            if mesh.name.to_ascii_uppercase().contains("WALL") {
                ground.warning(Error::new(
                    "ground_wall_omitted",
                    &entry.file,
                    0,
                    "WALL geometry is not part of a vertical support query.",
                ))?;
                continue;
            }
            let surface = match surfaces::mesh_surface(&mesh.name, &surfaces) {
                Ok(Some(s)) => {
                    ground.retain_metadata(
                        mesh.name.len()
                            + entry.file.len()
                            + serde_json::to_vec(s)
                                .map_err(|e| {
                                    Error::new("ground_metadata", &entry.file, 0, &e.to_string())
                                })?
                                .len(),
                    )?;
                    s.clone()
                }
                Ok(None) => continue,
                Err(mut e) => {
                    e.source = entry.file.clone();
                    e.message = format!("{}: {}", mesh.name, e.message);
                    ground.warning(e)?;
                    continue;
                }
            };
            ground.input_triangles += mesh.indices.len() / 3;
            if ground.input_triangles > MAX_TRIANGLES {
                return Err(Error::new(
                    "ground_limit",
                    &entry.file,
                    0,
                    "Ground input exceeds 1000000 triangles.",
                ));
            }
            let mut positions = mesh.positions;
            for point in &mut positions {
                for (value, shift) in point.iter_mut().zip(entry.position) {
                    *value += shift;
                }
                if point
                    .iter()
                    .any(|v| !v.is_finite() || v.abs() > MAX_COORDINATE)
                {
                    return Err(Error::new(
                        "ground_range",
                        &entry.file,
                        0,
                        "Translated ground coordinates exceed +/-10000000 metres.",
                    ));
                }
            }
            let start = ground.triangles.len();
            for indices in mesh.indices.chunks_exact(3) {
                let points: [Point; 3] = std::array::from_fn(|i| positions[indices[i] as usize]);
                let [a, b, c] = points;
                let ux = b[0] - a[0];
                let uz = b[2] - a[2];
                let vx = c[0] - a[0];
                let vz = c[2] - a[2];
                if (ux * vz - uz * vx).abs() <= 1e-12 * 1f64.max(ux.hypot(uz) * vx.hypot(vz)) {
                    ground.skipped_degenerate += 1;
                    continue;
                }
                let bounds = Bounds {
                    lo: std::array::from_fn(|i| a[i].min(b[i]).min(c[i])),
                    hi: std::array::from_fn(|i| a[i].max(b[i]).max(c[i])),
                };
                ground.triangles.push(Triangle {
                    points,
                    mesh: ground.meshes.len(),
                    bounds,
                });
            }
            ground.meshes.push(Mesh {
                name: mesh.name,
                model: entry.file.clone(),
                surface,
                triangles: ground.triangles.len() - start,
            });
        }
    }
    if !ground.triangles.is_empty() {
        ground.order = (0..ground.triangles.len()).collect();
        ground.build_node(0, ground.triangles.len());
    } else {
        ground.warning(Error::new(
            "no_ground",
            &source,
            0,
            "No supported ground triangles assembled.",
        ))?;
    }
    ground.warning(Error::new("ground_query_scope",&source,0,"Static support geometry queries only; surface coefficients are metadata. Response and force laws belong to the caller. Wall collisions, audio and FFB are not simulated here."))?;
    Ok(ground)
}
