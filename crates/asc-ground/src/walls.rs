//! Static double-sided numeric-prefix WALL triangles, independent of support-surface definitions.
use super::*;
use glam::DVec3;
const MAX_WALL_TRIANGLES: usize = 250_000;
#[derive(Serialize)]
struct WallMesh {
    name: String,
    model: String,
    triangles: usize,
}
pub struct WallIndex {
    triangles: Vec<Triangle>,
    meshes: Vec<WallMesh>,
    order: Vec<usize>,
    nodes: Vec<Node>,
    diagnostics: Vec<Error>,
    input_triangles: usize,
    skipped_degenerate: usize,
    metadata_bytes: usize,
}
#[derive(Debug, Serialize)]
pub struct WallContact<'a> {
    pub geometry: BoxTriangleContact,
    pub triangle: usize,
    pub mesh: &'a str,
    pub model: &'a str,
}
#[derive(Debug, Serialize)]
pub struct WallSweep<'a> {
    /// Conservative proximity event, not an exact time of geometric intersection.
    pub fraction: Option<f64>,
    pub iterations: usize,
    pub candidate_count: usize,
    pub distance_tolerance_m: f64,
    pub contacts: Vec<WallContact<'a>>,
}
impl WallIndex {
    pub fn load(config: &Path) -> Result<Self> {
        let mut index = Self {
            triangles: Vec::new(),
            meshes: Vec::new(),
            order: Vec::new(),
            nodes: Vec::new(),
            diagnostics: Vec::new(),
            input_triangles: 0,
            skipped_degenerate: 0,
            metadata_bytes: 0,
        };
        let mut node_count = 0;
        let declared = asc_scene::sources(config)?;
        for warning in declared.diagnostics {
            index.warning(Error::new(
                "scene_declaration",
                &config.to_string_lossy(),
                0,
                &warning,
            ))?;
        }
        for entry in declared.models {
            let collision = asc_kn5::load_collision(&entry.path)
                .map_err(|e| Error::new(e.code, &entry.file, 0, &e.message))?;
            node_count += collision.summary.nodes;
            if node_count > 200_000 {
                return Err(Error::new(
                    "wall_limit",
                    &entry.file,
                    0,
                    "Combined models exceed 200000 nodes",
                ));
            }
            if collision.summary.skinned_meshes > 0 {
                index.warning(Error::new(
                    "wall_skinning_unsupported",
                    &entry.file,
                    0,
                    "Skinned meshes are validated structurally and omitted from static walls",
                ))?;
            }
            for mesh in collision.meshes {
                if mesh.indices.is_empty() || !mesh.name.to_ascii_uppercase().contains("WALL") {
                    continue;
                }
                if mesh.lod[0] > 0. {
                    index.warning(Error::new(
                        "wall_lod_unsupported",
                        &entry.file,
                        0,
                        &format!("{}: positive near-LOD wall geometry is omitted", mesh.name),
                    ))?;
                    continue;
                }
                index.input_triangles += mesh.indices.len() / 3;
                if index.input_triangles > MAX_WALL_TRIANGLES {
                    return Err(Error::new(
                        "wall_limit",
                        &entry.file,
                        0,
                        "WALL input exceeds 250000 triangles",
                    ));
                }
                index.retain_metadata(mesh.name.len() + entry.file.len())?;
                let mut positions = mesh.positions;
                for point in &mut positions {
                    for (v, shift) in point.iter_mut().zip(entry.position) {
                        *v += shift;
                    }
                    if point
                        .iter()
                        .any(|x| !x.is_finite() || x.abs() > MAX_COORDINATE)
                    {
                        return Err(Error::new(
                            "wall_range",
                            &entry.file,
                            0,
                            "Translated WALL coordinates exceed +/-10000000 m",
                        ));
                    }
                }
                let start = index.triangles.len();
                for ids in mesh.indices.chunks_exact(3) {
                    let points = std::array::from_fn(|i| positions[ids[i] as usize]);
                    let [a, b, c] = points.map(DVec3::from_array);
                    let u = b - a;
                    let v = c - a;
                    if u.cross(v).length() <= 1e-12 * (u.length() * v.length()).max(1.) {
                        index.skipped_degenerate += 1;
                        continue;
                    }
                    let bounds = Bounds {
                        lo: std::array::from_fn(|i| {
                            points[0][i].min(points[1][i]).min(points[2][i])
                        }),
                        hi: std::array::from_fn(|i| {
                            points[0][i].max(points[1][i]).max(points[2][i])
                        }),
                    };
                    index.triangles.push(Triangle {
                        points,
                        mesh: index.meshes.len(),
                        bounds,
                    });
                }
                index.meshes.push(WallMesh {
                    name: mesh.name,
                    model: entry.file.clone(),
                    triangles: index.triangles.len() - start,
                });
            }
        }
        if !index.triangles.is_empty() {
            index.order = (0..index.triangles.len()).collect();
            spatial::build(
                &index.triangles,
                &mut index.order,
                &mut index.nodes,
                0,
                index.triangles.len(),
                true,
            );
        } else {
            index.warning(Error::new(
                "no_walls",
                &config.to_string_lossy(),
                0,
                "No supported WALL triangles assembled",
            ))?;
        }
        index.warning(Error::new("wall_query_scope",&config.to_string_lossy(),0,"Static numeric-prefix WALL geometry only. No inferred surface coefficients, obstacle classes, collision response or native driving CCD; prescribed rigid-path sweep queries are available"))?;
        Ok(index)
    }
    fn retain_metadata(&mut self, bytes: usize) -> Result<()> {
        self.metadata_bytes += bytes;
        if self.metadata_bytes > 32 * 1024 * 1024 {
            return Err(Error::new(
                "wall_limit",
                "<walls>",
                0,
                "WALL metadata exceeds 32 MiB",
            ));
        }
        Ok(())
    }
    fn warning(&mut self, error: Error) -> Result<()> {
        self.retain_metadata(error.code.len() + error.source.len() + error.message.len())?;
        self.diagnostics.push(error);
        Ok(())
    }
    pub fn summary(&self) -> serde_json::Value {
        serde_json::json!({"scope":"native_static_wall_triangles","triangles":self.triangles.len(),"input_triangles":self.input_triangles,"meshes":self.meshes,"skipped_degenerate":self.skipped_degenerate,"bvh_ready":!self.nodes.is_empty(),"diagnostics":self.diagnostics,"collision_response":false,"continuous_collision_detection":false,"prescribed_rigid_path_sweep":true})
    }
    /// All contacts from at most 4096 full-3D AABB candidates, sorted by retained triangle index.
    /// Overflow is an error; no partial or truncated report is returned.
    pub fn contacts(&self, body: &OrientedBox, margin_m: f64) -> Result<Vec<WallContact<'_>>> {
        let (center, half) = body.aabb(margin_m)?;
        let bounds = Bounds {
            lo: std::array::from_fn(|i| center[i] - half[i]),
            hi: std::array::from_fn(|i| center[i] + half[i]),
        };
        let indices =
            spatial::candidates(&self.triangles, &self.order, &self.nodes, bounds, || {
                Error::new(
                    "wall_query_limit",
                    "<wall box query>",
                    0,
                    "Local WALL query exceeds 4096 triangle candidates",
                )
            })?;
        let mut result = Vec::new();
        for i in indices {
            let triangle = &self.triangles[i];
            if let Some(geometry) = body.contact_triangle(triangle.points, margin_m)? {
                let mesh = &self.meshes[triangle.mesh];
                result.push(WallContact {
                    geometry,
                    triangle: i,
                    mesh: &mesh.name,
                    model: &mesh.model,
                });
            }
        }
        Ok(result)
    }
    /// Conservative advancement along a prescribed translation/shortest-arc rotation.
    /// A clear result certifies this path within numeric tolerances; iteration exhaustion is an error.
    pub fn sweep(&self, motion: &BoxMotion, settings: WallSweepSettings) -> Result<WallSweep<'_>> {
        settings.validate()?;
        let indices = spatial::candidates(
            &self.triangles,
            &self.order,
            &self.nodes,
            motion.bounds(settings.distance_tolerance_m)?,
            || {
                Error::new(
                    "wall_sweep_limit",
                    "<wall sweep>",
                    0,
                    "Swept bounds exceed 4096 WALL candidates",
                )
            },
        )?;
        let mut result = WallSweep {
            fraction: None,
            iterations: 0,
            candidate_count: indices.len(),
            distance_tolerance_m: settings.distance_tolerance_m,
            contacts: Vec::new(),
        };
        if indices.is_empty() {
            return Ok(result);
        }
        let speed = motion.speed_bound();
        let mut fraction = 0.;
        for iteration in 1..=settings.max_iterations {
            let body = motion.at(fraction)?;
            let mut gap = f64::INFINITY;
            for &i in &indices {
                gap = gap.min(body.separation_triangle(self.triangles[i].points)?);
            }
            result.iterations = iteration;
            if gap <= settings.distance_tolerance_m + 1e-9 {
                result.fraction = Some(fraction);
                result.contacts = self.contacts(&body, settings.distance_tolerance_m)?;
                if result.contacts.is_empty() {
                    return Err(Error::new(
                        "wall_sweep_patch",
                        "<wall sweep>",
                        0,
                        "Proximity event has no finite contact patch",
                    ));
                }
                return Ok(result);
            }
            if fraction == 1. || speed == 0. {
                return Ok(result);
            }
            let advance = (gap - settings.distance_tolerance_m) / speed;
            if !advance.is_finite() || advance <= 1e-15 {
                return Err(Error::new(
                    "wall_sweep_progress",
                    "<wall sweep>",
                    0,
                    "Conservative sweep cannot make finite progress",
                ));
            }
            fraction = (fraction + advance).min(1.);
        }
        Err(Error::new(
            "wall_sweep_iterations",
            "<wall sweep>",
            0,
            "Conservative sweep exceeded the iteration budget; path is not certified clear",
        ))
    }
}
