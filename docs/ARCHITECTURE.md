# Architecture

The implemented project contains a first native Rust KN5 reader and wgpu offscreen renderer and desktop inspection window, Python content readers, a WebGL 2 preview, an independent fixed-step Python driving prototype and a documentation website. Production rendering and validated full vehicle simulation remain incomplete.

## Implemented components

| Location | Responsibility |
| --- | --- |
| `crates/asc-app/src/app/session.rs` | Native static track/car assembly, source namespace rebasing and inspection-pose metadata |
| `crates/asc-render/src/scene/pose.rs` | Bounded rigid root composition and transformed bounds |
| `crates/asc-render/src/gpu/instances.rs` | Persistent root uniforms and disjoint draw-group binding |
| `crates/asc-vehicle/src/suspension.rs` | Source-aware STRUT/DWB hardpoints and numeric records without force-law inference |
| `crates/asc-physics/src/spring.rs` | Passive spring/damper loads and atomic common-midpoint multibody stepping |
| `crates/asc-physics/` | Explicit 3D mass/inertia, world momenta, forces/impulses and fixed-step scheduling |
| `crates/asc-physics/src/linkage.rs` | Five-distance upright constraints and shared bounded local Newton iteration |
| `crates/asc-physics/src/strut.rs` | Three-distance upright with a telescoping coaxial strut |
| `crates/asc-session/src/kinematics.rs` | Explicit DWB/STRUT hardpoint binding in the uncorrected raw design frame |
| `crates/asc-session/src/body.rs` | Rigid-body seed using a spawn basis and explicitly supplied centre of mass/tensor |
| `crates/asc-session/` | Native geometric vehicle/spawn preparation and four-wheel support preflight |
| `crates/asc-scene/src/spawns.rs` | Canonical rigid spawn bases, active ancestry and ambiguity diagnostics |
| `crates/asc-ground/` | Explicit surface definitions, independent static physical geometry and vertical support BVH |
| `crates/asc-acd/` | Bounded ordinary folder-keyed ACD index and in-memory record reads |
| `crates/asc-vehicle/` | Whole-source packed/unpacked selection, typed vehicle configuration and field provenance |
| `crates/asc-config/` | Bounded source-aware native INI/LUT readers and interpolation |
| `crates/asc-scene/` | Confined models.ini loading, static translation, rebased geometry and source resource ranges |
| `tools/inspect_content.py` | Read-only content inventory and JSON CLI |
| `tools/prototype/openasc/config.py` | Source-aware INI and LUT parsing, numeric validation, and interpolation |
| `tools/prototype/openasc/scene.py` | Confined static model references, transforms, and KN5 header inspection |
| `tools/prototype/openasc/kn5.py` | Bounded KN5 parsing, static meshes, node transforms, material definitions, and embedded texture offsets |
| `tools/prototype/openasc/dds.py` | DDS validation, mip descriptors, and bounded embedded texture reads |
| `tools/prototype/openasc/surfaces.py` | Typed surface definitions, explicit file precedence, and conservative physical mesh naming |
| `tools/prototype/openasc/ground.py` | World-space ground triangles, a BVH, and vertical geometric support queries |
| `tools/prototype/openasc/acd.py` | Bounded in-memory indexing and reading of ordinary folder-keyed ACD records |
| `tools/prototype/openasc/vehicle.py` | Explicit packed/unpacked car-data selection and typed vehicle configuration subset |
| `tools/prototype/openasc/placement.py` | Rigid spawn transforms and a four-wheel geometric support preflight |
| `tools/prototype/openasc/dynamics.py` | Independent fixed-step planar force model with four ground contact patches |
| `tools/prototype/openasc/driving.py` | Car/track preview assembly, a sequenced local controller and simulation clock |
| `tools/prototype/openasc/spatial.py` | Shared deterministic static AABB hierarchy |
| `tools/prototype/openasc/walls.py` | Static wall geometry, a collider-bounds body proxy, continuous SAT and planar contact impulses |
| `tools/prototype/openasc/viewer.py` | Local scene assembly and loopback-only preview server |
| `tools/prototype/viewer/` | WebGL 2 diffuse preview, portable DDS pixel decoding, camera controls, and mesh/material inspection |
| `tools/prototype/openasc/__main__.py` | JSON diagnostics through `python -m openasc` |
| `tests/` | Tests using original, temporary fixtures |
| `docs/` | English documentation and current compatibility record |
| `web/` | Shared website template and overview source |
| `tools/build_site.py` | Build documentation pages using Python's standard library |
| `site/` | Generated HTML, shared CSS/JS, and public static assets |
| `.github/workflows/` | Source checks and GitHub Pages deployment |
| `crates/asc-kn5/src/lib.rs` | Independent bounded Rust KN5 structural reader and static world geometry |
| `crates/asc-render/src/gpu/capture.rs` | Native offscreen capture and frame metadata |
| `crates/asc-render/src/gpu/scene.rs` | Persistent buffers, shared render pass and scene statistics |
| `crates/asc-render/src/gpu/presenter.rs` | SDR surface presentation, reconfiguration and capture |
| `crates/asc-render/src/gpu/readback.rs` | Shared aligned RGBA/BGRA readback |
| `crates/asc-render/src/scene/orbit.rs` | Bounded camera orbit, pan, zoom and projections |
| `crates/asc-app/src/app/viewer.rs` | winit lifecycle, desktop controls and diagnostic smoke mode |
| `crates/asc-texture/src/lib.rs` | Bounded DDS decoding and authored mip preservation |
| `crates/asc-render/src/resources/texture.rs` | Unique active embedded diffuse resolution and scene budgets |
| `crates/asc-render/src/pipelines/material.rs` | Reconstructed multilayer base colour and approximate lighting, alpha and depth pipelines |
| `crates/asc-render/src/scene/material.rs` | Validated scalar/vector interpretation and planar/UV layer coordinates |
| `crates/*/tests/` | Original format, resource, camera, material pixel and PNG fixtures |
| `crates/asc-app/` | Native CLI executable named openasc |
| `tools/test-support/` | Shared original Rust fixture crate |
| `tools/asc-telemetry/` | Read-only Windows original-game prefix acquisition and bounded research JSONL journals; see [telemetry](TELEMETRY.md) |
| `tools/check_native_parity.py` | Native/Python reader contract comparisons; not original-game physics validation |

## Workspace and subsystem boundaries

The workspace follows the subsystem-crate organisation of [openOMSI](https://github.com/openOMSI-org/openOMSI/blob/main/Cargo.toml), with original openASC implementation code. `asc-app` assembles the native content, rendering and session subsystems. Workspace dependency versions and local crate paths are declared in the root `Cargo.toml`; `Cargo.lock` pins the resolved build. Each subsystem keeps its implementation in its own `src/` and its integration tests in its own `tests/` where needed.

The vehicle crate depends on the configuration and ACD readers. `asc-scene` depends on the configuration and KN5 readers; `asc-ground` builds physical support queries from those source records. `asc-physics` contains independent rigid-body, spring and clock calculations without depending on content readers or rendering. `asc-session` combines vehicle, scene, ground and explicit physics inputs for preparation. These crates have no wgpu dependency and can be tested independently of a GPU. `asc-render` consumes scene source ranges alongside KN5 geometry and DDS resources; its sources are divided into `gpu/`, `pipelines/`, `resources/`, `scene/` and `shaders/`. The shared original fixture crate lives under `tools/test-support/` and is a development dependency. Future input and audio implementations belong in sibling crates when implemented.

The earlier Python/WebGL research implementation lives under `tools/prototype/`. The root `openasc/__init__.py` only redirects package discovery there, preserving `python -m openasc` and existing Python imports. Website sources and documentation remain separate under `web/`, `docs/` and `site/`. No openOMSI implementation files are included.

| openOMSI responsibility | openASC source location |
| --- | --- |
| `crates/omsi-app` application | `crates/asc-app` |
| `crates/omsi-cfg` configuration | `crates/asc-config` for AC INI/LUT records |
| `crates/omsi-o3d` model-format reader | `crates/asc-kn5` for AC's model format |
| `crates/omsi-scene` scene assembly | `crates/asc-scene`; physical support in `crates/asc-ground` |
| `crates/omsi-vehicle` vehicle data | `crates/asc-vehicle`; session preparation in `crates/asc-session` |
| `crates/omsi-texture` texture formats | `crates/asc-texture` |
| `crates/omsi-render` rendering subsystems | `crates/asc-render` |
| `crates/omsi-sim` simulation | `crates/asc-physics` independent mathematical kernel; full vehicle simulation remains incomplete |
| `tools/test-support` test utilities | `tools/test-support` |
| `docs` and `site` documentation/publishing | `docs` and `site` |

The table maps responsibilities, not APIs or compatibility levels. Assetto Corsa-specific formats retain their own crate names. Platform packaging, launcher and network-service directories can be added when those components exist.

- **Content:** configuration readers, resource resolution, metadata, and precise loading diagnostics.
- **Scene:** geometry, material definitions, transforms, surfaces, and collision data independent of a renderer.
- **Render:** scene inspection, camera, lighting, and material preview.
- **Simulation:** fixed simulation step, drivetrain, suspension, aerodynamics, and tyres.
- **Platform:** windowing, clocks, input devices, and force feedback.
- **Application:** content selection, session setup, diagnostics, and the driving loop.

Keep file-format readers separate from rendering and physics so each can be tested without opening a window. Unsupported resources should produce diagnostics rather than silently appearing compatible.

## Technology choices

The selected production stack is Rust for the engine core, including simulation, device input and force feedback, and wgpu for rendering. The first native content/render slice is implemented; native simulation, driving-device input and FFB are not. Python currently keeps the broader research content loading and local HTTP server dependency-free. The browser inspection renderer uses WebGL 2 with original JavaScript and remains available alongside the native diagnostic renderer.

The native binary directly opens one KN5 through a bounded buffered Rust reader; it does not call Python or the browser. Source node transforms are composed in double precision, validated and flattened into float32 world positions with original indices rebased into shared GPU buffers. UVs are retained and normals use inverse-transpose transforms, with geometric fallback for singular transforms. The renderer resolves unique active embedded txDiffuse references, retains validated BC1/2/3 source blocks or decodes supported raw DDS within budgets, and uploads authored mip chains as sRGB BC or RGBA resources. Device creation negotiates optional BC support; unsupported/explicit RGBA operation decodes retained blocks one texture at a time, with a shared upload budget. Six compatible WGSL pipelines cover approximate blend/depth combinations. Known multilayer variants bind a linear RGBA mask and four sRGB detail maps; validated scalar/vector uniforms select world-XZ or source-UV projection and the reconstructed base-colour formula. Raw mask mip generation averages data channels directly. Transparent meshes are sorted by camera depth. A padded GPU copy is mapped back, unpacked row by row and encoded into an explicitly requested new PNG. Camera framing uses active near-level ROAD bounds when present, otherwise visible scene bounds. The native configuration path assembles multiple static MODEL_n sources with validated translations and per-model texture ranges. Original material fidelity and simulation remain separate work. The desktop window uses winit lifecycle events and a persistent GpuScene shared with offscreen capture. Orbit camera changes update a uniform buffer and transparency sort direction without rebuilding geometry or textures. Idle windows wait for events. SDR surface reconfiguration and optional readback are isolated in the presenter.

The local viewer receives a metadata manifest and aligned vertex/index buffers from its loopback server. Static vertices retain their original interleaved format. Node transforms are composed on the CPU and applied per draw on the GPU. The binary importer indexes texture offsets without extracting assets. The preview then validates and loads referenced embedded diffuse textures into separate bounded memory buffers. Only numeric texture IDs are exposed by same-origin local routes; no disk path or resource export endpoint is exposed. The browser uses S3TC uploads when possible, with an independent RGBA software fallback. Authored compressed mip chains are retained; raw texture base levels use GPU-generated mipmaps. The public documentation site and local viewer are separate applications.

## Prototype implementation language

The current dynamics and static wall solver are also Python reference implementations for the prototype. This keeps the mathematical fixtures, content readers and first local driving loop in one testable toolchain. Measured single-car timings are not hard real-time guarantees. Move production simulation and platform work into the planned Rust core, retaining Python for research, content inspection and validation. Preserve verified format contracts and regression fixtures during migration, but do not treat the simplified prototype force model as original Assetto Corsa physics. WebGL rendering is JavaScript/GPU work, separate from the Python solver.

## Ground support queries

The first contact preparation path is separate from rendering. `ground` loads system and selected track surface definitions, includes supported active physical meshes even when hidden/non-renderable, transforms their vertices, and builds a median BVH. A downward probe returns the nearest triangle within its finite distance, an upward geometric normal, and the source surface definition. It does not execute a simulation step, tyre forces, procedural surface bumps or wall response. Surface friction is recorded input for later physics, not a validated grip model.

## Vehicle configuration and placement

The vehicle configuration reader uses a single selected data source, without mixing files from an archive and a directory. It retains source locations for consumed INI fields and loads the referenced torque LUT without extraction. Inertia dimensions, steering and axle geometry are separate recorded inputs; the configuration reader does not execute vehicle dynamics. The KN5 reader also retains node parent links, active state and composed transforms, so invisible spawn markers survive independently of drawing visibility. The placement preflight maps the car's wheel footprint through a selected source basis, then queries static support. It does not solve chassis height, spring equilibrium or collision response.

## First-drive runtime

The first-drive prototype has a separate Python simulation state, stepped at 120 Hz by a clock thread. HTTP telemetry and WebGL redraws do not advance physics. A locked session applies complete, validated, sequenced controls; missing heartbeat input pauses and clears controls. Clock catch-up is capped at eight steps per wake, with discarded backlog counted in telemetry. A browser tab polls telemetry and sends keyboard input, while the existing WebGL buffers draw the car beneath its current world transform. Track and car texture/geometry budgets are shared. The simulation constrains vertical pose to supported static ground and resolves static wall contact with an upright body box. Full six-degree-of-freedom suspension and collision remain unimplemented.

## Static wall contact

Static WALL triangles are collected alongside ground geometry and receive their own 3D BVH. The car's collider model supplies a conservative upright box. A continuous separating-axis query checks translation and yaw, then the fixed-step model resolves a clipped planar manifold with inelastic normal/tangential impulses. Ground attachment and a final proxy overlap check gate the accepted pose. Numeric/iteration failures remain explicit, while a bounded contact-event cap drops unresolved remaining motion and increments telemetry. Ground probes, spawn inspection and the read-only geometry viewer retain their separate scopes; this does not introduce a complete 6DOF collision engine.

## Website

The website uses a shared documentation layout inspired by [openomsi.org](https://openomsi.org/): persistent navigation, a project overview, and dedicated documentation pages. Its original visual styling uses a charcoal, red, and white motorsport palette, italic branding, and a dark default theme. The implementation and content are original to openASC.

Documentation pages are generated from the same Markdown files that appear on GitHub. Run `python tools/build_site.py` after editing documentation or the shared template, then commit the generated HTML. Relative assets work beneath `/openASC/` on GitHub Pages.

## Rigid upright kinematics

`asc-physics::FiveLink` receives five fixed chassis anchors, five upright-local attachments, a reference pose and a unit travel axis. Two attachments may represent the same upper joint, two the same lower joint, and the fifth the steering joint. Each reference length is computed from that explicit pose. The origin is the caller's chosen travel point; this generic solver does not infer a wheel centre from a mesh.

For each link, `d = t + R*r - anchor` and the residual is `length(d) - reference_length`. The final constraint is `axis dot (t - reference_t) - travel`. With `u = d / length(d)`, translation derivatives are `u`; derivatives for a left-applied incremental rotation are `(R*r) cross u`. Rotational columns are scaled by the largest attachment radius, with a 0.001 m floor. Only the fifth chassis anchor receives the explicit steering displacement.

The implementation uses an independently written partial-pivot 6-by-6 elimination and damped Newton updates. A scaled pivot below 1e-10 rejects the local system, including an already zero-residual reference; this is a numerical gate, not a formal condition-number estimate. Updates are bounded to 0.05 m and 0.15 rad, with at most twelve residual-decreasing backtracking attempts per update and forty updates. Every final residual must be within 1e-10 m. Position inputs/outputs are bounded to +/-100 m, reference lengths to 1e-6..100 m, and commands to 0.25 m. The local pose remains within 0.5 m and 1 rad of its reference. Seeds guide nearby solutions but do not establish uniqueness or guarantee a global assembly branch. Failure does not mutate the mechanism or seed.

This geometric distance-constraint description of wishbones is also documented by [Project Chrono](https://api.projectchrono.org/wheeled_suspension.html). No Chrono implementation code or Assetto Corsa solver was copied. This module supplies position kinematics only; control-arm inertia, constraint reactions and spring/tyre forces are outside this solver.

`asc-session::dwb_geometry` binds the eight explicit DWB point names to the five links at the raw zero/identity design pose. The `suspension-motion` CLI exposes this assumption and requires an explicit travel axis. It does not infer the AC coordinate/mirroring convention or apply source setup corrections. Those mappings need original-game measurements before use in a complete vehicle simulation.

## Telescoping strut kinematics

`asc-physics::StrutLink` constrains one rigid upright with three distances: two lower-arm legs and a steering link. Its fixed chassis top lies on an upright-local line through an explicit strut attachment. The reference pose determines that line's positive direction; two orthogonal local normals span its perpendicular plane. This is an independently written geometric idealization, separate from the multibody MacPherson template documented by [Project Chrono](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_ch_mac_pherson_strut.html), and is not a reconstruction of AC's solver.

For a local normal `n`, upright point `s`, fixed top `A` and pose `(t,R)`, each coaxial residual is `(R*n) dot (A-t-R*s)`. Its translation Jacobian is `-R*n`; its left-rotation Jacobian is `(R*n) cross (A-t)`. Both residuals have metre units. Signed axial length is `(R*axis) dot (A-t-R*s)` and must remain within 1e-6..100 m; collapse or reversal fails. The column scale includes the attachment radii and reference top lever arm. The same bounded Newton loop, travel constraint and tolerances serve DWB and STRUT; only the topology-specific constraints differ.

`StrutSolution` keeps its three length errors separate from its two perpendicular errors and reports axial length and geometric extension from the reference. `asc-session::strut_geometry` binds the seven explicit source points at zero/identity design pose. The CLI dispatches by the source TYPE while retaining the DWB report shape for DWB. No spring-law, unsprung-mass, side/setup conversion or renderer animation is introduced by this position solve.
