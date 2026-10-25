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
| `crates/asc-physics/src/differential.rs` | Relative upright velocities, virtual-work load projection and ideal held-load reactions |
| `crates/asc-physics/src/link_spring.rs` | Explicit fixed-anchor/upright spring mount, motion ratios and projected elastic/damping loads |
| `crates/asc-physics/src/travel.rs` | Fixed-frame/rack massive upright dynamics, exact local constraints and transactional midpoint steps |
| `crates/asc-app/src/app/suspension_run.rs` | Bounded explicit JSON workbench and recorded dynamic samples |
| `crates/asc-physics/src/chassis.rs` | Floating chassis and 1..4 mounted uprights, coupled acceleration solve and atomic Lie-midpoint steps |
| `crates/asc-physics/src/coupled_linear.rs` | Diagonally normalized positive-definite solve for 7..14 coordinates |
| `crates/asc-physics/src/wheel.rs` | Separate axisymmetric rotor mass/tensor, relative spin, external COM wrench and internal motor pair |
| `crates/asc-physics/src/chassis/kinematics.rs` | Force-independent stage snapshots and transactional state-dependent load providers |
| `crates/asc-physics/src/contact.rs` | Thin-circle support geometry and passive unilateral normal law on a static infinite plane |
| `crates/asc-app/src/app/chassis_run.rs` | Explicit per-corner source/frame/mass configuration and bounded coupled runs |
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

The unpacked index bounds enumeration to 4096 immediate entries before sorting. It skips non-file entries and warns for files outside the supported portable-name namespace, without reading their content or treating them as aliases. All entries count toward the enumeration budget, including skipped entries. Portable-name collisions still fail. Requested resources retain strict name, byte, encoding and canonical-root checks; packed archive validation is unchanged. Rust and Python share this selection policy, with diagnostic counts compared on original fixtures.

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

## Suspension differential and held loads

`FiveLink::differential` and `StrutLink::differential` require a converged pose for the supplied travel/rack coordinates. They reuse the topology's analytical constraint Jacobian and the shared pivoted linear solve; there is no finite-difference derivative in the implementation. The six residuals must each remain within 1e-10 m. Command/reference-pose bounds and singularity rejection match the position solver.

Let `L` be the solver's fixed rotational column scale and `y = [v, L*omega]`. For control rates `qdot = [hdot, rackdot_x, rackdot_y, rackdot_z]`, the differentiated constraint equation is `J_scaled*y = C*qdot`. The travel column of `C` is one in row 5. Each rack column is the corresponding translation gradient in the steering-link row (DWB row 4 or STRUT row 2). Other entries are zero. Four solves produce the tangent map `T` after undoing the rotational scale, so `[v, omega] = T*qdot`. Velocities are relative to a fixed linkage frame; moving chassis velocity is not silently added. A local point has velocity `v + omega cross (R*point)`.

`LinkLoad` stores `[force, torque_about_upright_origin]`, expressed in the same fixed frame. A point force uses torque `(R*point) cross force`. Generalized loads are `T_transpose*[force, torque]`; this preserves virtual power `force dot v + torque dot omega = Q dot qdot`. This is the standard Jacobian-transpose statics relation described in [Modern Robotics](https://modernrobotics.northwestern.edu/nu-gm-book-resource/5-2-statics-of-open-chains/). `LinkLoad` is separate from the rigid-body kernel's wrench type because their torque origins differ.

`hold_load` solves `J_scaled_transpose*lambda = -[force, torque/L]` for ideal massless equilibrium at a held pose. Rows 0..4 provide support force and torque. Row 5 supplies an abstract actuator force along the travel axis at the upright origin. DWB rows are upper-front, upper-rear, lower-front, lower-rear and steering distances. STRUT rows are lower-front, lower-rear and steering distances followed by two orthogonal top-to-axis residuals. Multipliers are in N for both topologies because every residual has metre units. Their signs follow each residual gradient; they are not automatically source spring/damper forces.

The travel holding load is `-Q_travel`; the force on the steering anchor is `Q_rack`. Upright support torque is about its moving origin `t`. Support reaction on the fixed frame is `-support_force`, with moment `-support_torque - t cross support_force` about the frame origin. These support reactions explicitly exclude the travel actuator. The sum of support, actuator and applied loads is checked with a 1e-8 scaled relative residual tolerance. There is no physical actuator attachment or automatic full chassis reaction in this diagnostic.

Finite input/output limits reject excessive velocities and loads. The map is immutable and does not advance body state. CLI reports distinguish explicit probes from generated forces and retain false simulation/original-validation flags. Source frame/setup interpretation, validated source spring mounting/force laws, wheel-spin inertia, gravity/contact/tyre forces and a dynamically constrained multibody step remain separate work.

## Explicit spring linkage coupling

`LinkSpring` attaches the existing independent spring/damper element between a fixed linkage-frame point A and an explicitly supplied upright-local point B. `LinkDifferential::pose` returns the immutable converged pose used by the map. With `r = R*B`, `delta = t+r-A`, length `l = |delta|` and axis `n = delta/l`, each length gradient is `dl/dq_i = n dot (T_i.translation + T_i.angular cross r)`. The sample uses the full point velocity, including upright rotation; `ldot = gradient dot qdot`. This supports direct upright attachments, including an explicitly mounted telescoping strut. Springs attached to moving control arms or rocker/pushrod mechanisms require their own attachment kinematics.

The scalar law and sign conventions are shared with `SpringLink` through one internal sampling path. Signed tension is `s = k*(l-l0) + D(ldot)`, upright point force is `-s*n` and upright-origin torque is `r cross force`. The differential map projects this load to `Q_i = -s*dl/dq_i`. Elastic energy is `U = 0.5*k*(l-l0)^2`; dissipated power is `D(ldot)*ldot >= 0`. Therefore `Q dot qdot = -dU/dt - dissipated_power`. The linear spring is bilateral. Damping remains the previously tested continuous asymmetric slow/fast law; no AC-specific nonlinear rule or unit conversion is inferred.

The direct frame reaction is `s*n` at A with moment `A cross (s*n)` about the fixed frame origin. This excludes rod/coaxial support reactions and the abstract travel actuator. The CLI separately solves ideal held support reactions for the spring's upright load; its existing externally supplied load probe is preserved as a separate calculation. There is no automatic addition of either probe to the native body integrator.

Sampling validates mounts, law, separation, rates and projected loads before returning. It does not mutate the map or advance state. `SpringLink::loads` still returns world-frame body wrenches, while `LinkSpringSample` explicitly names fixed-frame and upright-local quantities; their torque origins are not mixed. Source spring/damper records remain inspection data until mounting, units, preload, setup and motion conventions have been established. Original-game spring behavior, unsprung inertia/contact and dynamically constrained chassis coupling remain unvalidated.

## Fixed-frame upright dynamics

Both geometry types now expose analytical relative accelerations for explicit travel/rack rates and accelerations. The twice-differentiated system is `J_scaled*[a, L*alpha] = C*qddot - bias`. For a distance constraint, with current endpoint lever `r`, unit rod direction `n`, length `l` and relative endpoint velocity `u`, its bias is `n dot (omega cross (omega cross r)) + (u dot u - (n dot u)^2)/l`. The steering rod includes rack velocity in `u` and rack acceleration on the right-hand side. For a STRUT perpendicular normal `n = R*n_local` and fixed top A, bias is `(omega cross (omega cross n)) dot (A-t) - 2*(omega cross n) dot v`. Travel has zero bias. A point acceleration includes `a + alpha cross r + omega cross (omega cross r)`. No finite-difference derivative is used in these solvers.

`TravelSystem` restricts dynamics to fixed frame and rack, leaving travel h as one free coordinate. It owns the geometry, explicit COM mass/tensor, gravity, up to sixteen mounted springs and a validated travel state. At each pose, the travel tangent supplies COM gradient `V = T_translation + T_angular cross (R*COM)` and angular gradient W. Effective mass is `M = m*(V dot V) + W dot (I_frame*W)`. The body-local COM tensor rotates into the fixed frame at the solved orientation.

A zero-control-acceleration solve at the current travel rate gives convective COM acceleration `a0` and angular acceleration `alpha0`. The projected inertial bias is `b = m*V dot a0 + W dot (I_frame*alpha0 + omega cross (I_frame*omega))`. External, point-gravity and mounted spring loads are added about the upright origin and projected to Q. The equation of motion is `M*hddot + b = Q`. This is a one-coordinate reduction of rigid-body Newton/Euler dynamics; the mass and velocity-product structure follows the standard [Lagrangian formulation](https://modernrobotics.northwestern.edu/nu-gm-book-resource/chapter-8-1-lagrangian-formulation-of-dynamics-part-1-of-2/). There is no global four-coordinate rack mass solve: rack motion is prescribed/fixed in this subsystem.

After solving hddot, a second acceleration evaluation gives actual COM acceleration and alpha. The inertial force is `m*a_COM`; moment about the upright origin is `I_frame*alpha + omega cross (I_frame*omega) + (R*COM) cross (m*a_COM)`. Applying the existing constraint transpose solve to applied-minus-inertial loads recovers support reactions. The auxiliary travel multiplier is checked against a 1e-7 load-scale tolerance and represents only a numerical residual. Direct spring-anchor reactions remain separate from linkage supports.

`step` uses explicit midpoint on `[h, hdot]`: evaluate current acceleration, predict a half-step state, evaluate its acceleration, then form the full state. Geometry, velocity, loads, inertia and reactions are validated at all three states. The final state and pose commit together only after the final solve; any error preserves both. Spring addition also validates the resulting zero-external-load sample and rolls back on failure. Returned step samples describe the final state, unlike `step_springs`' midpoint force samples.

The workbench reads at most 16384 bytes of strict schema-1 JSON and limits iterations/records before running. It buffers the completed report, so rejected runs print no partial data. All masses, COM/tensor, gravity and force laws are explicit; the car loader contributes raw geometry only. Moving-chassis behavior is provided by the separate coupled subsystem below. Wheel spin, contact/tyres, side/setup mapping and original-game validation remain requirements for a native vehicle.

## Floating chassis and mounted uprights

`ChassisSystem` owns one rigid chassis and 1..4 `MountedUpright` configurations. Each upright retains its geometry, mass/COM tensor, rack prescription, spring mounts, travel state and solved source pose. The default rack is fixed; explicit moving prescriptions are described below. A proper rigid mount `(d,S)` maps the source linkage frame into chassis-local COM axes. For chassis world COM `(x,R)`, source upright pose `(t,Q)` and local upright COM c, its world COM offset is `r = R*(d + S*(t + Q*c))`, position `x+r` and orientation `R*S*Q`.

Let chassis velocity/rotation rate be v and omega; let relative COM/angular velocities from the source differential be u and w. World upright velocities are `v + omega cross r + R*S*u` and `omega + R*S*w`. For zero generalized accelerations, the convective biases are `a0 = omega cross (omega cross r) + 2*omega cross (R*S*u) + R*S*a_relative0` and `alpha0 = omega cross (R*S*w) + R*S*alpha_relative0`. These terms carry centrifugal, Coriolis and rotating-frame angular transport; independent fixed-frame steps would omit them.

Without wheel rotors, the acceleration coordinates are `[chassis_COM_a_world, chassis_alpha_world, hddot_0, ...]`, with 7..10 entries. Upright COM Jacobian columns are identity for chassis translation, `axis cross r` for chassis angular acceleration and its rotated travel COM gradient for its own travel. Angular columns are zero/identity/rotated travel angular gradient respectively. All other corner travel columns are zero. The chassis contributes its mass and world COM inertia blocks. Each upright adds `m*Jv_transpose*Jv + Jw_transpose*I_world*Jw`. Its convective right-hand side subtracts `Jv_transpose*(m*a0) + Jw_transpose*(I_world*alpha0 + omega_u cross (I_world*omega_u))`; chassis gyro torque is also subtracted.

External world COM wrenches and gravity project through those Jacobians. Mounted spring pairs are internal: their net generalized force on the six rigid chassis coordinates cancels, leaving only each spring's already validated travel load. This retains both spring recoil and global Newton/Euler balance without manually stepping each body multiple times. The equations use the same [standard mass/velocity-product structure](https://modernrobotics.northwestern.edu/nu-gm-book-resource/chapter-8-1-lagrangian-formulation-of-dynamics-part-1-of-2/) as the fixed-frame reduction; implementation and fixtures are original.

The symmetric mass matrix is normalized by `sqrt(M_ii)` on both axes before Cholesky factorization. Active diagonals must lie within 1e-12..1e24, normalized pivots must exceed 1e-10, symmetry is checked to 1e-10, and the unscaled solution must satisfy a 1e-8 row-relative residual gate. Invalid or poorly conditioned systems fail explicitly. There is no diagonal-mass approximation or fallback uncoupled step.

Solved accelerations recover actual upright COM/angular acceleration. Applied-minus-inertial wrenches, transformed back into the instantaneous source frame about the upright origin, recover rod/coaxial reactions through the existing transpose solve. The travel multiplier is checked as a near-zero residual. Support and direct spring reactions are transformed to world moments about the chassis COM and independently checked against its Newton/Euler balance. Reports distinguish these moments from source-origin support moments and total angular momentum about the world origin.

The integrator predicts chassis COM, velocity, world angular velocity and travel coordinates to the common midpoint, updates orientation by left-multiplied quaternion exponentials, then evaluates the full coupled system there. Final velocities/positions/travel use the midpoint derivative and final orientation uses its world angular velocity. All final bodies, source branches, loads, mass solves and reactions validate before the whole system commits. Returned samples describe final states. This explicit second-order method has finite-step momentum/energy error and no universal stiff-system stability guarantee.

The application shares strict spring configuration parsing with the fixed-frame workbench. Coupled JSON is bounded to 65536 bytes, four sources, sixteen springs per upright, ten thousand steps and 256 records; car paths resolve relative to the config file. Synthetic mounting positions/rotations and masses remain explicit. Optional wheel-spin DOFs are described below. Moving racks, contact/tyre laws, automatic AC frame/setup mapping, native animation/control integration and original-game validation remain incomplete.

## Axisymmetric wheel rotors

An optional `Wheel` is a separate massive rotor attached to an upright at an explicit local COM and unit axle a. Its tensor in upright-aligned axes is `I = I_transverse*identity + (I_axial-I_transverse)*a*a_transpose`. The existing mass/tensor validator enforces bounded positive mass, positive moments and physical principal-moment inequalities. Upright mass/tensor excludes the wheel, so no locked-wheel inertia is counted twice. Inertia about the wheel COM remains distinct from translation of that COM.

Let upright world orientation/angular velocity be Q and omega_u, relative spin angle/rate be theta/s, and world axle `n = Q*a`. Rotor orientation is `Q*Exp(a*theta)` and angular velocity is `omega_w = omega_u + n*s`. Axisymmetry makes its world inertia independent of theta. The world angular acceleration is `alpha_w = alpha_u + omega_u cross (n*s) + n*sdot`. This transport term, together with `omega_w cross (I_world*omega_w)`, provides gyroscopic reaction as the upright/axle precesses. The rotational inertia and Newton/Euler structure follow the primary [rigid-body dynamics reference](https://modernrobotics.northwestern.edu/nu-gm-book-resource/8-2-dynamics-of-a-single-rigid-body-part-1-of-2/); implementation and fixtures are original.

The rotor COM uses the same moving-frame point velocity/acceleration machinery at its own attachment point. Its mass and angular Jacobians contribute to the shared matrix just like a rigid upright, with an additional spin column: zero COM translation and world axle n for angular acceleration. Relative spin coordinates follow all travel coordinates. The mass solve now supports 7..14 active dimensions, retaining normalized Cholesky gates and full coupling instead of a scalar wheel-inertia approximation.

External wheel wrenches are world COM wrenches. Internal wheel/upright motor torque enters only its spin right-hand side: the equal/opposite rigid-body contributions cancel under a common chassis/upright virtual rotation. Motor power is torque times relative spin rate. This is an explicit internal actuator, not automatic AC driveline or brake modeling. The spin equation checks `n dot (I_world*alpha_w + omega_w cross (I_world*omega_w) - external_torque) - drive_torque` against the same scaled reaction tolerance.

Wheel reaction on the upright is external-plus-gravity force minus wheel inertial force. Combined bearing/motor moment about the upright origin is external wheel COM torque minus wheel inertial COM torque plus the wheel-COM lever crossed with that reaction force. Bearing and motor moments are also reported separately; their drive components cancel in the combined action/reaction transfer. These wheel reactions join springs/external loads before recovering rod/coaxial supports. System totals include each wheel's COM mass, orbital-plus-spin momentum and kinetic energy exactly once.

The common midpoint predictor advances wheel angle/rate with all other states, reconstructing phase modulo a revolution only for the body quaternion. Final-state/rate/pose/load errors preserve the entire prior system. Legacy `sample`/`step` calls supply zero wheel loads; explicit `sample_with_wheels`/`step_with_wheels` take one wheel-load slot per upright and reject a nonzero slot without a wheel. Schema-1 JSON accepts optional wheel objects; missing/null disables them. Finite-road support, tangential contact and prescribed moving racks are added below; source-calibrated drivetrain/brake laws and renderer integration remain work ahead.

## State-dependent forces and normal support

`ChassisSystem::kinematics` computes current world chassis/upright/wheel body states from the source poses and relative velocities without forces, accelerations or a mass solve. The snapshot contains the same body kinematics used by the coupled solver. A provider returns `ChassisLoads` with exactly one upright and wheel-load slot per corner; wheel loads can transactionally accumulate an external COM wrench without changing their internal drive torque.

`sample_with_forces` evaluates its provider at offset zero. `step_with_forces` invokes it against the current, predicted midpoint and predicted final kinematics at offsets 0, dt/2 and dt, then uses the same coupled matrix/reaction path. The final call supplies diagnostics/validation, not another integration advance. Existing constant-load methods retain their direct path through the shared transactional integrator. Any failed force stage prevents commit of chassis, source poses, travel and spin. Providers should keep evaluation separate from committing their own history: captured side effects are not automatically rolled back.

`new_with_forces` validates the initial sample with the provider's actual loads; it does not first require a physically irrelevant unloaded/preloaded acceleration to remain within bounds. The legacy constructor retains its zero-external-load check. Both paths use the same topology, kinematics, normalized mass and reaction validation. A balanced high-preload fixture distinguishes these initialization contracts.

For a static plane point P/unit normal n, wheel COM C/unit axle a and radius R, let `u = n - a*(a dot n)`. The support lever is `r = -R*u/|u|`, wheel point `C+r` and gap `g = n dot (C+r-P)`. This is a circle support function, including camber; it is not a sphere or finite-width tyre. `|u| < 1e-6` fails as an unsupported axial degeneracy. Gap velocity is `vn = n dot (v_COM + omega_wheel cross r)`. Motion of the geometric support point contributes no extra term along n; axial spin likewise has zero normal contribution.

Penetration is `delta = max(0,-g)`. For `g <= 0`, normal load is `N = max(0, k*delta-c*vn)`; otherwise N is zero. World force is `N*n`, with COM torque `r cross (N*n)`. The plane point is the wheel point projected onto the plane; applying a purely normal force at either point has the same moment. This is an original unsmoothed unilateral penalty law. The spring/damper contact concept is described in the primary [contact-force reference](https://www.mathworks.com/help/sm/ug/modeling-contact-force-between-two-solids.html); this implementation does not reproduce that tool's transition/smoothing rules or original AC tyres.

Stored contact energy is `U = 0.5*k*delta^2`. Mechanical normal power is `N*vn`. In the active unclamped branch, dissipation is `c*vn^2`; when force is clamped during separation it is `k*delta*vn`. Both are nonnegative and satisfy `mechanical_power + dU/dt + dissipation = 0` away from activation boundaries. Clamping therefore introduces no attractive force or energy generation. At first touch, closing damping can act at zero penetration; above the plane there is no damping force.

The application adds this wrench to the configured external wheel wrench at every force stage, preserving drive torque and applying gravity only in the coupled solver. Schema-1 inputs pair an optional static plane with per-wheel normal models. Reports keep core gravity/suspension energy unchanged and add record-level contact potential/total mechanical energy and dissipation. No contact state/history is integrated by the provider.

The explicit penetration limit is checked before returning a force. Excessive penetration or invalid circle/plane geometry aborts with no projection/fallback. This gives a tested compliant-support primitive, not CCD, hard collision, body scraping or a calibrated tyre. The finite road-face provider below reuses it; later layers add tangential forces and native driving/window integration. Edge/vertex response and original tyre calibration remain incomplete.


## Finite road-face normal support

`asc-ground::Ground::faces_in_box` traverses the physical triangle BVH in three dimensions. Query centre components are finite within +/-1e7 m; half extents lie within 0..20 m. Triangle AABB overlaps use a 1e-8 m tolerance. A query errors above 4096 overlapping candidates instead of truncating; returned faces are sorted by source triangle index and borrow mesh/model/surface metadata. `Face::contains_point` checks plane distance within 1e-7 m and signed XZ edge distances within 1e-8 m for either winding. Existing ground loading excludes vertical/XZ-degenerate faces.

`asc-session::face_contact` queries a cube of half extent `R + max_penetration` about the wheel COM. It filters upward geometric normals by an explicit `min_normal_y` in 1e-4..1, computes thin-circle geometry without a force/penetration check, then clips the projected plane point to the finite triangle. This ordering prevents an unrelated infinite plane extension from triggering excessive penetration. A wholly overhead face is ignored; overlap from below is diagnosed as unsupported backside response. Among eligible front faces it selects the smallest gap; ties within 1e-9 m use source order. Exactly one normal wrench is applied, including on a shared seam. No eligible face means airborne; no flat support is invented.

The selected plane then uses the existing normal law and penetration limit. This models face interiors only: the closest edge/vertex contact is not solved, and a step can tunnel through geometry without CCD. Normal changes at seams are unsmoothed; discontinuous contacts have no general energy-continuity guarantee. Surface damping does not replace explicitly configured normal stiffness/damping. Surface friction scales the optional tangential law below; with that provider disabled it remains metadata. Reported provenance omits the full arbitrary surface parameter map to avoid duplicating it per wheel/record.

Schema-1 `chassis-run` pairs wheel normal models with either `support_plane` or `support_track`. The latter loads explicit model and ordered surface-file paths relative to the JSON. Optional `initial_spawn` selects a unique supported marker and an explicit COM offset in its left/up/forward basis, with the existing stricter rigid-basis validator. Zero chassis position and identity orientation are required in spawn mode; configured world velocities remain world velocities. Marker height is not inferred to be equilibrium. Source diagnostics remain in the output, including unrelated duplicate markers.


## Tangential contact and wheel traction

`asc-physics::TangentialContact` is an original memoryless slip-velocity law used with `NormalContact::sample_with_friction`. It provides a bounded dissipative force for native traction development. It does not implement static stiction, transient brush deformation, load-sensitive grip, temperature/pressure/wear, aligning trail or an original AC tyre law. More developed handling tyre families and transient slip state models are described in the primary [Project Chrono tyre documentation](https://api.chrono.projectchrono.org/wheeled_tire.html); no Chrono code or parameter set is used here.

For axle a and support normal n, longitudinal direction is `l = normalize(a cross n)`, lateral direction `b = n cross l`. Material-point velocity is `v = v_COM + omega_wheel cross r`, using the existing thin-circle lever r. Slip components are `s_l = v dot l`, `s_b = v dot b`. Raw components are `f_i = -C_i*s_i`, with nonnegative explicit viscosities C. Directional limits are `L_i = mu_i * surface_scale * N`, using the actual clamped normal load N. A zero limit disables that component. Remaining components share `q = hypot(f_l/L_l, f_b/L_b)` and a common scale `1/max(1,q)`, so combined force stays within one anisotropic ellipse. There is no slip-ratio denominator or division by vehicle speed.

World force is `F = f_l*l + f_b*b`; wheel COM torque is `r cross F`. The material wheel point is used, not its projection onto the plane: tangential forces at these separated points would give different torques during normal compression. World wrench power equals `f_l*s_l + f_b*s_b <= 0`; its negative is reported dissipation, with no additional stored tangential energy. At zero slip the law produces zero force and cannot hold a slope with static stiction. Motor power is the separate internal pair `drive_torque * relative_spin_rate`; adding the external contact moment to the wheel lets the coupled solver transmit traction and bearing reactions to the chassis.

Viscosities are finite in 0..1e7 N s/m; directional friction coefficients and surface scale are finite in 0..10. Existing wheel geometry/velocity/wrench bounds apply. Nonfinite ellipse utilization is diagnosed rather than silently converted to zero force. Explicit midpoint integration still needs a step appropriate to the slip relaxation timescale; force passivity alone does not imply arbitrary-step numerical stability.

`asc-session::face_contact_with_friction` preserves finite-face selection and passes the selected surface FRICTION as the scale. No selected face means neither normal nor tangential load. The CLI may also use a plane with an explicitly configured scale. Each force stage adds both COM wrenches to the wheel while preserving its internal motor torque. Record-level dissipation includes the tangential contribution; existing gravity/spring/normal stored energies are unchanged.


## Scheduled wheel actuation and passive brakes

`asc-physics::WheelBrake` samples a bounded regularized torque between wheel and upright. For maximum torque M, brake input b, transition rate w0 and relative spin w, `T = -b*M*clamp(w/w0,-1,1)`. Its internal mechanical power is `T*w <= 0`; reported heat-equivalent dissipation is `-T*w`. The opposite scalar torque acts on the upright along the same signed axle. Absolute world wheel rotation is not substituted for relative spin: a stationary rotor relative to a rotating upright has no brake slip or loss.

Maximum torque is finite in 0..1e6 N m, transition rate in 1e-3..1e3 rad/s, input in 0..1 and relative speed within +/-1e5 rad/s. Near zero the law is viscous with coefficient `b*M/w0`; it has no static locking constraint and does not guarantee an exact finite-time stop. At high rate it is a bounded opposing torque. There is no original-game calibration, hydraulic state, ABS, brake temperature or fade model.

`WheelLoad` exposes validated internal-torque set/add operations, preserving its external world COM wrench. Rejected changes leave the previous torque intact; bounded combined torque uses the existing +/-1e18 N m load limit. The coupled solver already applies the motor/upright reaction pair and now receives the sum of commanded motor and brake torque through that same coordinate. Brake loss is aggregated by the provider, outside core spring/damper dissipation, so it is counted once.

The workbench's bounded `Timeline` holds 1..64 strictly ordered keys starting at zero. It interpolates signed drive and unsigned brake fractions linearly, holds the final key, and rejects invalid numeric/time ranges. At absolute step start t, force callbacks sample `t`, `t+dt/2` and `t+dt`; initial construction/reporting uses time zero. The optional provider is enabled for controls or brakes even without road contact. Records report applied fractions, brake samples and separate motor input power. Constant examples with neither controls nor brakes retain their old path.

The timeline is a repeatable actuator input for development. It does not implement keyboard/gamepad sampling, source engine/clutch/gearing or an original brake system. Rack steering uses the separate prescription below. Scheduled input corners are continuous but have derivative changes; midpoint integration remains subject to timestep convergence and the existing transactional stage checks.


## Prescribed rack dynamics and steering

`RackMotion` supplies source-frame position, velocity and acceleration for a massless steering anchor. Component bounds are +/-0.25 m, +/-1000 m/s and +/-1e6 m/s^2; the linkage's existing local-branch/conditioning bounds still apply. `MountedUpright::set_rack_motion` configures an initial or diagnostic prescription after geometry/spring validation. It does not integrate a rack coordinate. Each corner has its own explicit source-frame prescription, with no automatic right-side reflection, Ackermann, steering-wheel ratio or setup inference.

The differential velocity now uses all four controls `[travel_rate, rack_vx, rack_vy, rack_vz]`. Analytical relative acceleration includes rack acceleration and all travel/rack velocity bias terms. World bias still includes rotating-chassis transport and Coriolis terms. The existing 7..14-coordinate mass solve keeps chassis, travel and spin free; prescribed rack derivatives enter known velocity/acceleration terms, not new unconstrained degrees of freedom. Spring extension rates include the induced upright motion. Wheel gyroscopic terms use the updated total upright angular velocity.

Applied-minus-inertial upright/wheel loads pass through the same massless constraint reaction solve. The required source-frame rack actuator force is the negative of `steering_anchor_force_n`, and its power is that actuator force dotted with source-frame rack velocity. This includes spring and wheel reactions. It is an internal actuator: chassis reaction remains in the global Newton/Euler balance, while rack work can add or extract mechanical energy. Negative rack power is energy returned to the ideal actuator, not dissipation. Per-upright reports expose `rack_motion`, `rack_actuator_force_in_source_n` and `rack_actuator_power_w`; the chassis sample sums actuator power.

`sample_with_racks_and_forces` provides an instantaneous diagnostic prescription without changing the system. `step_with_racks_and_forces` samples rack callbacks before force snapshots at offsets 0, dt/2 and dt. Start position/velocity must match committed values within 1e-10 m and 1e-8 m/s; acceleration may jump without an impulse. The whole chassis/upright/wheel/rack state commits only after final validation. Callback-owned side effects remain outside this atomic guarantee. Legacy step calls reject nonzero stored rack velocity/acceleration and require the prescribed-rack API instead; fixed-rack calls retain their behavior. Caller-supplied position/velocity/acceleration functions must describe a consistent path.

The CLI uses a bounded cubic Hermite scalar displacement path along an explicit unit source-frame axis, added to each upright's `rack_m` base. Keys contain time, displacement and velocity. Cubic coefficients give analytical velocity/acceleration. Validation checks interior position and velocity extrema plus endpoint acceleration, rather than only key values. Scalar displacement/velocity/acceleration use the same bounds above; combined base/axis positions and linkage geometry validate at sampled stages. Keys start at zero, increase by at least 1e-6 s, remain within 1e6 s and number 1..64; initial/final velocities are zero. Final displacement is held with zero derivatives. Velocity is continuous; acceleration may change at keys. A live input steering actuator is not implemented by this offline prescription.


## Native driving window

`asc-app` now separates workbench construction from offline report stepping. The same `Workbench` owns the coupled physical system, source configuration and finite-face/contact providers for both `chassis-run` and `drive`. The interactive path overrides scheduled drive/brake fractions without replacing the force law. Each successful tick advances its own time; rejected inputs/steps preserve mechanical state and tick count. Reset restores a cloned initial system without reloading source assets.

Live steering uses each configured source axis and current committed rack position/velocity. For scalar target p*, current p/v and frequency w, desired acceleration is `w*w*(p*-p)-2*w*v`, capped explicitly. The endpoint velocity is bounded by selecting a compatible acceleration, not by jumping velocity. Over one physics step the rack follows a quadratic position with its derivative velocity and constant acceleration. This prescription passes through `step_with_racks_and_forces`, retaining inertial/Coriolis/gyroscopic terms and actuator work. The displacement setting bounds the input target; it is not a kinematic position projection or a calibrated steering mechanism.

A winit event loop owns the native surface, shared renderer, orbit camera and key state. Physical key codes feed one handler; diagnostic input uses that same handler. Pause/focus/visibility/resize clears held commands, and autorepeat cannot re-enable cleared pedals. Motor direction changes while paused. A bounded `FixedClock` supplies a reciprocal-integer physics rate independent of redraw timing. Up to 128 ticks execute per advance. A quarter-second backlog pauses instead of silently throwing away simulation time. Physics errors retain successful earlier ticks and pause at the last valid state; pending wall-clock debt is reset only as part of pausing.

The car uses one or more disjoint draw-range roots. Its physical COM pose composes an explicit model-in-body correction and source graphics correction; track geometry remains static. Optional articulation partitions the car draw range into selected source subtrees and intervening chassis ranges. Persistent GPU resources are reused, including camera-dependent transparent sorting. Following translates only the orbit target. Capture readbacks and diagnostic motion hashes are taken from the native surface. Surface loss/retry handling is bounded to three attempts. Suspend/resume preserves physics while rebuilding window resources, starts paused and refits the camera around the current car position; device-loss and general platform recovery remain unvalidated.

This first live-control window deliberately exposes the existing independent synthetic workbench parameters. It adds no original engine/clutch/gearing, calibrated tyre law or native body/wall collision. Explicit source subtree articulation is described below. It establishes a path from key command to coupled force solve to rendered vehicle pose, with validation scope stated separately.

## Native visual articulation

`asc-kn5::load_hierarchy`/`parse_hierarchy` retain validated source names, parent links, world transforms and contiguous visible draw ranges for each subtree in the same traversal that produces ordinary flattened geometry. Node-only mode still retains no render geometry and reports empty ranges. Inactive/hidden/unsupported geometry does not become visible through articulation. The existing node, string, depth and geometry limits remain in force.

The application accepts explicit `node`/`upright`/`kind` bindings, validates uniqueness and disjoint visible ranges, then partitions every car draw exactly once. Unselected gaps follow the chassis. Selected wheel groups follow the full rotor orientation; upright groups follow the nonspinning body orientation. Both use physical wheel-centre travel, so upright COM offsets cannot displace a source hub-centred suspension pivot. A physical wheel is required for either binding kind. No naming heuristic, material change, GPU vertex rewrite or per-frame asset loading is involved. Texture-source namespaces remain attached to their original material ranges. Diagnostics enumerate each root, its draw range, binding and corrected source pivot.

Let B be the chassis pose, C the explicit model/graphics correction, L = inverse(B) * physical_group the current physical group pose relative to the chassis, L0 its initial value and p the source node origin transformed by C. Define D = rotation(L) * inverse(rotation(L0)) and d = position(L) - position(L0). The GPU root is B * [translation(p + d - D*p), rotation(D)] * C. Thus the original model pivot maps to B*(p+d), while points about it rotate by D. Original rest placement is preserved even when the synthetic physical centre differs from the mesh pivot. Spin cannot turn that difference into a visual hub orbit. Recomputing these deltas from current committed state also preserves reset and window recreation behavior.

This is a source-pivot visual mapping with physical deltas, not calibrated AC frame/setup semantics or proof that displayed wheel centres coincide with solved tyre contacts. Nonrigid source mesh content keeps its original flattened form; skeletal/deforming animation, source animation files, blur meshes and dynamic LOD switching remain unsupported.

## Source wheel-centre registration

The coupled workbench accepts either an explicit frame translation or an explicit source corner reference per upright. The shared typed vehicle reader supplies the corner footprint and selected compound radius; raw suspension geometry still comes from the suspension reader. A corner must match its explicit axle, and canonical car paths prevent selecting the same source corner twice. The registered wheel may have a nonzero local centre and nonzero initial travel/rack.

Let c be the typed source wheel centre in chassis coordinates, R the caller's source-frame-to-chassis rotation, P0 the solved initial raw upright pose and w the configured wheel centre in upright coordinates. The frame translation is c - R*(P0*w). This is a one-time rest registration: subsequent wheel positions come entirely from the coupled dynamics. The initial physical centre agrees with the selected footprint even when initial travel or a wheel lever is nonzero. Frame orientation remains explicit; the method does not infer reflections or original setup transforms.

A source reference also supplies the optional normal model's radius, with an exclusive choice between source and numeric radius. Resolved values, compound and field-source diagnostics are kept alongside the original explicit configuration. The chassis mass/inertia and all spring/damper/traction/motor/brake laws are unchanged. This prevents treating total source mass or HUB_MASS as automatically assignable to the independently massive chassis/upright/rotor model.

Renderer diagnostics compare the corrected source node pivot with the initial physical wheel centre in chassis coordinates. Animation preserves the model pivot while applying physical deltas, so residual alignment remains measurable instead of being silently repaired. Typed footprint agreement alone does not validate model-space/COM semantics, original tyre contact alignment or original-game dynamics.

## Source reflection and handedness

The session adapter exposes `dwb_geometry_with_reflection` and `strut_geometry_with_reflection`. Existing raw adapter functions delegate with reflection disabled, preserving their behavior. Reflection copies each named point with X negated; DWB uses all five rod anchors/attachments, and STRUT also reflects its fixed top and upright line point. The solvers reconstruct constraints from these coordinates and an identity design pose. No determinant-minus-one pose enters the rigid-body or renderer APIs. Caller-supplied travel/rack controls are in the resulting frame.

Under S = diag(-1,1,1), a reflected proper pose has p' = S*p and R' = S*R*S. Linear velocity, acceleration, force and steering-anchor displacement transform by S. Angular velocity/acceleration and torque transform by det(S)*S = -S. Local inertia transforms by S*I*S. Applying S directly to an angular velocity or torque would change the mechanism's handedness and break virtual work; a Y half-turn would also reverse source Z.

The coupled JSON flag applies only to raw source hardpoints. All explicit synthetic inertia/COM/spring/wheel/frame inputs are retained as written. This keeps their frame interpretation reviewable instead of applying an inferred vehicle-side conversion. The source corner reference still independently registers the initial wheel centre to its typed footprint. The BMW example now selects reflection on the left based on the local chassis-anchor direction, identity frame rotations on both sides, explicitly mirrored synthetic COM/tensors/spring endpoints, and a common wheel axle/motor convention. It does not import original mass, preload, camber/toe or setup force semantics.

## Coupled impulse response

At fixed configuration the instantaneous operator solves `M * delta_velocity = B` in the existing 7..14 generalized coordinates: chassis world linear/angular velocity, free upright travel and optional rotor rates. A chassis-point impulse J supplies B = [J, (point-COM) cross J, zero travel/spin inputs]. The mass matrix includes all bodies, COM coupling and axisymmetric rotors; rank updates and the normalized Cholesky solve are shared with force dynamics. Geometry-derived Jacobians are verified against the established continuous force operator.

Gravity, spring/damper force, acceleration bias, finite actuator torque, time integration and position repair do not enter a zero-duration impulse. A cloned candidate receives the solved velocity increments, increments chassis momenta at its exact retained pose and validates all body/rate/kinematic bounds before commit. Mechanical positions/angles and rack prescriptions remain unchanged. No force-provider callback runs. The unit-direction query projects the chassis-point Jacobian through inverse mass. A one-contact restitution impulse follows `j = -(1+e)*vn/k`; simultaneous inelastic contacts use the separate solver below.

With prescribed rack motion, physical velocity is affine in free rates: `v_body = J_free*u + v_rack`. External impulse work is J dotted with the mean before/after chassis-point velocity. Rack work is independently summed over upright/rotor bodies as `v_rack dot (mass*delta_v) + omega_rack dot (I_world*delta_omega)`. The reported residual is kinetic-energy change minus both works. The ideal rack drive can supply or recover energy while its velocity stays prescribed. Global linear/angular momentum still changes only by the external impulse and its world-origin moment.

This API acts on a chassis point only. It selects no collider, triangle feature, normal or TOI, and provides no nonpenetration or wheel/upright-targeted external impulse. The native driving window remains disconnected from body-collision detection/response pending that integration.

## Native WALL contact geometry

`asc-ground::WallIndex` owns static WALL triangles and source metadata separately from upward support. It uses the validated `asc-scene` declarations and `asc-kn5` physical selection, retaining hidden active numeric-prefix static WALL meshes without requiring a surface KEY. Positive near-LOD/skinned geometry remains unsupported. A bounded full-3D BVH uses the longest of all three extents; the support tree retains its historical X/Z split policy. Both paths share candidate traversal with their existing error codes and exact overflow behavior.

`OrientedBox` validates finite half extents and a proper quaternion, computes world AABBs and transforms a finite triangle into box coordinates. SAT tests the triangle normal, three box axes and nondegenerate edge cross-products. Axis signs/order are canonicalized for winding and tie stability. The maximum normalized interval separation selects a normal directed toward the box. This is exact static overlap classification within numeric tolerances; a positive SAT margin is conservative and is not an exact closest-distance query.

Sutherland-Hodgman clipping intersects the triangle with the box's six margin-expanded planes. Returned points remain on the finite source triangle, not an extrapolated plane. Small duplicates are merged and patch storage is bounded. The margin uses a SAT threshold while clipping expands box axes; those operations intentionally provide a conservative patch and must not be presented as a spherical collision skin. World range, quaternion, degeneracy and patch/candidate limits are validated. No source surface friction or restitution is guessed.

`asc-session::ColliderBox` loads a confined, bounded collider file, rejects skinning, and bounds referenced supported vertices after explicit model/graphics correction. Its enclosing chassis-frame box transforms with a fully oriented chassis pose. `wall-query` composes optional source spawn and explicit probe poses, retains provenance and builds the entire bounded JSON result before printing it. It advances no mechanics.

These spatial queries supply finite contact patches for subsequent coupled impulse/manifold work. They are not yet connected to native driving. A complete driving integration still needs continuous motion/TOI handling and validated nonpenetration; source convex shape/setup semantics and original-game collision parity remain open.

## Simultaneous chassis contacts

`ChassisSystem::resolve_chassis_contacts` accepts up to 64 world points, unit normals and explicit isotropic Coulomb coefficients. It holds the complete chassis/suspension/rotor configuration and prescribed racks fixed. The shared normalized mass factorization is computed once. Six unit force/moment solves condense inverse mass to a reciprocal 6-by-6 chassis mobility; each point's three wrench rows project that mobility to normal and two tangent directions. The full contact matrix need not be invertible, so redundant points remain admissible.

Projected Gauss-Seidel updates nonnegative normal impulse and a two-dimensional friction disk. The tangential update minimizes kinetic energy in the local anisotropic mobility metric. When the unconstrained tangent impulse lies outside the disk, a bounded scalar search solves `(K_t + alpha I) * lambda_t = b` at the disk radius. Radial clipping of the unconstrained impulse would give an incorrect oblique sliding direction. Convergence checks normal complementarity and tangent sticking/sliding optimality in velocity units, with a bound for negligibly small friction budgets. Contact order can change a nonunique distribution of redundant impulses; unique per-point impulses are not promised.

After convergence the aggregate force/moment impulse is applied through one full generalized solve to a cloned candidate. This updates free travel/spin as well as chassis rates. Actual committed-candidate point velocities are checked again; static-world external impulse work must be nonpositive within roundoff. Physical kinetic energy can also exchange with a prescribed moving rack, whose independent work is reported separately. Only a completely validated candidate is committed. Nonfinite/ill-conditioned inputs, bounds failures and nonconvergence leave the original system unchanged. Default settings are 128 sweeps and 1e-7 m/s; convergence is not universal, particularly for nearly parallel facets. No silent fallback or partial manifold is used.

`wall-impact` connects this operator to `ColliderBox` and `WallIndex` at the dynamics workbench's initial pose. It retains source patches and uses all clipped points, merging only matching point/normal pairs, rejecting more than 64 distinct points. It builds the whole JSON report before stdout. Initial force preflight is retained, but the impulse operation itself invokes no force provider and advances no time. Positions and overlap are unchanged. This static adapter does not establish continuous body collision in the native driving window; event timing/TOI, penetration handling and robust motion integration remain separate work.

## Prescribed rigid-path sweeps

`BoxMotion` describes a linearly translating reference plus constant-speed shortest-arc quaternion rotation. A collider's local centre offset rotates with that reference. Relative quaternion sign and the pi tie are canonicalized; an exponential rotation map avoids the variable angular speed of normalized linear interpolation. The radius bound is `length(local_centre) + length(half_extents)`. The segment of reference positions expanded by this radius encloses every box point at every fraction, including rotation with clear endpoints.

`WallIndex::sweep` obtains at most 4096 candidates from this entire swept envelope once. At each fraction, the true maximum normalized SAT gap for each finite triangle is computed using the same axes as static overlap. The historical deterministic normal tie rule remains separate from the true maximum gap. A positive gap is a lower bound on distance. Holding its separating axis fixed, no point can close that gap faster than `length(reference_end-reference_start) + rotation_angle*radius` per unit fraction. The minimum candidate gap therefore supplies a conservative advance toward the requested spatial tolerance. This checks intermediate rotation, not just a hull of endpoint boxes.

At a proximity event the ordinary static query returns finite clipped patches with source provenance. A nonmoving clear shape or a safely reached clear endpoint returns no event. Initial overlap returns fraction zero. Iteration exhaustion, nonfinite/stalled advancement, invalid bounds and candidate/report overflow fail explicitly; no unresolved path becomes clear. Settings bound computational work but do not guarantee convergence for all grazing or nearly degenerate paths. SAT proximity is conservative and does not supply an exact Euclidean distance or exact time of geometric contact.

The application shares proper spawn-basis validation with static queries, preserves source diagnostics and builds all motion records before stdout. This path query has no mechanical state or collision response. Coupling it to a force-integrated chassis still requires controlling interpolation error of the actual trajectory, handling initial overlap and repeated/remaining-time events, and applying the coupled contact solver transactionally. Those safeguards remain prerequisites for native driving collision claims.
