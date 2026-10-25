# openASC

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="site/brand/openasc-dark.svg">
  <img src="site/brand/openasc-light.svg" alt="openASC — open circuit and forward arrow" width="520">
</picture>

[Logo source files and usage](site/brand/README.md) · Artwork licensed under [MIT](LICENSE).

An independent open-source engine project for **Assetto Corsa (2014)** content, inspired by [openOMSI](https://github.com/openOMSI-org/openOMSI).

**Target: 1:1 behavioral compatibility with Assetto Corsa (2014).** This includes vehicle physics, controls and force feedback, visuals, audio, AI, session rules, networking and original content behavior. This is the project objective, not an achieved compatibility claim. The production stack is Rust with wgpu. A first native KN5 reader and offscreen diffuse renderer now run independently of Python and the browser; production native driving remains incomplete. The Python implementation remains a research prototype and tooling foundation.

**Status: native Rust/wgpu keyboard-driving prototype and local browser research path. Original physics parity is not established.** The repository contains local inventory, INI/LUT readers, a bounded KN5 static geometry importer, an interactive local WebGL viewer, original tests, and a documentation website. Supported embedded diffuse textures, alpha test, approximate transparency, typed track surfaces, vertical ground support queries, ordinary ACD car-data reading and geometric spawn placement checks are available. The native window now moves a locally loaded car through coupled 3D chassis/suspension/wheel dynamics, finite road-face normal/tangential forces and smooth rack steering. Its controls use direct synthetic motor/brake inputs. The separate Python browser prototype uses a 120 Hz planar model and WALL contact against an upright collider box. Full native collision, source-calibrated suspension/tyre/drivetrain behavior, audio and FFB remain incomplete.

[Website](https://openasc-project.github.io/openASC/) · [User guide](docs/USER_GUIDE.md) · [Compatibility record](docs/FORMATS.md) · [Architecture](docs/ARCHITECTURE.md) · [Roadmap](docs/FEASIBILITY.md)

## Quick start

For the native path, install Rust with Cargo and the platform's native build tools. Rust 1.96 is tested locally. Rendering needs a wgpu-compatible native GPU/driver. Create `reports/` before saving a capture; existing outputs are never overwritten.

```sh
cargo run --release --locked -p asc-app -- drive examples/native-drive.json
cargo run --locked -p asc-app -- inspect ./assettocorsa/content/tracks/drift/drift.kn5
cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/drift/drift.kn5
cargo run --locked -p asc-app -- render ./assettocorsa/content/tracks/drift/drift.kn5 reports/native-drift.png
cargo run --locked -p asc-app -- car ./assettocorsa/content/cars/bmw_m3_e30
cargo run --locked -p asc-app -- spawns ./assettocorsa/content/tracks/drift/models.ini
cargo run --locked -p asc-app -- suspension ./assettocorsa/content/cars/bmw_m3_e30
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 rear 0.025 0 1 0
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 front 0.025 0 1 0
cargo run --locked -p asc-app -- suspension-run ./assettocorsa/content/cars/bmw_m3_e30 front examples/suspension-travel.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-suspension.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-wheels.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-normal-plane.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-normal-track.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-traction-track.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-accelerate-brake.json
cargo run --locked -p asc-app -- chassis-run examples/chassis-steering-track.json
cargo run --locked -p asc-app -- preview ./assettocorsa/content/cars/bmw_m3_e30 ./assettocorsa/content/tracks/drift/models.ini
```

The native view command opens a desktop window with orbit, pan, zoom and perspective/orthographic cameras. It reuses persistent wgpu resources and the same material pipeline as the PNG path. Pass either one KN5 or an explicit `models.ini`/`models_LAYOUT.ini` path to inspect, render or view a composed static scene. For example, `cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/magione/models.ini` loads both declared models. Embedded diffuse resources remain local to each KN5 and share a bounded scene budget. Supported BC1/2/3 textures now upload directly on capable adapters; --rgba-textures forces the software fallback. The native path now reconstructs masked four-detail base colour for three ksMultilayer variants; original normal/Fresnel/specular lighting remains incomplete. Use `materials SCENE` to inspect retained shader properties and sampler slots. Add `--diagnostic` to skip texture loading. Native INI/LUT readers are available through `ini FILE.ini` and `lut FILE.lut --sample X`. Nonzero model rotations and dynamic objects remain unsupported by static scene loading; the separate drive command is described below. See the [native guide](docs/USER_GUIDE.md#native-static-scene-configuration).

Native `spawns` and `place` provide geometric preparation only. Drift has a duplicate `AC_START_3`, so its reports retain `has_errors: true` and a nonzero process status even when the selected pit footprint has support; see the [placement guide](docs/USER_GUIDE.md#native-spawn-and-placement-preflight).

Native car/track `preview` opens a static inspection session with a separate car root and graphics correction. It reports source-marker errors but does not run native driving physics; see the [session preview guide](docs/USER_GUIDE.md#native-car-and-track-inspection).

Native `suspension-motion` solves DWB or STRUT raw design geometry for an explicit travel axis and optional steering-anchor displacement. DWB preserves five link lengths; STRUT preserves three lengths and a telescoping coaxial constraint. Source frame/setup interpretation and original-game kinematics remain unvalidated. See the [DWB](docs/USER_GUIDE.md#native-dwb-design-geometry) and [STRUT](docs/USER_GUIDE.md#native-strut-design-geometry) guides.

Optional `--rates HDOT RXDOT RYDOT RZDOT` and `--load FX FY FZ TX TY TZ` probes calculate relative upright velocity, virtual-work load projection and ideal massless reactions at a held pose. These explicit diagnostic loads do not generate tyre/spring forces or advance the vehicle. See [velocity and load probes](docs/USER_GUIDE.md#native-suspension-velocity-and-load-probes).

An explicit `--spring AX AY AZ BX BY BZ REST K` mount, with optional `--damping CS CF CK RS RF RK`, connects the native spring/damper element to that mechanism. It reports attachment velocity, length gradients, forces, energy and projected travel/rack loads. Mounting and law parameters are supplied by the caller; automatic AC spring mapping and constrained vehicle dynamics remain incomplete. See [spring probes](docs/USER_GUIDE.md#native-suspension-spring-probes).

Native `suspension-run` advances one massive DWB/STRUT upright on a fixed frame and fixed rack, with an explicit COM/tensor, gravity and spring mounts. The example configuration uses synthetic parameters. It reports constrained motion, energy and dynamic support reactions; its frame stays fixed. See the [dynamic workbench](docs/USER_GUIDE.md#native-fixed-frame-suspension-dynamics) or the coupled workbench below.

Native `chassis-run` couples a floating chassis to one to four massive uprights, solving chassis acceleration/rotation and suspension travel together. It reports spring recoil, dynamic supports, system momentum and energy. Masses, tensors and mounting frames are explicit synthetic inputs in the example; automatic AC side/setup mapping, tyres/contact and native rendered driving remain incomplete. See [coupled dynamics](docs/USER_GUIDE.md#native-coupled-chassis-dynamics).

Optional axisymmetric wheels now add separate mass/inertia and spin coordinates to the same solve. The wheel example exercises internal wheel/upright motor torque, opposite motor reaction and gyroscopic transport. Upright mass/tensor must exclude the separately supplied wheel. These explicit motors do not implement AC drivetrain/brake behavior or road traction. See [wheel rotation](docs/USER_GUIDE.md#native-wheel-rotation-and-motor-reactions).

State-dependent forces are now sampled at each midpoint stage. An explicit compliant thin-circle normal model can support the coupled vehicle on a static infinite plane with gravity, spring preload and damping. The same law now supports finite road-face interiors from the native track BVH, with explicit spawn COM offset and per-wheel face/surface provenance. An optional memoryless tangential slip law now supplies traction and lateral resistance with an anisotropic friction ellipse. Edge/vertex response, CCD, tyre patch dynamics and original-game calibration remain separate work. See [plane support](docs/USER_GUIDE.md#native-normal-support-on-a-plane) and [finite track faces](docs/USER_GUIDE.md#native-normal-support-on-track-faces).

The existing content tools, browser viewer and keyboard-driving prototype require Python 3.10+. No third-party Python packages are needed.

```sh
python tools/inspect_content.py ./assettocorsa
python tools/inspect_content.py ./assettocorsa --output reports/content.json
python -m openasc scene ./assettocorsa --track drift
python -m openasc scene ./assettocorsa --track ks_barcelona --layout layout_gp
python -m openasc view ./assettocorsa --track drift
python -m openasc drive ./assettocorsa --car bmw_m3_e30 --track drift
python -m openasc ground ./assettocorsa --track drift --at 0 100 0 --distance 150
python -m openasc car ./assettocorsa --car bmw_m3_e30
python -m openasc place ./assettocorsa --car bmw_m3_e30 --track drift --spawn AC_PIT_0
python -m unittest discover -s tests -v
```

The scanner reads the immediate `content/cars` and `content/tracks` directories and checks for expected files. It does not launch the game, unpack ACD archives, parse geometry, or copy resources. Reports use content-relative paths and are not saved unless requested. Existing output files are never overwritten.

The `openasc` CLI also provides `ini FILE`, `lut FILE --sample X`, and `kn5 FILE` commands. The `scene` command remains a lightweight configuration/header inspector. The `view` command loads supported static geometry and starts an interactive preview at [127.0.0.1:4180](http://127.0.0.1:4180/). See the [user guide](docs/USER_GUIDE.md) and [supported format subset](docs/FORMATS.md).

The viewer requires a browser with WebGL 2. Drag to orbit, right-drag to pan, scroll to zoom, or select a mesh to focus it. It previews supported embedded txDiffuse textures with basic lighting and approximate alpha behavior. Unsupported resources fall back to diagnostic colours; original shaders are not reproduced. Original resources remain local and are not written into the website or repository.

Your installation may live outside this repository: pass its root path instead of `./assettocorsa`. The `assettocorsa/` and `reports/` directories are ignored by Git.

## Website and documentation

The website follows a documentation-first structure inspired by [openomsi.org](https://openomsi.org/). Documentation, code comments, diagnostics, and commit messages are maintained in English.

```sh
python tools/build_site.py
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Open [localhost](http://127.0.0.1:4173/). The site uses no npm packages, remote fonts, or CDN assets. Documentation pages are generated from `docs/` using Python's standard library. Edit `web/overview.html` for the overview, then regenerate HTML before committing.

Pushes affecting `site/` on the default `main` or `master` branch automatically deploy to [GitHub Pages](https://openasc-project.github.io/openASC/). See the [publishing guide](docs/GITHUB_PAGES.md) and [development guide](docs/DEVELOPMENT.md).

## Repository layout

```text
docs/                     English documentation
web/                      Website template and overview source
site/                     Generated HTML and shared static assets
tools/inspect_content.py  Local content inventory
crates/asc-app/           Native openasc executable
crates/asc-config/        Source-aware INI/LUT readers
crates/asc-acd/           Bounded in-memory car-data archive reader
crates/asc-kn5/           Bounded KN5 format reader and model data
crates/asc-scene/         Static model composition and spawn markers
crates/asc-texture/       Bounded legacy DDS decoder
crates/asc-render/        wgpu GPU work, pipelines, resources, scene and shaders
crates/asc-ground/        Physical surfaces and static support queries
crates/asc-vehicle/       Typed car configuration and suspension records
crates/asc-physics/       Rigid bodies, springs, constrained upright and fixed clock
crates/asc-session/       Vehicle/spawn placement and explicit body seeding
tools/test-support/      Original shared Rust fixtures
tools/asc-telemetry/     Read-only Windows original-game measurement tool
tools/prototype/openasc/ Python research readers and driving model
tools/prototype/viewer/  Research WebGL preview interface
openasc/                 Compatibility entry point for python -m openasc
tools/build_site.py       Documentation site builder
tests/                    Original, game-independent test fixtures
Cargo.toml / Cargo.lock   Native workspace and locked dependencies
.github/workflows/        CI and GitHub Pages
assettocorsa/             Local game installation, ignored by Git
```

The source layout follows the subsystem workspace used by [openOMSI](https://github.com/openOMSI-org/openOMSI/blob/main/Cargo.toml). Each engine subsystem has its own crate, with the executable assembling them in `asc-app`. Python research stays under `tools/prototype/`. See [workspace boundaries](docs/ARCHITECTURE.md#workspace-and-subsystem-boundaries) before adding a subsystem.

The separate Rust [telemetry collector](docs/TELEMETRY.md) reads an existing original-game session on Windows for future controlled comparisons. Its protocol and OS access are fixture-tested; live acquisition and original physics parity remain unverified.

## Next milestone

Typed surface definitions and BVH-accelerated vertical ground queries are available through `ground`. This includes supported hidden road meshes and returns geometric height, an upward normal and source surface metadata. The path is tested on original fixtures and local Drift. Ordinary packed car data, typed vehicle parameters, spawn transforms and four-wheel support preflights are also available. The separate drive command adds independent planar motion with geometric road attachment; it does not reproduce original tyre, suspension or wall behavior.

The separate browser BMW M3 E30/Drift prototype runs an independently implemented local keyboard-driving model with tested acceleration, steering, braking, reverse, pause and reset. Static wall contact now uses continuous translation/yaw queries and inelastic planar impulses, with verified nonpenetration on the tested traces. Native coupled suspension/traction, wheel animation and source footprint registration are described below. Original source calibration, vehicle/skin materials and full convex/6DOF contact remain ongoing work. Supported DDS diffuse textures are now available in the first viewer. Static KN5 geometry, hierarchy, and material definitions are read; unsupported features are diagnosed. Physics validation, force feedback, AI, audio, and networking remain later stages. Full mod compatibility, CSP support, and exact original behavior are not established.

The native Rust/wgpu path now renders real Drift geometry with 25 embedded diffuse textures, approximate alpha/depth states and source normals/UVs. It agrees with the Python reader on tested structural counts, visible selection and bounds. The workspace follows openOMSI's subsystem-crate organisation. Native INI/LUT and zero-rotation static scene assembly now load multiple declared models. Next, add original material behavior and a fixed-step simulation/platform foundation. Native physics and FFB must be validated against the original game rather than inferred from the approximate prototype.

Native static support queries are also available without Python or a GPU; see the [ground command guide](docs/USER_GUIDE.md#native-static-ground-support). Original native vehicle dynamics remain unimplemented.

The native rigid-body foundation is available as `asc-physics`, with an original [numerical example and validation guide](docs/USER_GUIDE.md#native-physics-foundation). Native vehicle force laws and a controlled driving loop remain work ahead.

## License and independence

New source code, original branding and website code use the [MIT license](LICENSE). Progress screenshots depict locally loaded game content; MIT does not relicense that underlying content. This does not grant rights to Assetto Corsa or third-party content. Game binaries, cars, tracks, textures, and audio are not distributed here. Use your own installed copy for local research.

Assetto Corsa is a Kunos Simulazioni product. openASC is independent and unaffiliated. No openOMSI code or branding assets are used.

Native [traction workbench](docs/USER_GUIDE.md#native-tangential-contact-and-traction) couples explicit wheel motor torque to finite road support. It uses synthetic viscous-slip parameters; static stiction, transient tyre behavior, source drivetrain mapping and calibrated handling remain incomplete.

Optional [scheduled wheel controls and brakes](docs/USER_GUIDE.md#native-scheduled-wheel-controls-and-brakes) now exercise acceleration followed by passive internal braking in one native workbench run. Static wheel lock, ABS, original brake/drivetrain mapping and full device input remain incomplete.

[Prescribed rack steering](docs/USER_GUIDE.md#native-prescribed-rack-steering) now drives the actual DWB/STRUT mechanism through position, velocity and acceleration, including chassis reaction and actuator work. The local workbench turns on finite road contacts and now feeds the native input/render path below. Original steering/setup calibration remains work ahead.

The [native keyboard-driving window](docs/USER_GUIDE.md#native-keyboard-driving-window) starts paused and shares the tested workbench solver. Diagnostic key-handler input, native GPU motion and braking are verified; physical keyboard/device behavior and original-game parity remain unverified. Explicit KN5 subtree bindings now animate four wheels and four upright groups from the coupled solver, with source rest pivots preserved. The default drive now reads wheel-centre dimensions and tyre radii through explicit source corner references, with explicit hardpoint-X reflection preserving fore/aft axes on both sides; rest-pivot residuals remain visible in diagnostics. Native body/wall collision and full source frame/setup calibration remain incomplete.

The native core exposes [coupled chassis-point impulses](docs/USER_GUIDE.md#coupled-chassis-impulses), including effective mass, free travel/rotor rates and moving-rack work. Run `cargo run --locked -p asc-physics --example coupled_impulse`. A separate fixed-pose adapter now connects collider/WALL detection to simultaneous inelastic contact; continuous driving response remains incomplete.

Use `cargo run --release --locked -p asc-app -- wall-query examples/wall-query.json` for [native collider/WALL geometry preflight](docs/USER_GUIDE.md#native-collider-and-wall-queries): finite 3D contact patches with source provenance and explicit model/spawn mapping. This query does not advance physics or enable native driving collision response.

Run `cargo run --release --locked -p asc-app -- wall-impact examples/wall-impact.json` for a [static coupled WALL impact](docs/USER_GUIDE.md#static-native-wall-impact), including Coulomb friction, per-contact impulses and energy balance. This diagnostic advances zero time and leaves initial overlap unchanged.

Use `cargo run --release --locked -p asc-app -- wall-sweep examples/wall-sweep.json` to [check a complete prescribed collider path](docs/USER_GUIDE.md#prescribed-collider-and-wall-sweeps), including shortest-arc rotation about the chassis reference. This conservative geometry query detects intermediate crossings; integration with actual driving trajectories remains incomplete.
