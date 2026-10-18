# openASC

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="site/brand/openasc-dark.svg">
  <img src="site/brand/openasc-light.svg" alt="openASC — open circuit and forward arrow" width="520">
</picture>

[Logo source files and usage](site/brand/README.md) · Artwork licensed under [MIT](LICENSE).

An independent open-source engine project for **Assetto Corsa (2014)** content, inspired by [openOMSI](https://github.com/openOMSI-org/openOMSI).

**Target: 1:1 behavioral compatibility with Assetto Corsa (2014).** This includes vehicle physics, controls and force feedback, visuals, audio, AI, session rules, networking and original content behavior. This is the project objective, not an achieved compatibility claim. The production stack is Rust with wgpu. A first native KN5 reader and offscreen diffuse renderer now run independently of Python and the browser; production simulation remains unimplemented. The Python implementation remains a research prototype and tooling foundation.

**Status: native window/diffuse preview and first local keyboard-driving prototype. Original physics parity is not established.** The repository contains local inventory, INI/LUT readers, a bounded KN5 static geometry importer, an interactive local WebGL viewer, original tests, and a documentation website. Supported embedded diffuse textures, alpha test, approximate transparency, typed track surfaces, vertical ground support queries, ordinary ACD car-data reading and geometric spawn placement checks are available. An independent 120 Hz driving model now moves a locally loaded car with keyboard input. Static WALL triangles now contact an upright box derived from collider.kn5. Full convex/6DOF collision, suspension dynamics, original tyre/drivetrain behavior, audio and FFB are not implemented.

[Website](https://openasc-project.github.io/openASC/) · [User guide](docs/USER_GUIDE.md) · [Compatibility record](docs/FORMATS.md) · [Architecture](docs/ARCHITECTURE.md) · [Roadmap](docs/FEASIBILITY.md)

## Quick start

For the native path, install Rust with Cargo and the platform's native build tools. Rust 1.96 is tested locally. Rendering needs a wgpu-compatible native GPU/driver. Create `reports/` before saving a capture; existing outputs are never overwritten.

```sh
cargo run --locked -p asc-app -- inspect ./assettocorsa/content/tracks/drift/drift.kn5
cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/drift/drift.kn5
cargo run --locked -p asc-app -- render ./assettocorsa/content/tracks/drift/drift.kn5 reports/native-drift.png
cargo run --locked -p asc-app -- car ./assettocorsa/content/cars/bmw_m3_e30
cargo run --locked -p asc-app -- spawns ./assettocorsa/content/tracks/drift/models.ini
```

The native view command opens a desktop window with orbit, pan, zoom and perspective/orthographic cameras. It reuses persistent wgpu resources and the same material pipeline as the PNG path. Pass either one KN5 or an explicit `models.ini`/`models_LAYOUT.ini` path to inspect, render or view a composed static scene. For example, `cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/magione/models.ini` loads both declared models. Embedded diffuse resources remain local to each KN5 and share a bounded scene budget. Supported BC1/2/3 textures now upload directly on capable adapters; --rgba-textures forces the software fallback. The native path now reconstructs masked four-detail base colour for three ksMultilayer variants; original normal/Fresnel/specular lighting remains incomplete. Use `materials SCENE` to inspect retained shader properties and sampler slots. Add `--diagnostic` to skip texture loading. Native INI/LUT readers are available through `ini FILE.ini` and `lut FILE.lut --sample X`. Nonzero model rotations, dynamic objects and native driving remain unsupported. See the [native guide](docs/USER_GUIDE.md#native-static-scene-configuration).

Native `spawns` and `place` provide geometric preparation only. Drift has a duplicate `AC_START_3`, so its reports retain `has_errors: true` and a nonzero process status even when the selected pit footprint has support; see the [placement guide](docs/USER_GUIDE.md#native-spawn-and-placement-preflight).

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
crates/asc-kn5/           Bounded KN5 format reader and model data
crates/asc-texture/       Bounded legacy DDS decoder
crates/asc-render/        wgpu GPU work, pipelines, resources, scene and shaders
tools/test-support/      Original shared Rust fixtures
tools/prototype/openasc/ Python research readers and driving model
tools/prototype/viewer/  Research WebGL preview interface
openasc/                 Compatibility entry point for python -m openasc
tools/build_site.py       Documentation site builder
tests/                    Original, game-independent test fixtures
Cargo.toml / Cargo.lock   Native workspace and locked dependencies
.github/workflows/        CI and GitHub Pages
assettocorsa/             Local game installation, ignored by Git
```

## Next milestone

Typed surface definitions and BVH-accelerated vertical ground queries are available through `ground`. This includes supported hidden road meshes and returns geometric height, an upward normal and source surface metadata. The path is tested on original fixtures and local Drift. Ordinary packed car data, typed vehicle parameters, spawn transforms and four-wheel support preflights are also available. The separate drive command adds independent planar motion with geometric road attachment; it does not reproduce original tyre, suspension or wall behavior.

BMW M3 E30/Drift now runs in an independently implemented local keyboard-driving prototype with tested acceleration, steering, braking, reverse, pause and reset. Static wall contact now uses continuous translation/yaw queries and inelastic planar impulses, with verified nonpenetration on the tested traces. Next, add wheel animation and better vehicle/skin materials, then replace geometric attachment with validated suspension/tyre dynamics and full convex/6DOF contact. Supported DDS diffuse textures are now available in the first viewer. Static KN5 geometry, hierarchy, and material definitions are read; unsupported features are diagnosed. Physics validation, force feedback, AI, audio, and networking remain later stages. Full mod compatibility, CSP support, and exact original behavior are not established.

The native Rust/wgpu path now renders real Drift geometry with 25 embedded diffuse textures, approximate alpha/depth states and source normals/UVs. It agrees with the Python reader on tested structural counts, visible selection and bounds. The workspace follows openOMSI's subsystem-crate organisation. Native INI/LUT and zero-rotation static scene assembly now load multiple declared models. Next, add original material behavior and a fixed-step simulation/platform foundation. Native physics and FFB must be validated against the original game rather than inferred from the approximate prototype.

Native static support queries are also available without Python or a GPU; see the [ground command guide](docs/USER_GUIDE.md#native-static-ground-support). Original native vehicle dynamics remain unimplemented.

The native rigid-body foundation is available as `asc-physics`, with an original [numerical example and validation guide](docs/USER_GUIDE.md#native-physics-foundation). Native vehicle force laws and a controlled driving loop remain work ahead.

## License and independence

New source code, original branding and website code use the [MIT license](LICENSE). Progress screenshots depict locally loaded game content; MIT does not relicense that underlying content. This does not grant rights to Assetto Corsa or third-party content. Game binaries, cars, tracks, textures, and audio are not distributed here. Use your own installed copy for local research.

Assetto Corsa is a Kunos Simulazioni product. openASC is independent and unaffiliated. No openOMSI code or branding assets are used.
