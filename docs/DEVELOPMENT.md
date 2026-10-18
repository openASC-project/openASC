# Development

All repository documentation, code comments, diagnostics, commit messages, and website content are maintained in English.

## Run checks

The native workspace uses Rust with wgpu. CI checks both Windows and Linux with the stable Rust toolchain. Native fixtures require no game content:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --locked -- -D warnings
cargo test --workspace --locked
cargo build --workspace --locked
python tools/check_native_parity.py --native target/debug/openasc
```

Use `target/debug/openasc.exe` on Windows. Native/Python contract checks compare original KN5 fixtures, selection, transforms/bounds, INI values/order/source lines, LUT samples, static model declarations and rejection codes. Optional repeated `--model PATH` arguments compare locally installed models without copying them. Agreement between two openASC implementations does not establish original-simulator parity.

The separate GPU test is explicitly opt-in on a machine with a native adapter:

```sh
cargo test -p asc-render --locked --test gpu --test track_textures -- --ignored --nocapture
```

Five single-model GPU tests check non-background geometry, occlusion independent of triangle order, non-aligned row readback, PNG pixels/overwrite refusal, UV orientation, opaque versus cutout alpha, opacity and linear-light blending, read-only/disabled depth and back-to-front transparency ordering. Another GPU test checks BC1/2/3 direct upload against a device created without BC support, including alpha and authored non-power-of-two/sub-block mip tails. A separate composition GPU test proves that two composed models with identical texture names retain distinct red/green pixels. CPU fixtures cover native config parsing, translation and rebasing, resource confinement, model/texture budgets and per-model resource isolation. Set `OPENASC_TEST_FALLBACK=1` to request a software adapter; Linux CI installs Mesa Vulkan and runs these tests explicitly. GPU adapter absence fails the explicit tests. Structural, DDS and resource-resolution tests remain GPU-independent. Cargo build outputs under `target/` are ignored; commit `Cargo.lock` with dependency changes.

The three explicit desktop tests open real windows for a KN5, a composed models.ini scene and forced RGBA fallback, and therefore require a display:

```sh
cargo test -p asc-app --locked --test window -- --ignored --nocapture
```

Linux CI installs `mesa-vulkan-drivers`, `xvfb` and `libxkbcommon-x11-0`, then runs it under Xvfb with software Vulkan. The X11 window backend requires the xkbcommon X11 runtime library. It verifies one uncaptured presentation and four presented/read-back frames, real window resize to 640 by 480, changed camera matrices, BGRA-to-RGBA PNG output and changed pixels from a diffuse toggle at a fixed camera. Camera math and camera-dependent transparency order also have GPU-independent tests. This diagnostic automation does not establish real mouse/keyboard, DPI, monitor migration or device-loss recovery behavior.

Source is organised as a subsystem workspace under `crates/`, following the openOMSI project layout. Build the executable with `cargo build -p asc-app`; it is named `openasc` (`openasc.exe` on Windows). The former `native/openasc-native` package has been replaced by the subsystem crates. Python/WebGL research source is under `tools/prototype/`; the root Python package is only a compatibility entry point. See the [architecture](ARCHITECTURE.md) for dependency boundaries.

Python 3.10+ is enough for the tools, tests, and documentation builder. No game installation is needed for tests.

```sh
python -m unittest discover -s tests -v
python tools/build_site.py --check
node --check site/app.js
node --check tools/prototype/viewer/viewer.js
node --check tools/prototype/viewer/dds.js
node --check tools/prototype/viewer/drive.js
node --test tests/test_dds.cjs
```

Node.js 18+ is used for JavaScript syntax checks and original DDS pixel tests. The website itself has no npm dependencies or build-time JavaScript framework.

The local geometry viewer also has no npm dependencies. Its JavaScript is in `tools/prototype/viewer/`, served only by `python -m openasc view`. Tests construct original complete KN5 fixtures and exercise the loopback server, including foreign-host/origin rejection. Binary fixtures are not copied from the game.

## Update the website

```sh
python tools/build_site.py
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Open [localhost](http://127.0.0.1:4173/). The document builder supports the Markdown subset used here: headings, paragraphs, links, inline code, bold text, lists, fenced code, and tables. Raw HTML in documentation is escaped.

Edit `docs/` for documentation, `web/overview.html` for the overview body, and `web/layout.html` for the shared shell. Edit `site/style.css` and `site/app.js` for styling and browser behavior. Generated HTML files contain a notice identifying their source.

## Contribute

The independent driving model is in `tools/prototype/openasc/dynamics.py`; its tests use original flat/sloped geometry and original car configurations. Keep rendering and HTTP telemetry reads out of the fixed-step solver. Controller tests can advance exact steps with an injected clock instead of sleeping. Real-content/browser validation remains local; record the scope and limitations before changing compatibility claims.

Website progress screenshots are selected flattened captures under `site/images/`, with captions in `web/overview.html`. Update them after meaningful visual milestones. Do not add raw game assets or temporary browser reports. The project's MIT license does not relicense the underlying game content depicted in screenshots.

Start with an [issue](https://github.com/openASC-project/openASC/issues) describing a reproducible problem or bounded feature. Keep parser fixtures original and small. A green test suite does not establish compatibility with all Assetto Corsa content.

Before committing, check that neither `assettocorsa/` nor `reports/` is tracked. These directories are ignored intentionally. Do not include original executable files, copyrighted game content, or third-party mods.

New source code and website files use the MIT license. Dependencies and any future third-party additions must retain their own license notices.
