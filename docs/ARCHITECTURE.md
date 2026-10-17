# Architecture

The implemented project contains Python content readers, a local static geometry preview in WebGL 2, and a documentation website. Vehicle simulation and a production renderer remain proposed work.

## Implemented components

| Location | Responsibility |
| --- | --- |
| `tools/inspect_content.py` | Read-only content inventory and JSON CLI |
| `openasc/config.py` | Source-aware INI and LUT parsing, numeric validation, and interpolation |
| `openasc/scene.py` | Confined static model references, transforms, and KN5 header inspection |
| `openasc/kn5.py` | Bounded KN5 parsing, static meshes, node transforms, and material definitions |
| `openasc/viewer.py` | Local scene assembly and loopback-only preview server |
| `viewer/` | WebGL 2 diagnostic shading, camera controls, mesh/material inspection |
| `openasc/__main__.py` | JSON diagnostics through `python -m openasc` |
| `tests/` | Tests using original, temporary fixtures |
| `docs/` | English documentation and current compatibility record |
| `web/` | Shared website template and overview source |
| `tools/build_site.py` | Build documentation pages using Python's standard library |
| `site/` | Generated HTML, shared CSS/JS, and public static assets |
| `.github/workflows/` | Source checks and GitHub Pages deployment |

## Proposed engine boundaries

- **Content:** configuration readers, resource resolution, metadata, and precise loading diagnostics.
- **Scene:** geometry, material definitions, transforms, surfaces, and collision data independent of a renderer.
- **Render:** scene inspection, camera, lighting, and material preview.
- **Simulation:** fixed simulation step, drivetrain, suspension, aerodynamics, and tyres.
- **Platform:** windowing, clocks, input devices, and force feedback.
- **Application:** content selection, session setup, diagnostics, and the driving loop.

Keep file-format readers separate from rendering and physics so each can be tested without opening a window. Unsupported resources should produce diagnostics rather than silently appearing compatible.

## Technology choices

Python keeps content loading and the local HTTP server dependency-free. The first inspection renderer uses browser-native WebGL 2 with original JavaScript. This does not select a production simulation stack; Rust with wgpu remains one candidate. No Rust engine currently exists in this repository.

The local viewer receives a metadata manifest and aligned vertex/index buffers from its loopback server. Static vertices retain their original interleaved format. Node transforms are composed on the CPU and applied per draw on the GPU. Textures are skipped during loading, so their payloads are neither held in the preview buffer nor exported. The public documentation site and local viewer are separate applications.

## Website

The website uses a shared documentation layout inspired by [openomsi.org](https://openomsi.org/): persistent navigation, a project overview, and dedicated documentation pages. Its original visual styling uses a charcoal, red, and white motorsport palette, italic branding, and a dark default theme. The implementation and content are original to openASC.

Documentation pages are generated from the same Markdown files that appear on GitHub. Run `python tools/build_site.py` after editing documentation or the shared template, then commit the generated HTML. Relative assets work beneath `/openASC/` on GitHub Pages.
