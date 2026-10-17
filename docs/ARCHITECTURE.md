# Architecture

The implemented project contains Python content readers, a WebGL 2 preview, an independent fixed-step driving prototype and a documentation website. A production renderer and validated full vehicle simulation remain proposed work.

## Implemented components

| Location | Responsibility |
| --- | --- |
| `tools/inspect_content.py` | Read-only content inventory and JSON CLI |
| `openasc/config.py` | Source-aware INI and LUT parsing, numeric validation, and interpolation |
| `openasc/scene.py` | Confined static model references, transforms, and KN5 header inspection |
| `openasc/kn5.py` | Bounded KN5 parsing, static meshes, node transforms, material definitions, and embedded texture offsets |
| `openasc/dds.py` | DDS validation, mip descriptors, and bounded embedded texture reads |
| `openasc/surfaces.py` | Typed surface definitions, explicit file precedence, and conservative physical mesh naming |
| `openasc/ground.py` | World-space ground triangles, a BVH, and vertical geometric support queries |
| `openasc/acd.py` | Bounded in-memory indexing and reading of ordinary folder-keyed ACD records |
| `openasc/vehicle.py` | Explicit packed/unpacked car-data selection and typed vehicle configuration subset |
| `openasc/placement.py` | Rigid spawn transforms and a four-wheel geometric support preflight |
| `openasc/dynamics.py` | Independent fixed-step planar force model with four ground contact patches |
| `openasc/driving.py` | Car/track preview assembly, a sequenced local controller and simulation clock |
| `openasc/viewer.py` | Local scene assembly and loopback-only preview server |
| `viewer/` | WebGL 2 diffuse preview, portable DDS pixel decoding, camera controls, and mesh/material inspection |
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

The local viewer receives a metadata manifest and aligned vertex/index buffers from its loopback server. Static vertices retain their original interleaved format. Node transforms are composed on the CPU and applied per draw on the GPU. The binary importer indexes texture offsets without extracting assets. The preview then validates and loads referenced embedded diffuse textures into separate bounded memory buffers. Only numeric texture IDs are exposed by same-origin local routes; no disk path or resource export endpoint is exposed. The browser uses S3TC uploads when possible, with an independent RGBA software fallback. Authored compressed mip chains are retained; raw texture base levels use GPU-generated mipmaps. The public documentation site and local viewer are separate applications.

## Ground support queries

The first contact preparation path is separate from rendering. `ground` loads system and selected track surface definitions, includes supported active physical meshes even when hidden/non-renderable, transforms their vertices, and builds a median BVH. A downward probe returns the nearest triangle within its finite distance, an upward geometric normal, and the source surface definition. It does not execute a simulation step, tyre forces, procedural surface bumps or wall response. Surface friction is recorded input for later physics, not a validated grip model.

## Vehicle configuration and placement

The vehicle configuration reader uses a single selected data source, without mixing files from an archive and a directory. It retains source locations for consumed INI fields and loads the referenced torque LUT without extraction. Inertia dimensions, steering and axle geometry are separate recorded inputs; the configuration reader does not execute vehicle dynamics. The KN5 reader also retains node parent links, active state and composed transforms, so invisible spawn markers survive independently of drawing visibility. The placement preflight maps the car's wheel footprint through a selected source basis, then queries static support. It does not solve chassis height, spring equilibrium or collision response.

## First-drive runtime

The first-drive prototype has a separate Python simulation state, stepped at 120 Hz by a clock thread. HTTP telemetry and WebGL redraws do not advance physics. A locked session applies complete, validated, sequenced controls; missing heartbeat input pauses and clears controls. Clock catch-up is capped at eight steps per wake, with discarded backlog counted in telemetry. A browser tab polls telemetry and sends keyboard input, while the existing WebGL buffers draw the car beneath its current world transform. Track and car texture/geometry budgets are shared. The simulation currently constrains vertical pose to supported static ground; it is not a six-degree-of-freedom suspension or collision solver.

## Website

The website uses a shared documentation layout inspired by [openomsi.org](https://openomsi.org/): persistent navigation, a project overview, and dedicated documentation pages. Its original visual styling uses a charcoal, red, and white motorsport palette, italic branding, and a dark default theme. The implementation and content are original to openASC.

Documentation pages are generated from the same Markdown files that appear on GitHub. Run `python tools/build_site.py` after editing documentation or the shared template, then commit the generated HTML. Relative assets work beneath `/openASC/` on GitHub Pages.
