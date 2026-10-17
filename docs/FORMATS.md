# Content formats

This is the compatibility record for the current implementation. Planned work is separate from verified behavior. No gameplay compatibility is established.

## Current support

| Area | Current behavior | Not implemented |
| --- | --- | --- |
| Content directories | Enumerates immediate car and track folders | Full layout discovery and semantic validation |
| UI metadata | Checks whether default JSON files exist | JSON field interpretation |
| INI | Strict section/key/value subset with source-aware errors | Duplicate merging, includes, expressions, and CSP extensions |
| LUT | Numeric pairs, validation, linear interpolation, endpoint clamping | Inline LUT expressions and higher-dimensional tables |
| Track scene configs | Static MODEL_n references, explicit transforms, resource confinement | Dynamic objects and validated nonzero model rotations |
| KN5 | v5/v6 static meshes, hierarchy, material definitions, and embedded texture offsets | Original shaders, skeletal rendering, protected variants |
| DDS textures | Embedded diffuse BC1/2/3 and mask-defined RGB/luminance subset | DX10, BC4-7, cubes, volume/array, premultiplied formats, external files |
| Track surfaces | Explicit system/selected-track definitions, typed coefficients and flags, supported numeric-prefix mesh matching | Original matching precedence, implicit defaults, procedural bumps, audio and FFB effects |
| Ground support | Transformed static triangles, BVH and nearest downward support probe | Walls, rigid-body collision response, suspension and tyre forces |
| ACD | Checks whether a car archive exists | Archive decoding |
| CSP and plugins | No compatibility claimed | Extension configs, scripts, and runtime behavior |

The inventory scope is `file_presence_only`. Lightweight scene resource inspection remains `configuration_and_header_only`, with `geometry_loaded: false`. The separate KN5 reader and local viewer use `static_geometry_preview`; the viewer reports `geometry_loaded: true` and whether validated texture resources are available via `texture_preview_available`. Actual successful texture uploads are counted in the browser inspector. `original_shaders` remains false; server metadata alone does not prove GPU rendering.

## INI subset

- UTF-8 with optional BOM by default; legacy encodings require an explicit selection.
- Section headers and keys are normalized to uppercase; values remain strings.
- Keys are split from values at the first equals sign. Empty values are supported.
- Blank lines and full-line semicolon, hash, or double-slash comments are ignored.
- Semicolons begin inline comments outside double-quoted values.
- Whole values may be double-quoted. Backslashes remain literal, without escape processing.
- Duplicate sections and duplicate keys, including case variants, are errors rather than silently merged values.
- Keys before a section, malformed headers, bare lines, and malformed quotes are errors.

This strict subset deliberately rejects ambiguous input. It does not claim the original game's complete parser behavior. Section and key source lines are retained in the Python API and diagnostics.

## LUT subset

Rows contain exactly two finite numeric values separated by a vertical bar. The following is an original example:

```text
0|10
10|30
```

The x coordinates must be strictly increasing. Duplicate or descending coordinates, empty tables, extra columns, NaN, and infinity are errors. Scientific notation and a single-point constant table are supported. Comments follow the same rules as the INI reader.

Sampling uses piecewise linear interpolation. The example returns 20 at x=5. Samples outside the domain return the nearest endpoint value. This interpolation policy belongs to openASC's current reader; equivalence to every original game subsystem is not claimed. Extreme numeric ranges that cannot be interpolated reliably are diagnosed.

## Static track configuration

The scene inspector selects `content/tracks/TRACK/models.ini`, or `models_LAYOUT.ini` when a layout is provided. It requires explicit file, position, and rotation fields in each numbered MODEL section.

Model paths may use forward or backslashes, but must resolve within the selected track folder. Absolute paths, Windows drive-relative paths, parent escapes, and resolved links outside that root are rejected. Vectors require three finite numbers. Values are recorded without interpreting rotation units or applying transforms yet.

Unknown sections and model keys are warnings. Missing resources, invalid transforms, and unsupported KN5 headers are errors. Errors in one model do not suppress diagnostics for the other model declarations. If no model passes inspection, the report also includes `no_models`.

## KN5 headers and static geometry

Local observations confirm the six-byte `sc6969` signature followed by a little-endian 32-bit version. Version 6 includes another 32-bit header field. The lightweight scene inspector accepts versions 5 and 6, checks header length, and records file size. It does not validate the remaining structure; a header-only fixture can pass that limited inspection. The separate KN5 reader parses the supported remaining structure and rejects unexplained trailing bytes.

The binary reader handles length-prefixed UTF-8 names, embedded texture descriptors and payload skipping, material properties and sampler mappings, transform nodes, and static triangle meshes. Static vertices contain position, normal, UV, and tangent fields in 44 bytes; indices are 16-bit. Material references and index ranges are checked. Skinned node records are consumed to preserve stream alignment, but their geometry is explicitly omitted from the preview.

The renderer applies node hierarchy transforms and model translations, and samples supported embedded txDiffuse textures with basic lighting. It uses a fixed 0.5 cutoff for alpha-tested/alpha-to-coverage materials, approximate mesh-sorted straight-alpha blending, recorded depth modes, and a small read-only overlay bias. Geometry remains double-sided. Original shader behavior, multilayer/detail/normal maps, shadows, and exact distance-based LOD are not reproduced. Hidden/inactive/non-renderable/empty and later-LOD meshes are omitted with a diagnostic. Nonzero model-level rotations are diagnosed and skipped rather than assigned an unverified Euler convention.

Malformed signatures, versions, truncated fields, invalid flags, unknown node types, invalid indices/material references, nonfinite geometry, non-affine transforms, and numeric overflow produce errors with byte offsets. The reader limits files to 2 GiB, total geometry to 256 MiB, combined string bytes to 32 MiB, nodes to 200000, and hierarchy depth to 128. The assembled local preview additionally limits geometry to 384 MiB. These are current implementation limits, not original engine limits.

Primary implementation references used to check field layouts are [AcTools Kn5Basic](https://github.com/gro-ove/actools/blob/master/AcTools/Kn5File/Kn5Basic.cs) and [kn5-converter](https://github.com/RaduMC/kn5-converter/blob/master/kn5%20converter/Program.cs). The openASC parser, bounds calculations, diagnostics, renderer, and fixtures are independently written. No source files from these tools are bundled.

## Embedded DDS preview

The KN5 importer records texture offsets without extracting payloads. The local viewer resolves a unique active embedded txDiffuse reference for each material, case-insensitively within that model. External texture files and ambiguous embedded names are not guessed. Unsupported or missing resources fall back to diagnostic material colours with warnings.

The supported legacy DDS subset includes DXT1/BC1, DXT3/BC2, DXT5/BC3, mask-defined RGB at 16/24/32 bits, and luminance at 8/16/24/32 bits with optional alpha. Channel masks must be contiguous, in range, and nonoverlapping. The reader validates dimensions, surface kind, declared compressed mip-chain lengths, and uncompressed base row pitch. A declared linear size is not treated as raw row pitch. Cube/volume resources, DX10 headers, BC4-7, premultiplied DXT2/4, and other formats are explicitly unsupported.

Compressed authored mip chains are preserved for upload. Raw textures use their validated base image and GPU-generated mipmaps; lower raw authored mips are not interpreted or validated. The upload payload contains only the retained subset, not a guaranteed complete DDS export. UVs and image row order are passed through without an additional flip.

WebGL uses S3TC when available and compatible with mip dimensions. Otherwise, original JavaScript decodes BC1/2/3 or raw pixels to RGBA. Compressed upload failure also attempts the bounded software path. Alpha indices, endpoint interpolation, cropped blocks, channel masks, padded row order, and both BC3 interpolation branches have original pixel tests. Software rounding may differ slightly from a GPU implementation.

DDS limits are 8192 pixels per axis, 16777216 base pixels, and 128 MiB per embedded source. Per-scene retained texture data is capped at 128 MiB, with a 256 MiB RGBA mip budget covering the software path. These are preview limits. A failed GPU upload is reported in the browser, and never counted as a successful upload.

Opaque materials ignore texture alpha. Material transparency uses the recorded opaque/alpha-blend states; alpha-to-coverage uses a cutoff approximation. Depth modes map to normal, read-only, and disabled depth tests. Mesh-centre sorting does not solve intersecting transparent geometry. Grass and other multilayer materials can look white or otherwise incorrect when only their base map is shown. No shader or material parity is claimed.

The independent DDS implementation was checked against the primary Microsoft documentation for [DDS headers](https://learn.microsoft.com/en-us/windows/win32/direct3ddds/dds-header), [pixel formats](https://learn.microsoft.com/en-us/windows/win32/direct3ddds/dds-pixelformat), and [block compression](https://learn.microsoft.com/en-us/windows/win32/direct3d10/d3d10-graphics-programming-guide-resources-block-compression). Recorded state names are cross-checked against AcTools [blend](https://github.com/gro-ove/actools/blob/master/AcTools/Kn5File/Kn5MaterialBlendMode.cs) and [depth](https://github.com/gro-ove/actools/blob/master/AcTools/Kn5File/Kn5MaterialDepthMode.cs) definitions. No third-party decoder source is bundled.

## Surface definitions and ground support

The `ground` command reports `static_ground_query`, independently of the rendering preview. It reads `system/data/surfaces.ini` and then `content/tracks/TRACK/data/surfaces.ini`. With a layout, the second path is `content/tracks/TRACK/LAYOUT/data/surfaces.ini`; the base track file is not used as a fallback. Same-KEY track definitions completely replace system definitions. Missing files produce warnings; no available definitions produce an error. An invalid selected file fails loading instead of silently falling back to a system coefficient. This explicit policy is tested, but equivalence to every original loader precedence rule is not established.

Numbered SURFACE sections require KEY, FRICTION, DAMPING, IS_VALID_TRACK and IS_PITLANE. Keys are normalized to uppercase ASCII identifiers. Friction and damping must be finite and nonnegative; the two flags must be 0 or 1. Other known numeric fields are validated as finite metadata. Sound/FFB references and unknown fields are retained without execution; unknown fields/sections produce warnings. Duplicate keys within one file are errors, with the original source and line preserved.

The supported physical naming subset requires a leading decimal integer greater than zero, including zero-padded names such as `01ROAD`. Exactly one configured surface KEY must prefix the remaining mesh name. Missing or ambiguous matches are diagnosed and omitted; no guessed friction is substituted. Names containing WALL are explicitly omitted because wall response is not implemented. This conservative prefix policy is openASC behavior, rather than a claim to reproduce all original name matching. Naming and default-surface locations were cross-checked against the author-maintained [track construction guide](https://assettocorsamods.net/threads/build-your-first-track-basic-guide.12/) and the local installation.

Active supported physical meshes are included even when hidden or non-renderable. Inactive, empty, later-LOD and skinned geometry are omitted. Node transforms and model translations are applied; nonzero model rotations are diagnosed and omitted. The loader retains up to 1000000 input triangles across models, including subsequently discarded degenerate faces, with world coordinates bounded to +/-10000000 metres. These are current openASC limits. The geometric query builds a median BVH and retains double-precision transformed positions. Faces whose XZ projected area is at most `1e-12 * max(1, projected edge length product)` are excluded as vertical or degenerate.

A probe accepts a finite X/Y/Z origin and nonnegative finite maximum distance, within the same 10000000 metre range. It returns the nearest double-sided triangle below the origin, its geometric height, upward unit normal, mesh/model identity and source surface definition. Multiple road levels are queried relative to the origin, so a bridge above it is not chosen. Shared edges are included with numeric tolerance; equal-height ties use original triangle order. Distance/height comparisons use a 0.0000001 metre tolerance, and barycentric comparisons use a 0.000000001 tolerance. A miss returns null. Invalid query input is an error; a miss alone is not.

This query is geometric preparation for wheel support. It does not implement rigid-body collision, walls, suspension, tyre forces, procedural sinusoidal bumps, surface sound, dirt accumulation, penalties or FFB. Recorded friction/damping and validity/pit flags are metadata for later systems. The rendering viewer's ROAD-name focus button remains a separate bounding-box heuristic.

## Local validation record

On 2026-10-17, the current readers inspected 42 model configurations referencing 295 static model headers in the development installation with no model errors. There were 22 unsupported-section warnings. These are model references, not a count of unique files or fully supported scenes.

The local Drift KN5 was also parsed through its complete supported structure: 350 nodes, 308 static meshes, 27 materials, and 56 indexed embedded textures. The viewer drew 254 visible near-level meshes containing 1150970 triangles. Browser verification checked non-background geometry pixels, WebGL errors, camera changes, top view, mesh focus, and material inspection. The source game file and local screenshots remain outside the repository. This is an approximate preview, not visual parity with Assetto Corsa.

The texture preview loaded 25 unique diffuse resources for all 27 Drift materials: 17 BC3, 3 BC1, 4 RGB, and 1 luminance texture. Both the S3TC-capable and forced software paths uploaded all 25 without JavaScript/WebGL errors. Browser checks verified substantial texture detail, a changed rendered frame when textures were toggled, and top-view rendering. BC2 is covered by original fixtures, not this Drift sample. Three additional original browser pixel fixtures verified that opaque rendering ignores zero texture alpha, alpha test removes transparent pixels, and half-alpha blending produces the expected intermediate colour. Screenshots remain local and are not published.

The ground path assembled 54522 nonvertical/nondegenerate triangles from 56 Drift meshes: 8 ROAD, 39 GRASS and 9 KERB meshes. Six vertical/degenerate triangles and eight WALL meshes were excluded. Two unsupported model-configuration sections remained explicit warnings. One thousand fixed-seed probes placed 0.1 metre above sampled triangle centroids found support within 0.1 metre, with finite upward unit normals. Median query time was approximately 0.072 ms and the 95th percentile approximately 0.195 ms on the development machine; these timings are observations, not real-time guarantees. Independent original fixtures cover analytic ramps, stacked roads, shared edges, opposite winding, transforms, hidden physical meshes, malformed surface definitions, file precedence, misses, and query/budget limits. No tyre or driving compatibility is established by these checks.

All 27 LUT files found under that installation's SDK content examples parsed with the current subset. This is local evidence, not a claim about all original or community content. Validation reports remain under ignored `reports/`; no game data is published.

## Reproducible examples

Tests use small original fixtures generated in temporary directories, including complete original KN5 structures and explicitly header-only fixtures for the lightweight inspector. Do not add copied cars, tracks, textures, audio, or SDK archives to the repository. A parser change should include a minimal example proving the new behavior and invalid input that must be diagnosed.
