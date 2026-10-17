# User guide

openASC provides content readers, local geometry inspection and an independent keyboard-driving prototype. The prototype is locally tested on BMW M3 E30/Drift; original physics and general gameplay compatibility are not established.

## Requirements

- Rust and Cargo with native platform build tools for the native preview; tested with Rust 1.96 on Windows.
- A compatible native GPU/driver for wgpu rendering. Structural inspection does not require a GPU.
- Python 3.10 or newer for the existing research tools and browser-driving prototype.
- Your own installed copy of Assetto Corsa (2014) to inspect game content.
- No third-party Python packages.

The website and automated tests work without an installation of the game.

## Get the source

```sh
git clone https://github.com/openASC-project/openASC.git
cd openASC
```

Keep your installation outside the repository, or use the ignored `assettocorsa/` directory. The scanner accepts either location.

## Native Rust and wgpu preview

The native path works independently of Python and the browser. From the repository root, inspect a single KN5:

```sh
cargo run --locked -p asc-app -- inspect ./assettocorsa/content/tracks/drift/drift.kn5
```

For a 1240 by 900 offscreen PNG, create the output directory first if it does not already exist, then run:

```sh
cargo run --locked -p asc-app -- render ./assettocorsa/content/tracks/drift/drift.kn5 reports/native-drift.png
```

Cargo downloads/builds locked dependencies on first use. Windows also needs the MSVC C++ build tools and Windows SDK for linking Rust dependencies. No C++ engine implementation is included. The PNG path must be new; no existing image is overwritten. Only a flattened frame is saved, with no model or texture extraction. Stdout reports structural counts, diagnostics, adapter/backend, camera focus and non-background pixel count.

The renderer uses native Vulkan/DirectX 12/Metal adapter selection through wgpu. Only the platforms/backends recorded in the compatibility record are verified. Append `--fallback` to explicitly request a software/fallback adapter if one is installed; absence of one is an error. No silent no-op renderer is used.

The current native scope is one static KN5, active visible renderable near-level meshes, composed node transforms, source UVs/normals and supported embedded txDiffuse textures. BC1/2/3 and bounded mask-defined RGB/luminance textures are decoded in Rust and uploaded as RGBA; authored compressed and raw mip chains are retained. Raw textures with only a base level receive approximate generated mips. Camera framing favours active ROAD bounds, otherwise visible scene bounds.

Rendering includes approximate opaque, alpha-test and straight-alpha blend states, material opacity, basic lighting and depth modes. Add `--diagnostic` to skip texture loading for comparison; it can be combined with `--fallback`. Stdout includes uploaded texture and draw-state counts, resource diagnostics and decoded texture metadata. Invalid/missing/ambiguous resources use diagnostic colours with an explanation. Inspect both scene and render diagnostics.

Original shaders, multilayer/detail/normal maps, alpha-to-coverage and shadows are not reproduced. In Drift, grass remains visibly white/stippled because its original material behavior is unsupported. Transparent intersections, colour/lighting and coplanar surfaces remain approximate. Skinned meshes are structurally read and omitted. Track scene/layout configuration, dynamic LOD, interactive windows/cameras, native vehicle simulation and FFB are not implemented. Use the existing browser path below for keyboard-driving features.

## Inspect your installation

Run from the repository root:

```sh
python tools/inspect_content.py ./assettocorsa
```

For an installation elsewhere, pass its root directory in quotes:

```powershell
python tools/inspect_content.py "D:/SteamLibrary/steamapps/common/assettocorsa"
```

The tool expects both `content/cars` and `content/tracks`. It lists immediate content directories and checks for UI metadata, KN5 files, model configuration files, and packed or unpacked car data. It does not read geometry or simulate a vehicle.

## Save a report

```sh
python tools/inspect_content.py ./assettocorsa --output reports/content.json
```

The output contains content-relative paths rather than the absolute installation path. Reports are ignored by Git. The tool refuses to overwrite an existing output file or write a report inside the game installation. Use a new filename for another scan.

## Understand the results

- `scope: file_presence_only` means the report only records file presence.
- `counts` counts content directories, not verified playable cars or track layouts.
- `kn5_present` checks for KN5 files directly inside a content directory.
- `ui_metadata_present` checks the default UI metadata path; individual track-layout metadata is not inspected.
- Data archive presence does not imply ACD decoding support.

## Troubleshooting

An invalid installation path exits with code 2. Check that you passed the game root, rather than `content/` or the repository root. Permission errors are reported without changing the installation.

For bugs, open an [issue](https://github.com/openASC-project/openASC/issues) with the command, Python version, and error. Review reports before sharing them; do not attach game assets or third-party mods.

## Inspect INI and LUT files

Run these commands from the repository root, passing the file you want to inspect:

```sh
python -m openasc ini ./assettocorsa/content/tracks/drift/data/surfaces.ini
python -m openasc lut ./assettocorsa/sdk/dev/content/cars/formula_k/data/power.lut --sample 1750
```

The INI command returns normalized section/key identifiers and string values. The LUT command returns numeric points and, optionally, a linearly interpolated sample. Samples outside the table are clamped to its endpoints. File paths vary by installation; the SDK example is optional.

Text defaults to UTF-8 with optional BOM. For a legacy text file, pass an explicit encoding such as `--encoding cp1252`. The reader does not guess an encoding. Malformed or ambiguous input reports a diagnostic with a code, source, and line number.

## Inspect a static track scene

```sh
python -m openasc scene ./assettocorsa --track drift
python -m openasc scene ./assettocorsa --track ks_barcelona --layout layout_gp
```

The track is a folder name beneath `content/tracks`. A layout selects `models_LAYOUT.ini`; without it, the command requires `models.ini`. If a choice is required, the error lists available layout identifiers rather than picking one automatically.

Each `MODEL_n` declaration needs an explicit `FILE`, `POSITION`, and `ROTATION`. Resource references must stay within the selected track directory. The tool checks file presence and a KN5 version 5/6 header. It preserves transform values but does not apply them or decode geometry.

The JSON report labels this scope `configuration_and_header_only` and sets `geometry_loaded` to false. Dynamic objects, other sections, and unknown model keys produce warnings. Invalid transforms, missing files, and unsupported headers produce errors. A report can contain valid models alongside errors, so always inspect `has_errors` and `diagnostics`.

Successful inspections exit with code 0. Error diagnostics exit with code 2; warnings alone do not change the exit status. These commands write JSON to standard output and never modify the installation. Parse failures are written as JSON to standard error.

## Preview local track geometry

```sh
python -m openasc view ./assettocorsa --track drift
```

Open [the local viewer](http://127.0.0.1:4180/) in a browser with WebGL 2. Loading can take several seconds while geometry is validated. The server prints its URL and stays running until Ctrl+C. Use `--port 4181` if the default port is occupied, and `--layout NAME` to select a track layout.

- Left-drag to orbit, right-drag or Shift-drag to pan, and scroll to zoom.
- Focus the canvas and use arrow keys to orbit or plus/minus to zoom.
- Fit road names uses bounds of active meshes containing ROAD, including non-renderable meshes. This is a naming heuristic, not verified surface classification.
- Fit all includes the full visible scene, which can contain a large background landscape.
- Top view changes camera elevation. Filter the mesh list and select an entry to focus and highlight it.
- The inspector shows material definitions, shader parameters, resource mappings, texture upload counts, and diagnostics.
- Textures on/off switches between diffuse texture preview and diagnostic material colours.

The viewer resolves active embedded txDiffuse resources and supports a documented DDS subset. It uses basic lighting, a fixed 0.5 alpha cutoff for alpha-tested materials, approximate mesh-sorted straight-alpha blending, and recorded depth modes. Alpha-to-coverage is approximated by a cutoff. A small depth bias is used for read-only overlays. Original shaders, multilayer/detail/normal maps, shadows, and exact LOD behavior are not reproduced. Coplanar overlays may still show artifacts, and materials that rely on additional maps can have incorrect colours. This is an inspection tool, not visual parity with Assetto Corsa.

The browser uses S3TC compressed uploads when supported and an original software RGBA decoder otherwise. Both paths were tested on local Drift. Raw RGB/luminance textures use software unpacking and GPU-generated mipmaps. For troubleshooting, open `http://127.0.0.1:4180/?software-textures=1` to force the software path. Unsupported DDS variants, missing embedded resources, budget failures, or GPU upload failures remain visible in diagnostics and use fallback colours.

Inactive, hidden, non-renderable, empty, and later-LOD meshes are omitted from drawing. Nonzero model-level ROTATION values are diagnosed and skipped until their convention is validated. Node hierarchy transforms and model translations are applied. Skinned meshes are diagnosed and skipped.

The server binds only to 127.0.0.1 and rejects foreign Host/Origin headers. It serves the selected scene from memory without exporting assets to disk. Do not use a proxy or tunnel to expose it publicly. GitHub Pages serves documentation only and cannot access the local game installation.

## Try the first-drive prototype

```sh
python -m openasc drive ./assettocorsa --car bmw_m3_e30 --track drift --port 4180
```

Open the printed loopback URL in a WebGL 2 browser. Loading validates car data, physical ground and spawn transforms, then loads the local track and car model. The session starts paused; click Resume to focus the scene. W/S apply throttle/brake, A/D steer, Space applies the handbrake, and E/Q shift up/down through reverse, neutral and forward gears. Arrow keys are alternative driving controls. P pauses/resumes; R resets to the selected spawn and pauses. The Follow car button switches the chase camera on/off; dragging, zooming or using inspection camera buttons releases follow mode.

The command also accepts `--spawn`, `--layout`, `--compound`, `--data-source` and `--encoding`. Naturally aspirated RWD/FWD configurations within the documented prototype bounds are admitted, and an existing supported collider.kn5 is required. Local verification currently covers BMW M3 E30 on Drift. A separate duplicate Drift start slot remains visible as a catalog error; the selected AC_PIT_0 is unambiguous and has four support contacts. Ground-loading errors, an invalid selected spawn or initial body/wall overlap prevent startup.

This is an independently implemented first-drive prototype, not validated Assetto Corsa physics. It uses a 120 Hz planar force model, four friction-limited contact patches and geometric road attachment. Chassis height is estimated from ground, tyre radius and BASEY, without solving spring equilibrium. Steering and wheel spin are not animated. The renderer applies the recorded graphics offset and a local X pitch rotation approximately; external skin textures and original paint/material behavior remain unsupported.

Static WALL meshes now contact an upright box enclosing the car's collider model. Continuous translation/yaw queries prevent ordinary endpoint tunnelling; inelastic planar impulses oppose entry and allow sliding/reverse departure. This is an approximate box proxy: full convex/6DOF contact, body/ground scraping, moving obstacles and non-WALL meshes are not supported. The on-screen wall label reports contact resolutions; repeated contact does not mean a distinct crash each time. A bounded event cap may drop remaining step motion in a complex contact and is counted in telemetry.

Missing/steep support, a ground-height jump over 0.35 metre, unsupported vertical contact, terrain-induced body overlap or a numeric/query limit pauses the prototype at the last valid position. Surface bumps, suspension dynamics, real tyre slip curves, weight transfer, clutch/engine rotational dynamics, original differential behavior, fuel/aero/electronics, audio, FFB, AI and multiplayer are not implemented. Use the on-screen diagnostics and [compatibility record](FORMATS.md).

Tab hiding, window blur or 0.5 second without a valid controller heartbeat pauses the simulation and clears pedal/steering input. Use one controller tab per session. Closing the page leaves the loopback process running until Ctrl+C; controller sequence errors from another tab require reloading. The local command endpoint checks same-origin headers, a session nonce, a monotonically increasing sequence and a bounded JSON input schema. It is not a public/network game server.

## Inspect a car configuration

```sh
python -m openasc car ./assettocorsa --car bmw_m3_e30
python -m openasc car ./assettocorsa --car bmw_m3_e30 --compound 0
```

The car is a folder under `content/cars`. The command reads the supported vehicle configuration fields and the referenced engine torque LUT. It reports mass, inertia dimensions, graphics offsets, steering, wheelbase and axle geometry, selected tyre dimensions, gear ratios, brakes and the first visual model reference. Model validation at this step is header-only. The report has `scope: vehicle_configuration_subset` and `simulation_ready: false`; successful parsing does not establish driving or physics compatibility.

By default an existing `data/` directory is selected as a whole; otherwise the ordinary `data.acd` archive is read in memory using the original car folder identifier. Files are never extracted or modified. If both sources exist, the selected source is diagnosed. Use `--data-source packed` or `--data-source unpacked` to choose explicitly. Missing files in the selected directory do not fall back to the archive. Renaming a packed car directory can change its data key. Other protected/archive variants are unsupported.

Without `--compound`, the reader requires the explicit default index in `tyres.ini`. Zero selects FRONT/REAR; another supported index selects FRONT_n/REAR_n. A missing requested compound is an error, not a fallback. Known unimplemented fields and turbo presence are diagnosed; recorded coefficients do not execute original physics.

## Inspect a spawn and wheel footprint

```sh
python -m openasc spawns ./assettocorsa --track drift
python -m openasc place ./assettocorsa --car bmw_m3_e30 --track drift --spawn AC_PIT_0
```

`spawns` lists supported active PIT, START and HOTLAP_START marker transforms with source positions and axes. `place` combines a selected car configuration, that spawn and the physical ground reader, probing vertically beneath the four wheel footprint locations. Both accept `--layout NAME`; `place` also accepts the car's compound and data-source options. The documented source convention is +X left, +Y up and +Z forward, checked against the local BMW wheel-node positions and the track marker basis.

Placement probes start in the marker's footprint plane with a fixed 20 metre downward range. They do not use BASEY or tyre radius to solve a chassis position. Four hits set `all_wheels_supported: true`; the separate `has_errors` field includes problems elsewhere in the inspected scene. In local Drift, AC_PIT_0 has four ROAD contacts, while a duplicate AC_START_3 remains an explicit catalog error. The command therefore exits with code 2 for that inspected scene even though the selected footprint has support. No implicit start-slot choice resolves duplicate names.

This is a geometric preflight, not a playable session. Suspension equilibrium, tyre forces, body clearance, walls, vehicle graphics alignment and motion remain unimplemented.

## Query physical ground support

```sh
python -m openasc ground ./assettocorsa --track drift
python -m openasc ground ./assettocorsa --track drift --walls
python -m openasc ground ./assettocorsa --track drift --at 0 100 0 --distance 150
```

The first command loads explicit surface definitions and supported physical ground geometry, then reports meshes, triangle counts, source coefficients and diagnostics. The second also casts downward from the supplied X/Y/Z position, returning the nearest surface within the specified distance or null when no ground is found. The origin and distance use metres; Y is up. Use `--layout NAME` to select a layout and `--encoding NAME` for an explicitly selected legacy encoding.

This path includes hidden/non-renderable physical meshes that the renderer omits. It uses system surface definitions followed by the selected track/layout definitions. It requires an unambiguous supported surface name and never substitutes a guessed friction value. Consult `has_errors` and `diagnostics` even when a hit is returned, since unsupported model declarations can leave a partial scene. Errors exit with code 2; warnings and geometric misses alone exit with code 0. Results go to standard output without exporting geometry or modifying the installation.

The result prepares geometric support for later vehicle work. It does not simulate a car, tyres, walls, surface bumps, sound or FFB. See the [compatibility record](FORMATS.md) for the exact matching policy and limits.

## Inspect a KN5 directly

```sh
python -m openasc kn5 ./assettocorsa/content/tracks/drift/drift.kn5
```

This parses the supported binary structure and returns counts and diagnostics without opening a server or exporting geometry. Binary errors include a `byte_offset` as well as the source filename. A header-only file that passes the lightweight scene inspector will fail this full supported-structure reader.
