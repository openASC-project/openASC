# User guide

For read-only measurement of an already running original-game session on Windows, see the separate [telemetry research guide](TELEMETRY.md). This collector supports future comparisons; live acquisition has not yet been verified.

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

The current native scope is a static KN5 or an explicit static scene configuration, active visible renderable near-level meshes, composed node transforms, source UVs/normals and supported embedded txDiffuse textures. Block-aligned BC1/2/3 textures upload directly when the selected device supports BC compression; other supported textures decode in Rust and upload as RGBA; authored compressed and raw mip chains are retained. Raw textures with only a base level receive approximate generated mips. Camera framing favours active ROAD bounds, otherwise visible scene bounds.

Rendering includes approximate opaque, alpha-test and straight-alpha blend states, material opacity, basic lighting and depth modes. Add `--diagnostic` to skip texture loading for comparison; it can be combined with `--fallback`. Stdout includes uploaded/compressed texture counts, upload payload bytes, draw-state counts, resource diagnostics and texture metadata. The bc_enabled field reports whether BC support was enabled on the requested device. Add --rgba-textures to force software decoding of retained BC sources for comparison; unsupported/disabled BC falls back within the same bounded upload budget. Invalid/missing/ambiguous resources use diagnostic colours with an explanation. Inspect both scene and render diagnostics.

The native renderer reconstructs the base colour of ksMultilayer, ksMultilayer_objsp and ksMultilayer_fresnel_nm using their mask/detail inputs. Original normal maps, Fresnel/specular lighting, alpha-to-coverage and shadows remain incomplete. Grass now receives detail colour, but stippling, colour/lighting and coplanar artifacts remain unvalidated. Transparent intersections, colour/lighting and coplanar surfaces remain approximate. Skinned meshes are structurally read and omitted. Dynamic objects/LOD, nonzero model rotations, native vehicle simulation and FFB are not implemented. Use the existing browser path below for keyboard-driving features.

## Native static scene configuration

Use an explicit configuration path, including the layout suffix when needed:

```sh
cargo run --locked -p asc-app -- inspect ./assettocorsa/content/tracks/magione/models.ini
cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/magione/models.ini
cargo run --locked -p asc-app -- render ./assettocorsa/content/tracks/magione/models.ini reports/native-magione.png
cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/ks_barcelona/models_layout_gp.ini
```

Only `MODEL_n` sections are assembled, in declaration order. Each requires `FILE`, `POSITION` and `ROTATION`. Zero rotation and finite translations within the supported world range are implemented; nonzero rotations fail explicitly until their original convention is validated. Models resolve relative to the selected configuration's directory. Windows/Unix absolute paths, drives, streams, parent traversal and symlinks resolving outside that directory are rejected. Missing/invalid static models fail the entire load. Dynamic sections and unknown keys are omitted with diagnostics.

The native manifest reports each source model's version, position and material/texture ranges. Composed geometry uses version `0` as a composition marker, not a KN5 format version. Textures resolve from each model's own embedded table; original shared/cross-model texture precedence is unvalidated. Source, retained texture storage and upload payload budgets apply to the whole scene. Resources beyond them use diagnostic colours and report the reason. Magione now loads all 44 diffuse resources on the verified BC-capable adapter. The RGBA fallback still reaches the upload budget; full original grass/material behavior remains unvalidated; see the multilayer base-colour scope below. Original source files remain unchanged.

Native text inspection is also available without a GPU:

```sh
cargo run --locked -p asc-app -- ini ./assettocorsa/content/tracks/drift/models.ini
cargo run --locked -p asc-app -- lut ./assettocorsa/content/cars/CAR/data/CURVE.lut --sample 5000
```

Replace the standalone LUT example with an existing unpacked file. Native vehicle loading can also read ordinary data.acd records in memory, as described below. Native text input is UTF-8 with an optional BOM, with strict duplicate/quote checks and source lines. LUT interpolation is linear with clamped endpoints and finite-number/range checks. Legacy encodings, includes and inline expressions are unsupported. The Python research tools retain their separate explicit encoding support.

## Native car and track inspection

Open a static native session preview, or save a flattened offscreen capture:

```sh
cargo run --locked -p asc-app -- preview ./assettocorsa/content/cars/bmw_m3_e30 ./assettocorsa/content/tracks/drift/models.ini
cargo run --locked -p asc-app -- preview ./assettocorsa/content/cars/bmw_m3_e30 ./assettocorsa/content/tracks/drift/models.ini --capture reports/native-session.png
```

The output directory must already exist and the PNG name must be new. `--capture` renders offscreen and exits. Without it, the normal orbit/pan/zoom inspection window opens; `--captures DIR` enables F12 captures and `--smoke` exercises presentation/resize/readback. `--spawn NAME`, `--data auto|packed|unpacked`, `--compound N`, `--fallback`, `--diagnostic` and `--rgba-textures` are supported. Select the matching models_LAYOUT.ini for a layout. This command does not accept driving controls.

The track and vehicle share geometry/material/texture budgets and one render pass. Car meshes retain their source geometry with a separate rigid GPU root. The initial visual root uses the selected marker basis/position, then GRAPHICS_OFFSET and GRAPHICS_PITCH_ROTATION. Its camera starts at the car bounds. The marker plane is an inspection pose, not solved chassis CG or suspension equilibrium. The report retains source marker errors even when the selected marker is usable; a preview can render that pose while reporting has_errors true. Missing/ambiguous selected markers and unsupported model data fail.

On local BMW M3 E30/Drift, the preview renders 166 car draws and 254 track draws with 72 embedded textures. The native window also passes presentation, resize and readback checks. Body paint/skins, glass/reflections, lighting/shadows, wheel animation and original material behavior remain incomplete. The report states driving false and simulation_ready false. See the [rigid-instance/session scope](FORMATS.md#native-rigid-instances-and-session-preview).

## Unpacked data directories

An unrelated backup/conflict file in the selected `data/` directory no longer prevents loading valid car configuration. Nonportable filenames produce `car_data_name_ignored` warnings and are excluded from the selectable resource index. For example, `tyres (conflicted copy).ini` is ignored while the explicit `tyres.ini` is read. The source files are not renamed, removed or rewritten.

Requested car-data references still require flat portable ASCII filenames; ignored files are never aliases for missing resources. Missing required files and case-insensitive collisions between selectable filenames remain errors. Auto mode still selects an unpacked directory as a whole and never fills missing files from `data.acd`. Explicit packed mode does not enumerate the unselected directory.

Enumeration is bounded to 4096 immediate directory entries before sorting; files with ignored names and subdirectories count toward that limit. Selected-file byte/encoding/confinement checks remain in force. See the [index policy and local validation](FORMATS.md#unpacked-directory-indexing).

## Native suspension records and spring links

Inspect the source suspension records without Python or a GPU:

```sh
cargo run --locked -p asc-app -- suspension ./assettocorsa/content/cars/bmw_m3_e30
cargo run --locked -p asc-app -- suspension ./assettocorsa/content/cars/bmw_m3_e30 --data packed
cargo run --locked -p asc-physics --example spring_pair
```

The `suspension` command reads a complete selected packed/unpacked source. It supports explicit STRUT/DWB hardpoint records, wheelbase/track/base offsets, rod length, recorded hub mass, spring/damper/stop numbers, toe/camber and optional ARB values. Every consumed field has a source and line number. Missing data is not filled from another source. Unknown fields remain diagnostic; the report is configuration-only with simulation_ready false.

Source parameter names and numeric values are preserved. The reader does not solve hardpoint coordinate frames/mirroring, linkage constraints, unsprung masses, ride-height preload or original progressive/bump-stop/damper laws. In particular, raw DAMP_*THRESHOLD values are not automatically treated as the independent element's velocity knees.

The original spring_pair example exercises a separate Rust mechanical element: two bodies, a linear spring and a passive damper. It checks coupled motion against a closed-form damped oscillator, not against the original game. The kernel can also sum multiple spring links at common midpoint states and commit every body once per tick. Link sampling returns forces, moments, stored energy and dissipated power. It does not provide a tyre, ground-contact or full vehicle suspension solver. See the [force-link and source-record contract](FORMATS.md#native-spring-links-and-suspension-records).

## Native physics foundation

The Rust physics kernel implements unconstrained three-dimensional rigid-body motion. Run its original numerical example without a game installation, Python or a GPU:

```sh
cargo run --locked -p asc-physics --example free_body
cargo test --locked -p asc-physics
```

The example starts a 2 kg body at (0,10,0), supplies an explicit body-space inertia tensor and constant world force/torque, and advances one second at 120 Hz. Its checked final position is (0.5,5.095,0.75) metres; angular momentum is (1,0.5,0.25) kg m2/s. Output contains the initial/final state and reports original_physics_validated false, simulation_ready false. Its eight-step work budget retains and drains the remaining 112 ticks rather than dropping elapsed time.

`asc-physics` exposes full inertia tensors, unit-quaternion orientation, world linear/angular momenta, loads at world/body points, point velocity and instantaneous impulses. Gravity is an explicit force supplied by the caller. A rejected operation leaves the body unchanged. `FixedClock` schedules integer tick credits independently of rendering; callbacks supply each tick's loads. Retained backlog and callback failures remain observable.

`asc-session::body_with_spawn_basis` accepts an explicit centre-of-mass position and mass/inertia properties with the selected spawn basis. It does not treat marker height as solved chassis height or convert car.ini dimensions into a confirmed physical tensor. Native suspension equilibrium, tyre/drivetrain forces, collision response and a controlled car-driving loop still need implementation and validation. See the [native kernel contract](FORMATS.md#native-rigid-body-and-fixed-step-path).

## Native spawn and placement preflight

Read canonical track markers, then check a vehicle footprint against static road geometry:

```sh
cargo run --locked -p asc-app -- spawns ./assettocorsa/content/tracks/drift/models.ini
cargo run --locked -p asc-app -- place ./assettocorsa/content/cars/bmw_m3_e30 ./assettocorsa/content/tracks/drift/models.ini ./assettocorsa/content/tracks/drift/data/surfaces.ini --defaults ./assettocorsa/system/data/surfaces.ini --spawn AC_PIT_0
```

Use matching model/surface paths for the selected layout. `place` accepts the car reader's `--data auto|packed|unpacked` and `--compound N` options; the default marker is `AC_PIT_0`. Both commands work without Python or a GPU. The node-only KN5 path validates complete supported structure while retaining node transforms instead of vertex/index buffers.

Supported marker names are exact-case `AC_PIT_N`, `AC_START_N` (no leading zeroes except index 0), and `AC_HOTLAP_START_0`. Active ancestry applies independently of render visibility. A transform plus its same-named attached mesh is one marker; genuine duplicate names are omitted with errors. The source basis must be finite, upright, rigid and right-handed; it is retained as left/up/forward vectors without an Euler conversion. Native model-level rotations remain unsupported.

Placement transforms the vehicle's four wheel offsets through the selected left/forward basis, then probes vertically down up to 20 metres. The footprint uses the marker plane; wheel BASEY, tyre radius and graphics offsets do not establish a solved chassis height. The report contains typed contacts, all_wheels_supported, diagnostics, has_errors and `simulation_ready: false`. It does not move a car, solve suspension equilibrium or run dynamics. Missing/ambiguous selected markers are fatal. Reports containing errors print JSON and return a nonzero process status; inspect the JSON even when contacts exist.

The local Drift configuration has 35 usable markers and a duplicate `AC_START_3` error. At `AC_PIT_0`, BMW M3 E30 has four ROAD support hits roughly 0.554 metres below the footprint plane, while has_errors remains true for that separate duplicate. This is geometric evidence only. See the [native placement contract](FORMATS.md#native-spawn-and-placement-path).

## Native static ground support

The native `ground` command reads the selected static model declarations and explicit surface definitions without Python or a GPU:

```sh
cargo run --locked -p asc-app -- ground ./assettocorsa/content/tracks/drift/models.ini ./assettocorsa/content/tracks/drift/data/surfaces.ini --defaults ./assettocorsa/system/data/surfaces.ini --probe 0 100 0 --distance 1000
```

Choose the matching `models_LAYOUT.ini` and `LAYOUT/data/surfaces.ini` for a layout. `--defaults` is optional; its complete surface definitions load first, then the selected file replaces complete definitions with matching keys. Both explicitly supplied files must exist and be valid. Add multiple `--probe X Y Z` arguments to query up to 4096 positions; `--distance` applies to all probes. Coordinates and distances are in metres. A probe returns the nearest supported triangle below the origin, including its height, upward geometric normal, model/mesh identity and recorded surface metadata, or `null` for a miss.

Active static physical meshes use a nonzero numeric name prefix and an unambiguous explicit surface key. Visibility/renderability flags do not remove physical support. Positive near-LOD meshes, skinned geometry and WALL meshes are omitted with diagnostics. The supported model translations and composed KN5 node transforms apply before querying. Nonzero model rotations remain fatal errors. The BVH stores double-precision physical positions independently of the float32 render buffers.

The JSON summary reports triangle/mesh counts, skipped vertical or degenerate triangles, diagnostics, BVH readiness and `simulation_ready: false`. This is contact preparation: no chassis equilibrium, suspension or tyre forces, wall response, procedural surface bumps, audio or FFB are calculated. Surface friction is recorded metadata. See [native ground scope and validation](FORMATS.md#native-static-ground-path).

## Native vehicle configuration

The native reader accepts the selected car directory independently of Python or a GPU:

```sh
cargo run --locked -p asc-app -- car ./assettocorsa/content/cars/bmw_m3_e30
cargo run --locked -p asc-app -- car ./assettocorsa/content/cars/bmw_m3_e30 --data packed --compound 0
cargo run --locked -p asc-app -- acd ./assettocorsa/content/cars/bmw_m3_e30/data.acd bmw_m3_e30
```

`car` prints typed mass/inertia dimensions, steering, axle/tyre dimensions, engine torque LUT, gear ratios, brakes, compound, geometric wheel offsets, first-model header and source lines. It requires car.ini, suspensions.ini, tyres.ini, engine.ini, drivetrain.ini, brakes.ini, lods.ini and the referenced torque LUT. Inertia dimensions are not a physical inertia tensor. The output explicitly reports `simulation_ready: false`; this command does not launch a native driving session or simulate original vehicle dynamics.

`--data auto` selects an existing unpacked data directory as a whole; otherwise it selects data.acd. When both sources exist, auto warns and selects unpacked. Missing files are never filled from the other source. `--data packed` or `--data unpacked` selects that source explicitly. A default tyre compound is required unless `--compound 0..99` is supplied; missing compound sections are errors. Model references must resolve inside the selected car root, and unpacked data references must remain inside its data directory. Parent traversal, absolute/drive/stream paths and resolved-outside-root symlinks are rejected for selected resources.

The packed reader supports the ordinary folder-keyed record format. The folder identifier is part of that format; renaming a packed car folder can produce invalid decoded data. Native identifiers are 1..128 ASCII letters, digits, underscores or hyphens. The archive is not authenticated, and a wrong key is not guaranteed to produce a dedicated error. `acd` prints only the validated record index and optional header word, without exporting decoded content. No extraction/writing API is provided.

Native text remains strict UTF-8 with an optional BOM. Turbo, unused configuration fields and incomplete simulation scope are reported as warnings. Tyre/suspension/drivetrain behavior, fuel, coast torque, aero, electronics, FFB and audio remain unimplemented in the native path. See the [compatibility record](FORMATS.md#native-vehicle-and-acd-path) for limits and verified scope.

## Native desktop window

The native `ksMultilayer`, `ksMultilayer_objsp` and `ksMultilayer_fresnel_nm` colour path uses txMask plus four detail textures and their recorded scales. Missing/invalid inputs fall back to diffuse preview with a diagnostic. `multilayer_draws` counts enabled draws. Original normal/Fresnel/specular lighting and colour-space parity remain unvalidated; see the [material scope](FORMATS.md#native-multilayer-base-colour).

Inspect full properties and texture slots without rendering:

```sh
cargo run --locked -p asc-app -- materials ./assettocorsa/content/tracks/drift/models.ini
```

```sh
cargo run --locked -p asc-app -- view ./assettocorsa/content/tracks/drift/drift.kn5
```

Left drag orbits, right drag pans and the wheel zooms. Home resets to the initial ROAD focus; F fits the full visible scene; V selects a near-top orthographic view; P switches perspective/orthographic projection; T switches loaded diffuse RGB to diagnostic colours while retaining texture alpha. Escape closes the window. This window inspects static geometry; it does not run the Python driving prototype or native vehicle physics. Rendering responds to window/input events and waits when idle.

Use `--capture reports/native-window.png` to save the first presented surface frame to a new file; create its parent directory first. The window remains open. Use `--captures reports/window-captures` to enable F12 snapshots with unique PNG names in that directory. Existing images are never overwritten. These are GPU readbacks of the client surface rather than captures of window decorations. Surface readback must be supported by the selected driver, otherwise capture reports an error. The initial client size is 1240 by 900 physical pixels; viewport dimensions are limited to 4096 per axis. Presentation currently requires an SDR RGBA/BGRA surface and a desktop display.

Resize recreates surface configuration and depth resources while retaining geometry/material buffers. Zero-size windows do not render. Transient outdated/suboptimal/lost surfaces have bounded reconfiguration paths; adapter/format changes and device failures are not a validated recovery guarantee. `--smoke` presents five frames (four with readback), programmatically resizes the window, changes cameras and diffuse mode, verifies surface pixels and exits. Normal desktop input still needs platform-specific UI validation; consult the compatibility record for tested scope.

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

## Native DWB design geometry

```sh
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 rear 0.025 0 1 0
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/ferrari_458 front -0.02 0 1 0 --rack 0.002 0 0 --data packed
```

Arguments are car directory, explicit `front` or `rear`, signed travel in metres, and three components of a unit travel axis in raw source coordinates. `--rack X Y Z` moves the inner steering anchor in that same frame; it is not a steering-wheel angle or a validated AC steering-rack conversion. `--data auto|packed|unpacked` uses the existing whole-source selection. Duplicate options, invalid numbers, unsupported types and missing hardpoints fail explicitly. The source TYPE selects DWB or [STRUT geometry](#native-strut-design-geometry).

The design upright initially has position zero and identity orientation. Two lengths locate its upper joint relative to the upper chassis mounts, two locate its lower joint, and one locates its steering joint. The solver finds a rigid upright pose preserving those lengths while setting the origin's displacement along the supplied axis. It reports a quaternion, position, each length residual and the travel residual. Reference lengths come from the unmodified source design, not guessed spring dimensions.

This is a position-level geometry diagnostic. It assumes the source hardpoints describe one rigid design upright in common coordinates. It does not apply side mirroring, static camber/toe, track, BASEY or rod-length corrections, derive wheel forces or establish chassis equilibrium. `simulation_ready` and `original_kinematics_validated` remain false. No wheel or suspension animation is connected to `preview` yet.

Travel and steering-anchor displacement are bounded to 0.25 m; the local pose must stay within 0.5 m and 1 radian of its reference. Singular, unreachable or nonconverging requests produce an error rather than a fallback pose. A command within these bounds can still be unreachable. See [solver details](ARCHITECTURE.md#rigid-upright-kinematics) and the [validation record](FORMATS.md#native-dwb-geometry-validation).

## Native STRUT design geometry

```sh
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 front 0.025 0 1 0
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_1m front -0.02 0 1 0 --rack 0.002 0 0
```

STRUT uses two lower-arm lengths and one steering-link length. The fixed `STRUT_CAR` top must lie on a line attached to the upright through `STRUT_TYRE`; that line's reference direction is defined from the explicit zero/identity design pose. Two perpendicular constraints keep the top on the line while allowing telescoping and steering rotation. `STRUT_TYRE` is independent of `WBTYRE_BOTTOM`; they need not coincide. Travel moves the chosen upright origin along the supplied axis, as for DWB.

The report's TYPE is `STRUT`, with three `rest_link_lengths_m`, a separate `reference_strut_length_m`, and a solution containing position/quaternion, three link residuals, two `coaxial_errors_m`, travel residual, positive `strut_length_m`, signed `strut_length_change_m` and iteration count. Length change is geometric extension from the reference, not spring compression or force. Collapsed/reversed strut axes are rejected. The common numeric bounds, source-frame/setup limitations and readiness flags remain the same as for DWB. See [STRUT implementation](ARCHITECTURE.md#telescoping-strut-kinematics) and [validation](FORMATS.md#native-strut-geometry-validation).

## Native suspension velocity and load probes

```sh
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 front 0.025 0 1 0 --rates 0.17 0.025 -0.012 0.009 --load 120 2300 -300 40 -15 25
```

Both DWB and STRUT support optional probes at the solved pose. `--differential` adds a zero-rate, zero-load probe. `--rates HDOT RXDOT RYDOT RZDOT` supplies travel rate and three steering-anchor velocity components in m/s; `--load FX FY FZ TX TY TZ` supplies force in N and torque in N m about the moving upright origin. Either option enables the probe; omitted rates or loads are zero. All vectors use the fixed raw source design frame, including the torque. These are explicit diagnostic inputs, not measured or source-generated vehicle forces. Each option may appear only once.

The added `differential_probe` contains four tangent columns in travel/rack-X/rack-Y/rack-Z order, relative origin/angular velocity, projected generalized loads and an ideal massless balance at the held pose. Angular velocity is in rad/s. Generalized loads are in N because all four controls are translations in metres. The holding travel-actuator load opposes the projected travel load; the reported steering-anchor force has the same sign as the projected rack load. The five topology multipliers correspond to the constraint order described in the [implementation](ARCHITECTURE.md#suspension-differential-and-held-loads).

Support force and torque on the upright exclude the abstract travel actuator. Frame support torque is about the fixed frame origin, whereas upright torque is about the moving upright origin. The frame support reaction also excludes the travel actuator; it is not the total chassis reaction of a physical car. STRUT perpendicular constraints are ideal supports and do not model strut friction, spring force or bending compliance.

Input rates must be finite and within +/-1000 m/s per component. Output origin/point velocities are bounded to 1e6 m/s per component, angular velocities to 1e5 rad/s, and input/projected loads, multipliers and reactions to 1e18 in their reported units. Invalid, unconverged, singular or excessively amplified requests fail before a JSON report is printed. The Rust API also supports velocity at an explicit upright-local point and a point force converted to torque about the upright origin; local point components are bounded to +/-100 m.

Without probe flags, the position report retains its existing shape. A probe does not integrate time, add chassis motion, generate tyre/spring/damping/gravity loads or assign unsprung inertia. `simulation_ready`, `original_kinematics_validated` and `dynamic_vehicle_simulation` remain false. See the [validation record](FORMATS.md#suspension-differential-validation).
