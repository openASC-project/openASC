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

Without probe flags, the position report retains its existing shape. These load probes do not integrate time, add chassis motion or generate tyre/gravity loads or unsprung inertia. Spring/damping generation requires the separate explicit spring option below. `simulation_ready`, `original_kinematics_validated` and `dynamic_vehicle_simulation` remain false. See the [validation record](FORMATS.md#suspension-differential-validation).

## Native suspension spring probes

```sh
cargo run --locked -p asc-app -- suspension-motion ./assettocorsa/content/cars/bmw_m3_e30 front 0.025 0 1 0 --rates 0.17 0.025 -0.012 0.009 --spring 0 0.6 0 0.07 -0.08 0.1 0.65 12000 --damping 80 20 0.05 140 30 0.07
```

The mount and law in this example are synthetic diagnostic inputs, not an Assetto Corsa suspension calibration. `--spring AX AY AZ BX BY BZ REST K` supplies a fixed-frame anchor A, upright-local attachment B, rest length in metres and stiffness in N/m. It enables the differential probe and adds `explicit_spring_probe`. The spring is a bilateral linear element: it can push or pull. No unilateral spring seat, bump stop, preload correction, linkage-arm mounting or ARB is inferred.

`--damping CS CF CK RS RF RK` sets compression slow/fast slopes in N s/m and knee speed in m/s, followed by the corresponding rebound values. Compression means decreasing element length. The continuous piecewise law uses the slow slope up to its knee and the fast slope beyond it. Omitting this option explicitly selects zero damping. Supplying damping without a spring, duplicate options, malformed values or negative slopes/stiffness fails.

The sample reports the attachment in the fixed frame, its velocity, actual element length/extension/rate, four length gradients in travel/rack-X/rack-Y/rack-Z order, elastic/damping tension, potential energy, dissipated power, upright force/torque and generalized loads. Positive tension pulls the upright toward A. Point velocity includes upright rotation, so the length gradients and resulting force ratios depend on the pose and mounting. `parameters` records the supplied rest length, stiffness and both damper branches.

The spring sample's frame force/moment is the direct reaction at A only. Its sibling `held_massless_balance` contains the separate linkage support and abstract travel-actuator loads required to hold that spring load. Direct spring reactions exclude linkage supports; support reactions exclude the travel actuator. The spring probe is evaluated separately from the manually supplied `--load` probe; those reports are not summed. Neither is a physical vehicle equilibrium or a dynamic step.

Mount coordinates are bounded to +/-100 m per component; rest length to 1e-6..10000 m; stiffness/slopes to 0..1e9 in their units; damper knees to 0..1000 m/s. Coincident endpoints (separation below 1e-9 m), separation above 10000 m, out-of-range rates/loads and singular linkage states fail explicitly. With a spring, `generated_tyre_or_spring_loads` is true, while `automatic_source_parameters`, `source_force_mapping_validated` and `dynamic_vehicle_simulation` remain false. See [implementation](ARCHITECTURE.md#explicit-spring-linkage-coupling) and [validation](FORMATS.md#spring-linkage-coupling-validation).

## Native fixed-frame suspension dynamics

```sh
cargo run --locked -p asc-app -- suspension-run ./assettocorsa/content/cars/bmw_m3_e30 front examples/suspension-travel.json
cargo run --locked -p asc-app -- suspension-run ./assettocorsa/content/cars/ferrari_458 rear examples/suspension-travel.json --data packed
```

This CPU workbench advances one massive rigid upright with ideal massless DWB or STRUT links. The linkage frame and rack displacement remain fixed. Travel is the single dynamic coordinate; upright position/orientation are solved from the constraints at every integration stage. Spring forces, gravity, full COM inertia and geometry-induced acceleration determine motion. This is a native suspension subsystem, not a complete car or rendered driving session.

Use the repository's [example configuration](https://github.com/openASC-project/openASC/blob/main/examples/suspension-travel.json) as the schema reference.

The selected car supplies only raw design hardpoints through the existing whole-source reader. The example JSON supplies synthetic mass, tensor, COM and spring values; gravity is explicitly zero to isolate spring response. No parameter is automatically inferred from HUB_MASS, INERTIA dimensions, ROD_LENGTH, source spring rates or damper records. Source frame/mirroring/setup interpretation and original-game dynamics remain unvalidated.

Configuration schema 1 requires every field shown in `examples/suspension-travel.json`; unknown/duplicate fields and missing values fail. `travel_axis` is an explicit unit axis in the fixed source frame, and `rack_m` is its fixed steering-anchor displacement. `mass_kg` and `inertia_rows_kg_m2` define the upright mass and physically realizable positive COM tensor, expressed in upright-local axes. `center_of_mass_local_m` is its COM offset from the chosen upright origin. `gravity_m_s2` is a fixed-frame acceleration vector, not a force. Each spring supplies fixed/upright-local mounts, rest length, stiffness and explicit compression/rebound branches as in the spring probe. An empty spring array disables springs.

`initial_travel_m` and `initial_rate_m_s` seed motion. `step_s` is an explicit midpoint step; `steps` is the number of steps and `record_every` selects saved states. The report always includes initial and final states. `external_force_n` and `external_torque_about_upright_origin_nm` are constant components in the fixed frame throughout the run. The torque is about each moving upright origin; this is not automatically converted from a fixed-world application point.

Each recorded sample includes travel/rate/acceleration, effective travel mass, velocity-dependent inertial bias, generalized applied load, upright/COM motion, spring samples and kinetic/potential energy. Potential energy includes springs and gravity; work by the external wrench is not included as a potential. The constraint balance uses applied-minus-inertial loads: its support reactions are dynamic. `travel_actuator_n` is a numerical residual that must be near zero because travel is free, not held by an actuator. Support frame reactions exclude direct spring-anchor reactions, which are listed separately in each spring sample.

The input file is bounded to 16384 bytes, with 1..10000 steps, at most 16 springs and 1024 records including the initial state. Step duration is 1e-6..1/30 s. A small enough step for the chosen stiffness/mass is the caller's responsibility; this explicit method has no universal stability or energy-monotonicity guarantee. All stages must satisfy local geometry and numeric bounds. An invalid initial state, singular/unreachable request or failed later stage aborts the CLI without partial JSON output; the Rust step leaves the previous state unchanged on error.

The existing +/-0.25 m travel, 0.5 m/1 radian local branch and spring limits still apply. Travel rate is bounded to +/-1000 m/s, travel acceleration to +/-1e6 m/s^2, origin/angular accelerations to 1e9 in their units, local-point acceleration to 1e13 m/s^2 and each gravity component to +/-1000 m/s^2. Effective mass must lie within 1e-12..1e24 kg; energy/power magnitudes are capped at 1e30. Body mass/tensor validation and the 1e18 load/reaction bounds also apply.

The report retains `vehicle_simulation: false`, `simulation_ready: false` and `original_force_mapping_validated: false`. There is no moving chassis, wheel spin, tyre/contact response, FFB, automatic source spring calibration or renderer animation. See [dynamics implementation](ARCHITECTURE.md#fixed-frame-upright-dynamics) and [validation](FORMATS.md#fixed-frame-suspension-dynamics-validation).

## Native coupled chassis dynamics

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-suspension.json
```

This CPU workbench couples a floating rigid chassis to one to four massive DWB/STRUT uprights through ideal massless links. A single solve determines the chassis's three COM and three angular accelerations plus each upright's travel acceleration. Racks remain fixed relative to the chassis. Mounted spring forces now produce chassis recoil as well as upright motion. Optional wheel rotation is described below. Finite-road/tangential tyre response, drivetrain, control input and renderer animation are not connected.

The [example JSON](https://github.com/openASC-project/openASC/blob/main/examples/chassis-suspension.json) supplies synthetic masses, COM tensors, mounting frames and springs. Its zero gravity deliberately isolates internal motion. The right-side frame rotations are example proper rotations, not a verified AC side/mirroring/setup conversion. Source car data supplies raw geometry only; no source mass, preload, track, BASEY, ROD_LENGTH or static toe/camber mapping is inferred.

Schema 1 requires every field in the example and rejects unknown/duplicate fields. `chassis` defines its mass, body-local tensor about its COM, world COM pose/velocity/angular velocity and external world COM wrench. `gravity_world_m_s2` applies to every body. Each `uprights` entry supplies a `car_directory` resolved relative to the JSON's directory, explicit `front`/`rear`, `data` selection (`auto`, `packed` or `unpacked`), source travel axis, fixed rack displacement, mass/tensor/COM, initial travel/rate, springs and external world COM wrench.

`frame_position_in_chassis_m` is the source frame origin relative to the chassis COM in chassis-local axes. `frame_orientation_xyzw` is a unit quaternion mapping source axes into chassis-local axes; it cannot represent a reflection. Upright COM and spring attachment coordinates remain in their documented local/source frames. Every frame and parameter must be supplied explicitly. A different per-corner car/axle may be selected, with at most four total sources.

All external wrenches here use world components and torque about the corresponding body's COM, including the uprights. This differs from the fixed-frame workbench's torque about the upright origin. Configured wrench components remain constant throughout each step; generated normal support below is recomputed from stage state. `step_s`, `steps` and `record_every` control the same initial/final-state recording behavior. The JSON input is bounded to 65536 bytes, 1..4 uprights, 16 springs per upright, 1..10000 steps and 256 saved records including the initial state.

Samples contain the chassis and upright world body states/accelerations, relative travel states, source geometry poses and spring samples in the attached source frames. A spring's `fixed_anchor_m` is fixed relative to the attached frame, which now moves with the chassis. Relative spring velocities exclude chassis rigid motion. Dynamic support reactions are reported both in source coordinates and as world forces/moments on the chassis; direct spring reactions are separate. Upright support `travel_actuator_n` is only a near-zero numerical residual of the free travel coordinate.

The report also records total mass/COM, linear momentum, angular momentum about the world origin, kinetic energy, spring/gravity potential and dissipation. Angular momentum about the world origin includes orbital momentum and differs from each body's spin momentum about its COM. The explicit Lie-midpoint method is second order and does not preserve total momentum/energy exactly at finite step size. See the measured convergence in [validation](FORMATS.md#coupled-chassis-dynamics-validation).

All stages must satisfy body/geometry/spring/load bounds. A step or a failure in any corner leaves the whole previous system unchanged; the CLI buffers successful results and prints no partial JSON on failure. Duration is 1e-6..1/30 s, with a maximum rotation increment of 0.25 rad per stage update. The coupled mass solve normalizes translation/angular/travel scales and rejects nonpositive or poorly conditioned systems without a fallback. Generalized acceleration components are limited to +/-1e6 in m/s^2 or rad/s^2, upright COM acceleration to 1e13 m/s^2 and angular acceleration to 1e9 rad/s^2. Existing world body, local geometry, gravity, spring and 1e18 load bounds still apply; total momentum magnitudes per component are capped at 1e24 and energy/power at 1e30.

`automatic_source_frames_mass_or_force_parameters`, `original_vehicle_dynamics_validated`, `simulation_ready` and `renderer_animation` remain false. It does not establish source frame/side fidelity, a car resting on a road, original handling or native driving. See [coupled equations](ARCHITECTURE.md#floating-chassis-and-mounted-uprights).

## Native wheel rotation and motor reactions

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-wheels.json
```

Each upright may now include an optional `wheel` object, as shown in the [synthetic wheel example](https://github.com/openASC-project/openASC/blob/main/examples/chassis-wheels.json). Missing or null means no separate wheel and preserves the earlier workbench behavior. `wheel_spin_dofs` reports the actual number of attached wheels. A wheel adds its own mass, COM inertia and spin coordinate to the coupled solve; it is not just a visual angle.

`center_in_upright_m` locates the wheel COM in upright-local coordinates. `axle_in_upright` is an explicit unit axle in those axes. The wheel is axisymmetric: `axial_inertia_kg_m2` is its COM moment around that axle and `transverse_inertia_kg_m2` is either perpendicular principal moment. All are positive and must satisfy the existing physically realizable tensor checks, including axial moment no greater than twice the transverse moment within tensor tolerance. Mass and tensor are additional to the upright, so the supplied upright mass/tensor must exclude this wheel. No mass or inertia is subtracted automatically.

`initial_angle_rad` and `initial_rate_rad_s` are wheel rotation relative to the upright about the chosen axle. Angles remain unwrapped in the state, bounded to +/-1e9 rad; phase is reduced modulo a revolution only when constructing the body orientation. Relative rate is bounded to +/-1e5 rad/s. Axle direction determines the signs; no left/right, forward-rolling or AC axle convention is inferred. Mount coordinates, mass/tensor and all existing body/geometry bounds still apply.

`external_force_world_n` and `external_torque_about_com_world_nm` are an external world wrench about the wheel COM. `drive_torque_nm` is a separate internal motor torque between the wheel and upright. It applies opposite torque to the upright and contributes mechanical power `drive_torque * relative_spin_rate`. This models that explicit motor pair; it does not infer the engine/gearbox/CV-shaft torque path or a passive brake law. All components remain constant during a step. There is no tyre, road traction, rolling resistance, bearing friction or FFB generation.

Each attached wheel's sample includes its own world body state/COM, axis, relative angle/rate, spin acceleration, world acceleration/angular acceleration and spin-equation residual. `motor_reaction_on_upright_world_nm` is opposite the supplied drive torque. `bearing_torque_about_wheel_com_on_upright_world_nm` separately reports the bearing reaction. `torque_about_upright_origin_world_nm` combines bearing and motor reactions and shifts the wheel force to the upright origin; do not add the motor reaction to it again. Wheel mass, orbital/angular momentum and kinetic energy are included exactly once in system totals.

The normalized system has 7..14 active coordinates depending on upright/wheel counts. Chassis and travel coordinates couple to wheel inertia, including gyroscopic transport as the axle turns. Generalized spin acceleration shares the +/-1e6 rad/s^2 bound. The phase update does not use the chassis's 0.25 rad rotation-increment limit because axisymmetry permits direct phase reconstruction; fast precession can still require a smaller step. Every stage must validate, including final spin/rate bounds, before any body or wheel state commits.

The example deliberately preserves the prior total mass by changing each 35 kg upright into a 15 kg upright plus a separate 20 kg wheel. These values and inertias remain synthetic. With internal motor work, total mechanical energy may rise even while dampers dissipate energy; finite-step integration and sampled work/dissipation have measurable error. `simulation_ready` and `original_vehicle_dynamics_validated` remain false. See [equations](ARCHITECTURE.md#axisymmetric-wheel-rotors) and [validation](FORMATS.md#axisymmetric-wheel-dynamics-validation).

## Native normal support on a plane

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-normal-plane.json
```

This benchmark adds normal wheel support, gravity and damping to the coupled chassis. The [example configuration](https://github.com/openASC-project/openASC/blob/main/examples/chassis-normal-plane.json) uses explicit synthetic radii, stiffness/damping, initial placement and spring preload. Its plane is infinite and fixed in world space; it does not load or approximate a selected track's triangles. There is no tyre width, patch distribution, friction, traction, rolling resistance or body/road collision.

Top-level `support_plane` supplies `point_world_m` and a unit `normal_world`. Each participating wheel supplies `normal_contact` with `radius_m`, `stiffness_n_m`, `damping_n_s_m` and `max_penetration_m`. Supply the plane and at least one wheel model together; unknown/missing fields or unused plane/model combinations fail. Missing/null contact models disable support for that wheel. No value is inferred from tyres.ini, pressure, surfaces.ini or original-game force laws.

The wheel is a thin circle in the plane perpendicular to its axle. The lowest circle point relative to the support normal determines the signed gap, so camber affects both support height and moment. Positive gap produces zero force. At zero/negative gap, elastic force is stiffness times penetration and damping opposes the circle point's normal velocity. Total normal force is clamped to zero when the unconstrained spring/damper result would pull toward the plane. Wheel spin alone creates no normal damping or axle torque. A normal force can still create a camber moment; there is no tangential traction to turn motor torque into forward acceleration.

The force is compliant: finite penetration stores elastic energy. It is not a hard nonpenetration constraint, continuous collision detection or a tyre/rim volume. Exceeding the explicit penetration limit fails the stage instead of projecting the vehicle to a new position. An axle nearly parallel to the support normal also fails, rather than selecting an arbitrary circle point. This model is intended for small deformations near a supported configuration.

Radius is bounded to 1e-4..10 m, stiffness to 1..1e9 N/m, damping to 0..1e9 N s/m and penetration limit to 1e-6..radius/2 m. Existing world body/rate/load bounds still apply. The example uses a 0.3 m radius, 200000 N/m stiffness, 1500 N s/m damping and an 0.08 m penetration limit. Step size must resolve the chosen stiffness/mass; neither the fixed step bounds nor the limit guarantee stable contact at arbitrary speed.

`normal_contacts` records one slot per upright, with null for disabled models. Samples include wheel/plane points, gap, penetration, normal velocity/load, world COM wrench, potential energy and actual dissipated power. A geometrically touching wheel may have zero force while separating. Record-level `mechanical_energy_with_normal_contact_j` includes contact energy in addition to the core sample's kinetic/gravity/suspension energy; `total_dissipated_power_w` includes both contact and suspension damping. The core sample fields retain their previous meanings. Contact damping after force clamping is accounted by the actual energy removed, which can differ from damping coefficient times squared normal speed.

The Rust `kinematics` API returns force-independent world body snapshots. `sample_with_forces` and `step_with_forces` take a provider returning chassis/upright/wheel loads. During a step it sees offsets 0, dt/2 and dt; final forces validate/report the final state, while only the midpoint derivative advances it. These are three evaluations of one step, not three physical steps. The normal model is stateless and is recomputed at all stages. Callback errors leave the whole mechanical system unchanged; callback-owned side effects are outside that guarantee.

`new_with_forces` validates initialization with the actual applied/support loads. Use it when support balances substantial spring preload; the legacy `new` validates an unloaded initial sample. The plane workbench uses force-aware initialization.

The CLI reports the number of normal models and `normal_contact_uses_track_geometry: false`; original-validation, renderer and simulation-ready flags remain false. Force errors include the upright slot and, during stepping, the callback's stage offset. See [equations](ARCHITECTURE.md#state-dependent-forces-and-normal-support) and [validation](FORMATS.md#normal-contact-and-force-stage-validation).


## Native normal support on track faces

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-normal-track.json
```

The [track example](https://github.com/openASC-project/openASC/blob/main/examples/chassis-normal-track.json) runs the same independent coupled chassis model on finite physical road-face interiors from local Drift geometry. It is a JSON workbench, with no rendered driving or input control. Masses, mounting frames, spring/contact laws and COM offset are synthetic inputs; this is not a verified original vehicle setup.

Replace `support_plane` with `support_track`, containing `models_ini`, ordered `surface_files` (1..16 paths), and `min_normal_y` (1e-4..1). Paths are relative to the JSON. Load the installed system surface definitions first, then selected track definitions; complete same-key definitions override earlier files. Omitting the defaults can omit physical road meshes whose keys the track file does not redefine. The example uses a minimum normal Y of 0.5. Wheel normal models retain their explicit radius, stiffness, damping and penetration limit.

Optional `initial_spawn` contains a marker `name` and `chassis_com_offset_in_spawn_m`. Set chassis position to zero and orientation to identity when using it. Offset components are metres along marker left/up/forward, bounded to +/-100 m. The example uses `AC_PIT_0` and `[0,-0.264,0]`, an explicitly chosen placement for this synthetic setup on this installation; it is not a general ride-height solution. Missing/ambiguous markers and non-rigid bases fail. Other marker diagnostics remain visible.

Each saved state adds parallel `normal_contacts` and `track_contact_faces` arrays: null means no selected face for that wheel. A selected face can still have positive gap and zero force. Face metadata includes source triangle index, model, mesh, surface key, friction/damping metadata and definition provenance. `track_geometry` and `track_diagnostics` retain loader information; `initial_spawn_diagnostics` retains marker diagnostics. `normal_contact_uses_track_geometry` is true, while `edge_vertex_response`, `continuous_collision_detection` and `simulation_ready` remain false.

The support point must project inside the finite face; the plane is not extended beyond it. A shared seam supplies one force. Unsupported backside overlap, excessive penetration, dense queries or invalid geometry fail without partial JSON. Edge/vertex impacts, walls/body clearance, swept collision, tyre width/tangential grip and control/render integration remain work ahead. See [selection and bounds](ARCHITECTURE.md#finite-road-face-normal-support) and [validation](FORMATS.md#finite-track-face-contact-validation).


## Native tangential contact and traction

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-traction-track.json
```

The [traction example](https://github.com/openASC-project/openASC/blob/main/examples/chassis-traction-track.json) adds a memoryless tangential slip law to the finite-face Drift workbench. Synthetic rear motors apply opposite signed 100 N m torques because the example right-side mounting frames reverse their axle directions. All four wheels have friction. This moves the coupled native chassis through wheel contact forces; it is still a JSON dynamics workbench with fixed racks, no player controls and no rendered motion.

Each enabled `wheel.tangential_contact` requires that wheel's `normal_contact`. Supply `longitudinal_viscosity_n_s_m`, `lateral_viscosity_n_s_m`, `longitudinal_friction` and `lateral_friction`; all fields are required and unknown fields fail. Viscosities lie in 0..1e7 N s/m, coefficients in 0..10. These are synthetic independent-law parameters and are not extracted from AC tyre LUTs. Track mode scales both directional limits by the selected surface FRICTION, supported within 0..10. Plane mode instead requires an explicit `support_plane.friction_scale` in 0..10. A zero coefficient, scale or normal load removes its corresponding force.

Each state adds parallel `tangential_contacts`, with direction vectors, slip velocities, component forces/limits, surface scale, common ellipse saturation scale, world force/COM torque and mechanical/dissipated power. Null means that wheel has no enabled tangential provider or selected track face. `tangential_dissipated_power_w` sums wheel losses, and `total_dissipated_power_w` includes them. `tangential_contact_wheels` counts enabled models. Reports still mark original dynamics and simulation readiness false.

At zero material-point slip the model produces zero force; it has no static-friction hold, transient contact patch or independently generated aligning torque. Normal force is not assigned from static axle weight. Wheel angular velocity, normal compression and chassis motion all affect the actual contact wrench. Use smaller steps when increasing viscosity or lowering wheel inertia; the explicit solver has no universal stability guarantee. See [force and power conventions](ARCHITECTURE.md#tangential-contact-and-wheel-traction) and [validation](FORMATS.md#tangential-contact-validation).


## Native scheduled wheel controls and brakes

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-accelerate-brake.json
```

The [accelerate/brake example](https://github.com/openASC-project/openASC/blob/main/examples/chassis-accelerate-brake.json) runs four seconds on local Drift with synthetic masses, mounts, contact parameters and brakes. Rear motors accelerate for one second; drive ramps to zero by 1.2 s, then all-wheel brake demand ramps to one by 1.4 s. This is a repeatable native dynamics workbench with fixed racks, not a player-controlled or rendered drive.

Optional top-level `controls` is an array of 1..64 keys. Each key requires `time_s`, `drive_fraction` and `brake_fraction`; unknown fields fail. Time starts exactly at zero, increases strictly and lies in 0..1e6 seconds. Drive lies in -1..1 and brake in 0..1. Between keys both fractions interpolate linearly; after the final key they hold. With no timeline, drive is one and brake zero, preserving existing constant-motor behavior. A negative drive fraction reverses the configured signed motor torque; it does not implement a gearbox. At least one wheel is required.

Each optional `wheel.brake` contains `maximum_torque_nm` (0..1e6) and `transition_rate_rad_s` (1e-3..1e3). Brake torque opposes the wheel's spin relative to its upright, reaches its demand-scaled limit above the transition rate, and decreases linearly to zero near rest. It is added as an internal pair with the opposite upright reaction. Brakes work while airborne as well as on a road. Parameters are explicit synthetic inputs, not automatically mapped from AC brake data.

Controls and brake torque are evaluated at initial, midpoint and final stage times. Reports add the current `controls`, parallel `wheel_brakes`, `motor_power_w`, and `brake_dissipated_power_w`; total dissipation includes brake losses. The existing per-wheel sample field `drive_torque_nm` now describes the net internal actuator torque, including the brake. Configured nominal motor torque remains in `explicit_configuration`. The report marks `scheduled_wheel_controls`, counts `wheel_brake_models` and keeps `static_wheel_lock` false.

This continuous regularization approaches rest with a small residual rate. It supplies neither static wheel locking nor slope holding, ABS, thermal fade or hydraulic timing. Small-step passivity checks do not guarantee stability at an arbitrary explicit step. Increasing capacity or decreasing transition rate stiffens the low-speed response; reduce the step accordingly. See [torque and power conventions](ARCHITECTURE.md#scheduled-wheel-actuation-and-passive-brakes) and [validation](FORMATS.md#scheduled-brake-validation).


## Native prescribed rack steering

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-steering-track.json
```

The [steering example](https://github.com/openASC-project/openASC/blob/main/examples/chassis-steering-track.json) combines rear drive, front rack motion and braking on local Drift. It runs three seconds with synthetic masses, source-frame mounts, force parameters and actuator paths. Front rack displacement reaches 0.01 m, holds, then returns to zero while drive is removed and brakes engage. This is still an offline JSON workbench without keyboard/gamepad control or rendered motion; no original steering/setup calibration is implied.

Add optional `steering` to an `uprights` entry. It requires `axis_in_source` (a unit vector in that corner's raw linkage frame) and `keys`, each with `time_s`, `displacement_m` and `velocity_m_s`. The scalar displacement is added along that axis to the existing `rack_m` base. Choose per-corner axis/sign explicitly; the example's proper right-side mounting rotations and source coordinates are synthetic conventions, not verified AC side/setup mapping.

Supply 1..64 keys starting at time zero, separated by at least 1e-6 s, within 0..1e6 s. Initial and final key velocities must be zero. Cubic Hermite interpolation gives continuous position/velocity and analytical acceleration. It rejects scalar excursions beyond +/-0.25 m, +/-1000 m/s or +/-1e6 m/s^2 even between keys; combined rack positions and local geometry have their own stage checks. Displacement holds after the last key. Acceleration may jump at a key; this is not a jerk-limited steering actuator. Use smaller steps to check convergence for faster inputs.

Reports mark `fixed_racks` false and count `prescribed_racks`. Each upright reports the actual source-frame `rack_motion`, required actuator force and power. The sample's `rack_actuator_power_w` sums corners. Record-level `actuator_input_power_w` sums motor and rack input power; brake/contact/spring losses stay in total dissipation. A rack can return energy to its ideal drive. Force models see the steered wheel axes and velocities at every stage, so contact forces turn the chassis through the coupled mechanism.

The native core rejects a step that jumps rack position or velocity, fails a rack/force callback, or exceeds geometry/numeric bounds, preserving the prior mechanical state. The CLI prints no partial JSON on a failed run. Live steering input, steering assist/FFB, source rack/steering-wheel mapping, collision edges/CCD and calibrated tyre behavior remain work ahead. See [rack dynamics](ARCHITECTURE.md#prescribed-rack-dynamics-and-steering) and [validation](FORMATS.md#prescribed-rack-validation).


## Native keyboard-driving window

```sh
cargo run --release --locked -p asc-app -- drive examples/native-drive.json --captures reports/native-captures
```

This opens the Rust/wgpu BMW M3 E30/Drift driving workbench using your local installation. It shares the coupled chassis/upright/wheel and finite road-contact solver with `chassis-run`, and updates the car's GPU root from its physical COM pose. It starts paused. Use the optimized build for interactive operation; the example runs physics at 2000 Hz.

| Input | Action |
| --- | --- |
| Enter | Pause/resume; clears pedal/steering commands |
| W | Apply the configured signed motor torque |
| S or Space | Apply regularized wheel brakes |
| A / D | Left/right rack target; both together centre the target |
| R, while paused | Toggle forward/reverse motor direction |
| F5 | Restore initial physical state, forward direction and paused input |
| C | Toggle camera following |
| Mouse left/right drag, wheel | Orbit, pan, zoom |
| Home | Fit the car at its current position |
| F12 | Save a new PNG when `--captures` is supplied |
| Escape | Close |

Focus loss, hiding/minimizing, resize/DPI change and suspension pause the simulation and clear held commands. Resume explicitly and press the controls again; key autorepeat does not restore a cleared pedal. No elapsed physics is silently skipped while running. Work per frame is bounded; a backlog above a quarter second pauses at the last successful tick. Invalid physics also pauses with its diagnostic in the title/console. Pause discards the pending wall-clock debt while retaining mechanical velocities and the committed pose. Reset restores the original state.

The [window configuration](https://github.com/openASC-project/openASC/blob/main/examples/native-drive.json) is at most 8192 bytes and rejects unknown fields. `dynamics_config` resolves relative to it and supplies all mechanical inputs. It must configure four wheels with normal/tangential contact and brakes, explicit steering axes, one common car/data source and `support_track`. `step_s` must be the reciprocal of an integer rate in 30..10000 Hz. Offline `steps`/`record_every` still validate but do not limit interactive duration. Scheduled drive/brake keys are overridden by live input; steering keys supply the initial rack state and axes, with later motion supplied by the live actuator.

`model_origin_in_chassis_m` and `model_orientation_in_chassis_xyzw` explicitly map model coordinates into the physical chassis COM frame, then the reader's source graphics offset/pitch is composed. The example's 0.2-m vertical offset is a synthetic visual mapping, not a verified original COM/setup conversion. Optional `compound` selects the vehicle reader compound. Car/track source diagnostics, including unrelated duplicate markers, remain visible in console metadata.

The `steering` object specifies `displacement_limit_m` (0..0.1), `frequency_rad_s` (0.1..100), `acceleration_limit_m_s2` (1e-4..10) and `rate_limit_m_s` (1e-4..1). Each physical tick uses a critically damped target response, bounded acceleration and endpoint rate, with a consistent quadratic rack position/velocity prescription through the step. The displacement limit bounds the commanded target, not a hard position clamp. Geometry and numeric bounds still apply. W and R control explicit motor torques directly; source engine/clutch/gearing, ABS and original steering/FFB are not implemented by these controls.

`--fallback`, `--diagnostic` and `--rgba-textures` retain native adapter/material fallback choices. F12 captures are new GPU readbacks rather than screenshots of window decorations. `--smoke` runs a diagnostic 2.4-second throttle/steer/brake sequence through the shared native key handler and exits after four presented/read-back states; adding `--captures` saves `drive-smoke-0.png` through `drive-smoke-3.png` and refuses existing names. This automation verifies the software control path and rendered motion, not independent physical keyboard use.

Optional `articulation` binds explicit source subtrees to physical upright slots. The example maps `WHEEL_LF/RF/LR/RR` to spinning wheels and `SUSP_LF/RF/LR/RR` to nonspinning uprights, with slots 0/1/2/3 respectively. Each entry requires `node`, zero-based `upright`, and `kind` (`wheel` or `upright`). Omit the array, or use an empty array, to move the entire car as one root. Up to 16 unique, active, visible and disjoint subtrees are accepted; missing/duplicate names, overlapping parent/child selections and absent physical slots/wheels fail before opening the window. Source names and corner order are explicit configuration, not inferred conventions.

Bindings preserve source rest placement and use the source node pivot as the visual rotation centre. Chassis-relative physical position and orientation changes drive suspension travel, steering and wheel spin; upright groups do not inherit rotor spin. Both kinds use physical wheel-centre travel, so each requires a wheel in the selected slot. Reset restores the same rest mapping. This prevents a synthetic physical mount offset from making a visual wheel orbit its hub. It does not calibrate original camber/toe, axle frames, source animation semantics or mesh-to-contact alignment; the synthetic example may still show visual road-contact mismatches. Geometry and textures stay on the GPU while group transforms update.

Body/wall collision, edge/vertex contact/CCD, source-calibrated vehicle forces, gamepad/FFB, sound, AI and networking remain incomplete. The separate browser prototype still has its own planar WALL response; that response is not available in this native path. See [window architecture](ARCHITECTURE.md#native-driving-window) and [validation](FORMATS.md#native-driving-window-validation).
