# Roadmap and feasibility

An independent engine that reads Assetto Corsa content is a technically plausible engineering direction. Access to modding tools and an SDK does not provide the simulator source or guarantee an exact reproduction of its physics.

[openOMSI](https://github.com/openOMSI-org/openOMSI) is the inspiration for organizing an independent implementation around documented formats and a public website. It is not an existing engine base for Assetto Corsa.

## Current state

The native Rust/wgpu path includes bounded INI/LUT/ACD and KN5 readers, typed vehicle/suspension data, static track rendering with supported DDS/material subsets, and a keyboard-driving window. Its coupled 3D chassis/upright/wheel solver supports DWB/STRUT geometry, prescribed racks, compliant finite-face road contact, viscous traction and regularized brakes. Explicit source-node bindings animate wheels and nonspinning uprights. The default example registers initial wheel centres and radii to explicitly selected source corners, while masses, frame orientations and force parameters remain synthetic. English documentation, original fixtures, diagnostics and GitHub Pages accompany the implementation.

The separate Python/WebGL research prototype includes a 120 Hz planar drive with geometric ground attachment and a bounded WALL body proxy. That wall response is not native 6DOF collision. Original tyre/drivetrain/setup fidelity, full body collision, original shaders/skins, gamepad/wheel-device input, force feedback, sound, AI and networking remain incomplete or unimplemented.

The local installation includes SDK documentation and UDP server examples. These are research starting points, not resources distributed by openASC.

## Compatibility target

The project target is 1:1 behavioral compatibility with Assetto Corsa (2014), rather than a similar driving experience. Scope includes vehicle physics and assists, controls and FFB, rendering and materials, audio, AI, session rules, original content behavior and network interactions. Community extensions, including CSP, require separately identified compatibility targets.

Use the installed original game as the comparison reference. Record its build, car/track configuration, setup, assists, conditions, device settings and test inputs with each comparison. Compare acceleration and braking traces, steering and yaw response, suspension travel and tyre behavior, contact response, FFB output, matched-camera images, audio events, AI decisions and protocol behavior. Declare per-test tolerances and unexplained differences; passing a simplified model's own fixtures does not establish parity. Exact reproduction remains an unproven objective, without a guaranteed completion date.

The production engine direction is Rust with wgpu rendering, in subsystem crates organised like openOMSI. Native KN5 loading, embedded diffuse DDS textures and approximate alpha/depth rendering now run independently of Python and the browser, tested on real Drift geometry and original fixtures. Native desktop inspection now shares the persistent GPU pipeline with offscreen capture. Native INI/LUT readers, explicit static scene/layout configuration loading, ordinary ACD records and typed packed/unpacked vehicle configuration now join that path. The native car report remains simulation_ready false; original vehicle dynamics are not inferred from reader agreement. Original material fidelity and a fixed-step simulation/platform foundation remain next architectural milestones, followed by subsystem comparisons against the original. Python research tools live under tools/prototype. Porting the current approximate force equations alone would not meet the compatibility target.

## Development stages

| Stage | Deliverable | Acceptance evidence |
| --- | --- | --- |
| 0. Inventory | Car and track folder inventory | Deterministic JSON and tests on original fixtures |
| 1. Configuration | INI/LUT readers and resource references | Documented subset, typed values, and failure-case tests |
| 2. Viewer | One supported KN5 scene, materials, and camera | A local scene renders; unsupported features are diagnosed |
| 3. First drive | One car, one track, and basic input | A repeatable drive using an independently implemented physics model |
| 4. Validation | Tyres, suspension, drivetrain, and FFB | Measurements compared against explicit tolerances |
| 5. Ecosystem | Wider content support, AI, audio, and networking | Separate compatibility evidence for each subsystem |

## Immediate work

The initial configuration subset, resource manifest, and first geometry viewer are implemented and verified with the local Drift track. Supported embedded diffuse textures now render through tested compressed and software paths. Typed track surface definitions and static downward ground queries are now tested on original fixtures and local Drift, including hidden physical roads. A local BMW M3 E30 configuration and Drift AC_PIT_0 footprint now load, with four supported ROAD contacts and an explicit error for a separate duplicate start slot. Source basis/left-right conventions are checked against original synthetic fixtures and the local wheel nodes. An independent fixed-step first-drive loop now passes original mathematical tests and browser checks for acceleration, steering, braking, reverse, pause and reset with local BMW/Drift. Continuous static wall queries and planar contact response now pass original fixtures and three real Drift traces, including one trace with a few bounded contact-event caps. The native path now provides coupled suspension/traction, explicit wheel animation and source footprint registration. Next, improve source frame/setup and force-law calibration, vehicle/skin materials and full convex/6DOF collision with explicit validation tolerances; multilayer/detail/normal material behavior remains separate ongoing renderer work. Keep unsupported features visible in diagnostics and preserve source locations in errors.

## Long-term scope

Full compatibility may require years of development and is not guaranteed. CSP, Python apps, plugins, and exact original physics need separate investigation. No release date or platform support is promised before a working prototype is tested.

Use the [compatibility record](FORMATS.md) for implemented behavior and the [architecture](ARCHITECTURE.md) for proposed module boundaries.

## Content handling

Write original code and document verified behavior. Use original minimal fixtures in tests. Do not commit game executables, models, textures, sounds, SDK archives, or third-party mods. Distribution rights for any future third-party material must be considered separately.

Native contact preparation now reads physical geometry separately from render buffers and provides static vertical support queries through `asc-ground`. Drift/Magione probes agree with the research loader, including surface metadata. These standalone probes do not advance bodies; coupled forces and the native driving window are described below.

Native session preparation now joins canonical rigid spawn transforms, typed vehicle offsets and four ground queries through `asc-session`. The local BMW/Drift footprint agrees with the research path, while its duplicate start-marker error stays visible. Native chassis equilibrium and the controlled driving loop remain work ahead.

A native unconstrained 3D rigid-body kernel now supplies full inertia validation, world momentum integration, force/impulse application and a fixed tick clock. Original analytic fixtures and a CI-run example establish this numerical foundation. Original vehicle mass-property interpretation, equilibrium, force calibration and collision behavior are not inferred from these kernel tests.

Native suspension inspection records STRUT/DWB hardpoints and spring/damper/stop source values with provenance. Independent DWB five-distance and STRUT distance/coaxial geometry solvers now pass analytical and local-source constraint checks. Their differential maps provide relative upright velocities, virtual-work load projection and ideal massless support reactions under an explicitly supplied held load. An explicit fixed-anchor/upright spring mount now shares the passive spring/damper law, producing length gradients and projected loads with tested energy/power consistency. A fixed-frame/fixed-rack dynamics step now advances an explicitly massive DWB/STRUT upright with COM/tensor, gravity and spring loads, passing analytical, energy and constraint checks. A floating chassis can now be coupled to 1..4 mounted uprights with a common acceleration solve, spring recoil and checked Newton/Euler balance; the mounting frames and masses are explicit inputs. Separate axisymmetric wheel masses and spin coordinates can now join that coupled solve, with tested internal motor reaction, gyroscopic transport and power balance. State-dependent force snapshots now support a compliant thin-circle normal law on an explicitly configured static infinite plane, with analytical equilibrium and energy checks. Finite road-face interiors from the native track BVH can now supply that normal support, with explicit spawn placement and selected-face provenance. An optional memoryless anisotropic tangential slip law now transmits wheel motor torque into chassis traction and adds lateral resistance, with tested force bounds and power balance. Scheduled drive/brake fractions now exercise acceleration followed by a passive regularized internal brake torque, with upright reaction and checked power loss. Prescribed source-frame rack position/velocity/acceleration now steers the coupled mechanism and road contact, with actuator reaction/work and bounded smooth-velocity paths. Edge/vertex collision, CCD, static tyre stiction/wheel lock and transient tyre width/patch behavior remain unimplemented. A separate common-midpoint multibody spring step also passes original analytical/energy tests. These components are not automatically connected as an original AC suspension: source frames/setup mapping, original kinematics/force validation, source spring mounts, unsprung-mass/preload assignment, original vehicle calibration, source steering calibration and calibrated drivetrain/brake dynamics, nonlinear laws and contact/tyre response remain work ahead.

The native renderer now assembles a selected car and track in one window/pass with independent rigid draw roots and shared source/upload budgets. A live-control window shares the coupled workbench solver, applying throttle/brake/rack commands and updating the car root from the physical pose. Diagnostic key-handler/native-surface tests and local BMW/Drift captures pass; independent physical keyboard verification remains outstanding. Local BMW/Drift captures and native window readbacks are verified. The static preview still uses an unsolved source marker plane. Native driving instead uses explicit synthetic COM/mount/contact parameters and a visual model mapping; original equilibrium/calibration and vehicle/material fidelity remain unvalidated.

## Native wheel and upright animation

The native Rust/wgpu driving window now supports explicit source-node bindings for wheel spin, steering and suspension travel. The BMW M3 E30 example maps four wheel and four nonspinning upright subtrees, preserving source rest pivots while applying chassis-relative coupled-physics deltas. Original fixtures verify both transforms and changed native surface pixels. This advances the native path beyond whole-car motion; it does not add animation to the separate browser prototype. Source setup/frame/contact alignment calibration, original materials/skins and full native body collision remain next work, together with drivetrain/tyre fidelity and later device/FFB/audio/AI/network systems.

## Source footprint and tyre dimensions

The native drive now registers its four initial physical wheel centres to explicitly selected typed source corners and uses selected compound tyre radii. Chassis-frame model-pivot differences remain visible in diagnostics (about 11.42/19.57 mm front/rear for local BMW). This removes arbitrary sample wheelbase/track/height/radius values from the default native path, while preserving explicit raw-frame orientations and synthetic masses/forces. Full setup, source force-law/mass mapping, original-game comparison and native body collision still require development and validation.
