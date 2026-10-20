# Roadmap and feasibility

An independent engine that reads Assetto Corsa content is a technically plausible engineering direction. Access to modding tools and an SDK does not provide the simulator source or guarantee an exact reproduction of its physics.

[openOMSI](https://github.com/openOMSI-org/openOMSI) is the inspiration for organizing an independent implementation around documented formats and a public website. It is not an existing engine base for Assetto Corsa.

## Current state

Implemented: a native Rust KN5 reader and wgpu offscreen diagnostic renderer, plus the Python research path with inventory, INI/LUT readers, static model diagnostics, supported KN5 geometry/hierarchy/material definitions, and a first local WebGL viewer with camera and mesh inspection. Original fixtures, English documentation, and the GitHub Pages site accompany the implementation. Supported diffuse DDS textures, approximate alpha/depth behavior, typed surface definitions and BVH-accelerated vertical ground support queries, ordinary ACD data reading, a typed vehicle configuration subset and geometric spawn placement preflights are also implemented. A local independent keyboard-driving prototype now includes 120 Hz planar forces, geometric ground attachment, a rendered car and chase camera. Static WALL contact now uses an upright collider-bounds proxy, continuous queries and planar impulses. Not implemented: full convex/6DOF body response, suspension dynamics, original tyre/drivetrain behavior, wheel animation, original shaders, full multilayer/detail/normal rendering, wheel-device input, force feedback, audio, or networking.

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

The initial configuration subset, resource manifest, and first geometry viewer are implemented and verified with the local Drift track. Supported embedded diffuse textures now render through tested compressed and software paths. Typed track surface definitions and static downward ground queries are now tested on original fixtures and local Drift, including hidden physical roads. A local BMW M3 E30 configuration and Drift AC_PIT_0 footprint now load, with four supported ROAD contacts and an explicit error for a separate duplicate start slot. Source basis/left-right conventions are checked against original synthetic fixtures and the local wheel nodes. An independent fixed-step first-drive loop now passes original mathematical tests and browser checks for acceleration, steering, braking, reverse, pause and reset with local BMW/Drift. Continuous static wall queries and planar contact response now pass original fixtures and three real Drift traces, including one trace with a few bounded contact-event caps. Next, implement wheel animation, improve car skin/material loading, and develop suspension/tyre dynamics and full convex/6DOF collision with explicit validation tolerances; multilayer/detail/normal material behavior remains separate ongoing renderer work. Keep unsupported features visible in diagnostics and preserve source locations in errors.

## Long-term scope

Full compatibility may require years of development and is not guaranteed. CSP, Python apps, plugins, and exact original physics need separate investigation. No release date or platform support is promised before a working prototype is tested.

Use the [compatibility record](FORMATS.md) for implemented behavior and the [architecture](ARCHITECTURE.md) for proposed module boundaries.

## Content handling

Write original code and document verified behavior. Use original minimal fixtures in tests. Do not commit game executables, models, textures, sounds, SDK archives, or third-party mods. Distribution rights for any future third-party material must be considered separately.

Native contact preparation now reads physical geometry separately from render buffers and provides static vertical support queries through `asc-ground`. Drift/Magione probes agree with the research loader, including surface metadata. Native chassis equilibrium, collision response, suspension/tyre force laws and a controlled driving session remain unimplemented.

Native session preparation now joins canonical rigid spawn transforms, typed vehicle offsets and four ground queries through `asc-session`. The local BMW/Drift footprint agrees with the research path, while its duplicate start-marker error stays visible. Native chassis equilibrium and the controlled driving loop remain work ahead.

A native unconstrained 3D rigid-body kernel now supplies full inertia validation, world momentum integration, force/impulse application and a fixed tick clock. Original analytic fixtures and a CI-run example establish this numerical foundation. Native vehicle mass-property interpretation, equilibrium, tyre/suspension laws and collision response still need implementation and original-game validation before a native controlled drive can be claimed.

Native suspension inspection records STRUT/DWB hardpoints and spring/damper/stop source values with provenance. Independent DWB five-distance and STRUT distance/coaxial geometry solvers now pass analytical and local-source constraint checks. Their differential maps provide relative upright velocities, virtual-work load projection and ideal massless support reactions under an explicitly supplied held load. An explicit fixed-anchor/upright spring mount now shares the passive spring/damper law, producing length gradients and projected loads with tested energy/power consistency. A fixed-frame/fixed-rack dynamics step now advances an explicitly massive DWB/STRUT upright with COM/tensor, gravity and spring loads, passing analytical, energy and constraint checks. A floating chassis can now be coupled to 1..4 mounted uprights with a common acceleration solve, spring recoil and checked Newton/Euler balance; the mounting frames and masses are explicit inputs. Separate axisymmetric wheel masses and spin coordinates can now join that coupled solve, with tested internal motor reaction, gyroscopic transport and power balance. A separate common-midpoint multibody spring step also passes original analytical/energy tests. These components are not automatically connected as an original AC suspension: source frames/setup mapping, original kinematics/force validation, source spring mounts, unsprung-mass/preload assignment, original vehicle calibration, moving-rack control and calibrated drivetrain/brake dynamics, nonlinear laws and contact/tyre response remain work ahead.

The native renderer now assembles a selected car and track in one inspection window/pass with independent rigid draw roots and shared source/upload budgets. Local BMW/Drift captures and native window readbacks are verified. The source marker plane remains an unsolved visual pose; input-driven native motion, chassis equilibrium and original vehicle/material fidelity remain work ahead.
