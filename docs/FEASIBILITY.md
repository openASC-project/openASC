# Roadmap and feasibility

An independent engine that reads Assetto Corsa content is a technically plausible engineering direction. Access to modding tools and an SDK does not provide the simulator source or guarantee an exact reproduction of its physics.

[openOMSI](https://github.com/openOMSI-org/openOMSI) is the inspiration for organizing an independent implementation around documented formats and a public website. It is not an existing engine base for Assetto Corsa.

## Current state

Implemented: inventory, INI/LUT readers, static model diagnostics, supported KN5 geometry/hierarchy/material definitions, and a first local WebGL viewer with camera and mesh inspection. Original fixtures, English documentation, and the GitHub Pages site accompany the implementation. Supported diffuse DDS textures, approximate alpha/depth behavior, typed surface definitions and BVH-accelerated vertical ground support queries, ordinary ACD data reading, a typed vehicle configuration subset and geometric spawn placement preflights are also implemented. A local independent keyboard-driving prototype now includes 120 Hz planar forces, geometric ground attachment, a rendered car and chase camera. Not implemented: wall/body response, suspension dynamics, original tyre/drivetrain behavior, wheel animation, original shaders, full multilayer/detail/normal rendering, wheel-device input, force feedback, audio, or networking.

The local installation includes SDK documentation and UDP server examples. These are research starting points, not resources distributed by openASC.

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

The initial configuration subset, resource manifest, and first geometry viewer are implemented and verified with the local Drift track. Supported embedded diffuse textures now render through tested compressed and software paths. Typed track surface definitions and static downward ground queries are now tested on original fixtures and local Drift, including hidden physical roads. A local BMW M3 E30 configuration and Drift AC_PIT_0 footprint now load, with four supported ROAD contacts and an explicit error for a separate duplicate start slot. Source basis/left-right conventions are checked against original synthetic fixtures and the local wheel nodes. An independent fixed-step first-drive loop now passes original mathematical tests and browser checks for acceleration, steering, braking, reverse, pause and reset with local BMW/Drift. Next, implement wall/body response and wheel animation, improve car skin/material loading, and develop suspension/tyre dynamics with explicit validation tolerances. Wall response and tyre dynamics remain required work; multilayer/detail/normal material behavior remains separate ongoing renderer work. Keep unsupported features visible in diagnostics and preserve source locations in errors.

## Long-term scope

Full compatibility may require years of development and is not guaranteed. CSP, Python apps, plugins, and exact original physics need separate investigation. No release date or platform support is promised before a working prototype is tested.

Use the [compatibility record](FORMATS.md) for implemented behavior and the [architecture](ARCHITECTURE.md) for proposed module boundaries.

## Content handling

Write original code and document verified behavior. Use original minimal fixtures in tests. Do not commit game executables, models, textures, sounds, SDK archives, or third-party mods. Distribution rights for any future third-party material must be considered separately.
