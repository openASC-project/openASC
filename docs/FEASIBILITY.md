# Roadmap and feasibility

An independent engine that reads Assetto Corsa content is a technically plausible engineering direction. Access to modding tools and an SDK does not provide the simulator source or guarantee an exact reproduction of its physics.

[openOMSI](https://github.com/openOMSI-org/openOMSI) is the inspiration for organizing an independent implementation around documented formats and a public website. It is not an existing engine base for Assetto Corsa.

## Current state

Implemented: a file-presence inventory, tests on original fixtures, English documentation, and a GitHub Pages website. Not implemented: configuration readers, geometry import, rendering, vehicle simulation, wheel input, force feedback, audio, or networking.

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

Implement the configuration readers and a resource manifest before loading geometry. Keep source locations in diagnostics and identify unsupported syntax explicitly. The first viewer must display real local geometry, rather than a placeholder scene.

## Long-term scope

Full compatibility may require years of development and is not guaranteed. CSP, Python apps, plugins, and exact original physics need separate investigation. No release date or platform support is promised before a working prototype is tested.

Use the [compatibility record](FORMATS.md) for implemented behavior and the [architecture](ARCHITECTURE.md) for proposed module boundaries.

## Content handling

Write original code and document verified behavior. Use original minimal fixtures in tests. Do not commit game executables, models, textures, sounds, SDK archives, or third-party mods. Distribution rights for any future third-party material must be considered separately.
