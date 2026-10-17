# Content formats

This is the compatibility record for the current implementation. Planned work is listed separately from verified behavior.

## Current support

| Area | Current behavior | Not implemented |
| --- | --- | --- |
| Content directories | Enumerates immediate car and track folders | Track-layout discovery and semantic validation |
| UI metadata | Checks whether default JSON files exist | JSON field interpretation |
| INI | Checks model configuration filenames | Section/key parsing and typed values |
| LUT | No reader yet | Table parsing, interpolation, and validation |
| KN5 | Checks top-level file presence | Header validation, geometry, materials, textures, hierarchy |
| ACD | Checks whether a car archive exists | Archive decoding |
| CSP and plugins | No compatibility claimed | Extension configs, scripts, and runtime behavior |

The inventory tool must not be used as a compatibility test. Its output deliberately uses `file_presence_only` as the scope.

## Next: configuration readers

The next implementation will define a tested INI/LUT subset, preserve source locations in errors, and diagnose ambiguous or unsupported syntax. Track model references will be resolved within the selected content root before geometry loading is attempted.

Local research has identified model references in `models*.ini`, configuration sections in `data/surfaces.ini`, and numeric `x|y` tables in SDK car examples. These observations are starting points, not a complete format specification.

## Reproducible examples

Tests use small original fixtures generated in temporary directories. Do not add copied cars, tracks, textures, audio, or SDK archives to the repository. A parser change should include a minimal example that proves the new behavior and a clear failure case for invalid input.

## Evidence and references

- [openOMSI](https://github.com/openOMSI-org/openOMSI) is the organizational inspiration for documenting formats alongside an independent implementation.
- The local Assetto Corsa SDK contains car pipeline documentation and server examples. It is not distributed here.
- Future binary format notes will cite the specific primary references and local observations used to validate them.
