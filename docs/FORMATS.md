# Content formats

This is the compatibility record for the current implementation. Planned work is separate from verified behavior. No gameplay compatibility is established.

## Current support

| Area | Current behavior | Not implemented |
| --- | --- | --- |
| Content directories | Enumerates immediate car and track folders | Full layout discovery and semantic validation |
| UI metadata | Checks whether default JSON files exist | JSON field interpretation |
| INI | Strict section/key/value subset with source-aware errors | Duplicate merging, includes, expressions, and CSP extensions |
| LUT | Numeric pairs, validation, linear interpolation, endpoint clamping | Inline LUT expressions and higher-dimensional tables |
| Track scene configs | Static MODEL_n references, explicit transforms, resource confinement | Dynamic objects and geometry loading |
| KN5 | Signature/version checks for v5/v6 headers | Geometry, materials, textures, hierarchy, animation |
| ACD | Checks whether a car archive exists | Archive decoding |
| CSP and plugins | No compatibility claimed | Extension configs, scripts, and runtime behavior |

The inventory scope is `file_presence_only`. Scene resource inspection is `configuration_and_header_only`; even a successful result has `geometry_loaded: false`.

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

## KN5 headers

Local observations confirm the six-byte `sc6969` signature followed by a little-endian 32-bit version. Version 6 includes another 32-bit header field. The current inspector accepts versions 5 and 6, checks header length, and records file size. It does not validate the remaining file structure; a header-only fixture can pass this limited inspection.

## Local validation record

On 2026-10-17, the current readers inspected 42 model configurations referencing 295 static model headers in the development installation with no model errors. There were 22 unsupported-section warnings. These are model references, not a count of unique files or fully supported scenes.

All 27 LUT files found under that installation's SDK content examples parsed with the current subset. This is local evidence, not a claim about all original or community content. Validation reports remain under ignored `reports/`; no game data is published.

## Reproducible examples

Tests use small original fixtures generated in temporary directories, including explicitly header-only KN5 fixtures. Do not add copied cars, tracks, textures, audio, or SDK archives to the repository. A parser change should include a minimal example proving the new behavior and invalid input that must be diagnosed.
