# User guide

openASC provides content inventory, configuration readers, static scene resource diagnostics, and a first local geometry viewer. There is no playable driving engine yet.

## Requirements

- Python 3.10 or newer.
- Your own installed copy of Assetto Corsa (2014) to inspect game content.
- No third-party Python packages.

The website and automated tests work without an installation of the game.

## Get the source

```sh
git clone https://github.com/openASC-project/openASC.git
cd openASC
```

Keep your installation outside the repository, or use the ignored `assettocorsa/` directory. The scanner accepts either location.

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

## Query physical ground support

```sh
python -m openasc ground ./assettocorsa --track drift
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
