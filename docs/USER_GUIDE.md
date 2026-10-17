# User guide

openASC provides local content inventory, configuration readers, and static scene resource diagnostics. There is no playable engine or scene viewer yet.

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
