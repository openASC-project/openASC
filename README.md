# openASC

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="site/brand/openasc-dark.svg">
  <img src="site/brand/openasc-light.svg" alt="openASC — open circuit and forward arrow" width="520">
</picture>

[Logo source files and usage](site/brand/README.md) · Artwork licensed under [MIT](LICENSE).

An independent open-source engine project for **Assetto Corsa (2014)** content, inspired by [openOMSI](https://github.com/openOMSI-org/openOMSI).

**Status: first static geometry viewer. There is no playable driving engine yet.** The repository contains local inventory, INI/LUT readers, a bounded KN5 static geometry importer, an interactive local WebGL viewer, original tests, and a documentation website. Supported embedded diffuse textures, alpha test, and approximate transparency are available. Original shaders and vehicle simulation are not implemented.

[Website](https://openasc-project.github.io/openASC/) · [User guide](docs/USER_GUIDE.md) · [Compatibility record](docs/FORMATS.md) · [Architecture](docs/ARCHITECTURE.md) · [Roadmap](docs/FEASIBILITY.md)

## Quick start

Python 3.10+ is required. No third-party packages are needed.

```sh
python tools/inspect_content.py ./assettocorsa
python tools/inspect_content.py ./assettocorsa --output reports/content.json
python -m openasc scene ./assettocorsa --track drift
python -m openasc scene ./assettocorsa --track ks_barcelona --layout layout_gp
python -m openasc view ./assettocorsa --track drift
python -m unittest discover -s tests -v
```

The scanner reads the immediate `content/cars` and `content/tracks` directories and checks for expected files. It does not launch the game, unpack ACD archives, parse geometry, or copy resources. Reports use content-relative paths and are not saved unless requested. Existing output files are never overwritten.

The `openasc` CLI also provides `ini FILE`, `lut FILE --sample X`, and `kn5 FILE` commands. The `scene` command remains a lightweight configuration/header inspector. The `view` command loads supported static geometry and starts an interactive preview at [127.0.0.1:4180](http://127.0.0.1:4180/). See the [user guide](docs/USER_GUIDE.md) and [supported format subset](docs/FORMATS.md).

The viewer requires a browser with WebGL 2. Drag to orbit, right-drag to pan, scroll to zoom, or select a mesh to focus it. It previews supported embedded txDiffuse textures with basic lighting and approximate alpha behavior. Unsupported resources fall back to diagnostic colours; original shaders are not reproduced. Original resources remain local and are not written into the website or repository.

Your installation may live outside this repository: pass its root path instead of `./assettocorsa`. The `assettocorsa/` and `reports/` directories are ignored by Git.

## Website and documentation

The website follows a documentation-first structure inspired by [openomsi.org](https://openomsi.org/). Documentation, code comments, diagnostics, and commit messages are maintained in English.

```sh
python tools/build_site.py
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Open [localhost](http://127.0.0.1:4173/). The site uses no npm packages, remote fonts, or CDN assets. Documentation pages are generated from `docs/` using Python's standard library. Edit `web/overview.html` for the overview, then regenerate HTML before committing.

Pushes affecting `site/` on the default `main` or `master` branch automatically deploy to [GitHub Pages](https://openasc-project.github.io/openASC/). See the [publishing guide](docs/GITHUB_PAGES.md) and [development guide](docs/DEVELOPMENT.md).

## Repository layout

```text
docs/                     English documentation
web/                      Website template and overview source
site/                     Generated HTML and shared static assets
tools/inspect_content.py  Local content inventory
openasc/                  INI/LUT readers and scene resource diagnostics
viewer/                   Local WebGL geometry preview interface
tools/build_site.py       Documentation site builder
tests/                    Original, game-independent test fixtures
.github/workflows/        CI and GitHub Pages
assettocorsa/             Local game installation, ignored by Git
```

## Next milestone

Move toward a first drive with validated road surfaces, vehicle configuration, and model rotation conventions, while improving multilayer/detail/normal material fidelity. Supported DDS diffuse textures are now available in the first viewer. Static KN5 geometry, hierarchy, and material definitions are read; unsupported features are diagnosed. Vehicle physics, force feedback, AI, audio, and networking are separate later stages. Full mod compatibility, CSP support, and exact original behavior are not established.

## License and independence

New source code and website files use the [MIT license](LICENSE). This does not grant rights to Assetto Corsa or third-party content. Game binaries, cars, tracks, textures, and audio are not distributed here. Use your own installed copy for local research.

Assetto Corsa is a Kunos Simulazioni product. openASC is independent and unaffiliated. No openOMSI code or branding assets are used.
