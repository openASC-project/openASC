# openASC

An independent open-source engine project for **Assetto Corsa (2014)** content, inspired by [openOMSI](https://github.com/openOMSI-org/openOMSI).

**Status: content tools. There is no playable engine yet.** The repository contains a local inventory, source-aware INI/LUT readers, static track configuration and KN5 header inspection, original tests, and a documentation website. Inspecting a header does not establish geometry or gameplay compatibility.

[Website](https://openasc-project.github.io/openASC/) · [User guide](docs/USER_GUIDE.md) · [Compatibility record](docs/FORMATS.md) · [Architecture](docs/ARCHITECTURE.md) · [Roadmap](docs/FEASIBILITY.md)

## Quick start

Python 3.10+ is required. No third-party packages are needed.

```sh
python tools/inspect_content.py ./assettocorsa
python tools/inspect_content.py ./assettocorsa --output reports/content.json
python -m openasc scene ./assettocorsa --track drift
python -m openasc scene ./assettocorsa --track ks_barcelona --layout layout_gp
python -m unittest discover -s tests -v
```

The scanner reads the immediate `content/cars` and `content/tracks` directories and checks for expected files. It does not launch the game, unpack ACD archives, parse geometry, or copy resources. Reports use content-relative paths and are not saved unless requested. Existing output files are never overwritten.

The `openasc` CLI also provides `ini FILE` and `lut FILE --sample X` commands. The `scene` command resolves static model declarations, validates transforms and referenced KN5 headers, and reports unsupported sections. It does not load geometry or render a scene. See the [supported format subset](docs/FORMATS.md).

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
tools/build_site.py       Documentation site builder
tests/                    Original, game-independent test fixtures
.github/workflows/        CI and GitHub Pages
assettocorsa/             Local game installation, ignored by Git
```

## Next milestone

Read KN5 geometry, hierarchy, and materials, then display one real local scene in a viewer with unsupported features diagnosed explicitly. INI/LUT readers and static scene resource diagnostics are implemented. Vehicle physics, force feedback, AI, audio, and networking are separate later stages. Full mod compatibility, CSP support, and exact original behavior are not established.

## License and independence

New source code and website files use the [MIT license](LICENSE). This does not grant rights to Assetto Corsa or third-party content. Game binaries, cars, tracks, textures, and audio are not distributed here. Use your own installed copy for local research.

Assetto Corsa is a Kunos Simulazioni product. openASC is independent and unaffiliated. No openOMSI code or branding assets are used.
