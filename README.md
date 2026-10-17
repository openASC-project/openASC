# openASC

An independent open-source engine project for **Assetto Corsa (2014)** content, inspired by [openOMSI](https://github.com/openOMSI-org/openOMSI).

**Status: foundation stage. There is no playable engine yet.** The repository currently contains a local content inventory tool, original tests, a development plan, and a documentation website. Detecting files does not establish format support or gameplay compatibility.

[Website](https://openasc-project.github.io/openASC/) · [User guide](docs/USER_GUIDE.md) · [Compatibility record](docs/FORMATS.md) · [Architecture](docs/ARCHITECTURE.md) · [Roadmap](docs/FEASIBILITY.md)

## Quick start

Python 3.10+ is required. No third-party packages are needed.

```sh
python tools/inspect_content.py ./assettocorsa
python tools/inspect_content.py ./assettocorsa --output reports/content.json
python -m unittest discover -s tests -v
```

The scanner reads the immediate `content/cars` and `content/tracks` directories and checks for expected files. It does not launch the game, unpack ACD archives, parse geometry, or copy resources. Reports use content-relative paths and are not saved unless requested. Existing output files are never overwritten.

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
tools/build_site.py       Documentation site builder
tests/                    Original, game-independent test fixtures
.github/workflows/        CI and GitHub Pages
assettocorsa/             Local game installation, ignored by Git
```

## Next milestone

Implement tested INI/LUT readers and resource diagnostics, then load one local scene in a viewer. Vehicle physics, force feedback, AI, audio, and networking are separate later stages. Full mod compatibility, CSP support, and exact original behavior are not established.

## License and independence

New source code and website files use the [MIT license](LICENSE). This does not grant rights to Assetto Corsa or third-party content. Game binaries, cars, tracks, textures, and audio are not distributed here. Use your own installed copy for local research.

Assetto Corsa is a Kunos Simulazioni product. openASC is independent and unaffiliated. No openOMSI code or branding assets are used.
