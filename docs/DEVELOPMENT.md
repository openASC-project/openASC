# Development

All repository documentation, code comments, diagnostics, commit messages, and website content are maintained in English.

## Run checks

Python 3.10+ is enough for the tools, tests, and documentation builder. No game installation is needed for tests.

```sh
python -m unittest discover -s tests -v
python tools/build_site.py --check
node --check site/app.js
node --check viewer/viewer.js
node --check viewer/dds.js
node --check viewer/drive.js
node --test tests/test_dds.cjs
```

Node.js 18+ is used for JavaScript syntax checks and original DDS pixel tests. The website itself has no npm dependencies or build-time JavaScript framework.

The local geometry viewer also has no npm dependencies. Its JavaScript is in `viewer/`, served only by `python -m openasc view`. Tests construct original complete KN5 fixtures and exercise the loopback server, including foreign-host/origin rejection. Binary fixtures are not copied from the game.

## Update the website

```sh
python tools/build_site.py
python -m http.server 4173 --directory site --bind 127.0.0.1
```

Open [localhost](http://127.0.0.1:4173/). The document builder supports the Markdown subset used here: headings, paragraphs, links, inline code, bold text, lists, fenced code, and tables. Raw HTML in documentation is escaped.

Edit `docs/` for documentation, `web/overview.html` for the overview body, and `web/layout.html` for the shared shell. Edit `site/style.css` and `site/app.js` for styling and browser behavior. Generated HTML files contain a notice identifying their source.

## Contribute

The independent driving model is in `openasc/dynamics.py`; its tests use original flat/sloped geometry and original car configurations. Keep rendering and HTTP telemetry reads out of the fixed-step solver. Controller tests can advance exact steps with an injected clock instead of sleeping. Real-content/browser validation remains local; record the scope and limitations before changing compatibility claims.

Website progress screenshots are selected flattened captures under `site/images/`, with captions in `web/overview.html`. Update them after meaningful visual milestones. Do not add raw game assets or temporary browser reports. The project's MIT license does not relicense the underlying game content depicted in screenshots.

Start with an [issue](https://github.com/openASC-project/openASC/issues) describing a reproducible problem or bounded feature. Keep parser fixtures original and small. A green test suite does not establish compatibility with all Assetto Corsa content.

Before committing, check that neither `assettocorsa/` nor `reports/` is tracked. These directories are ignored intentionally. Do not include original executable files, copyrighted game content, or third-party mods.

New source code and website files use the MIT license. Dependencies and any future third-party additions must retain their own license notices.
