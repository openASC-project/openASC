"""Generate the progress, gallery, launcher and mods pages from JSON data under web/data."""
import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'https://github.com/openASC-project/openASC'
RELEASES = REPOSITORY + '/releases/latest/download'
STATES = (('partial', 'In progress'), ('missing', 'Not started'), ('done', 'Verified'))


def load(name):
    return json.loads((ROOT / 'web' / 'data' / name).read_text(encoding='utf-8'))


def esc(value):
    return html.escape(str(value), quote=True)


def overall(progress):
    total = sum(item['weight'] for item in progress['subsystems'])
    return sum(item['weight'] * item['percent'] for item in progress['subsystems']) / total


def band(percent):
    return 'high' if percent >= 60 else 'mid' if percent >= 30 else 'low'


def progress_board(progress):
    """One bar split into sectors sized by weight, like a timing screen."""
    items = progress['subsystems']
    total = sum(item['weight'] for item in items)
    sectors = []
    for item in items:
        sectors.append(
            f'<a class="sector {band(item["percent"])}" href="./progress.html#{esc(item["id"])}" '
            f'style="--w:{item["weight"] / total * 100:.3f};--p:{item["percent"]}" '
            f'title="{esc(item["name"])}: {item["percent"]}%">'
            f'<span class="sector-fill"></span><span class="sector-name">{esc(item["name"])}</span></a>')
    return ('<div class="sector-bar" role="img" aria-label="Overall progress by subsystem">'
            + ''.join(sectors) + '</div>')


def progress_rows(progress):
    rows = []
    for item in sorted(progress['subsystems'], key=lambda i: (-i['percent'], i['name'])):
        label = dict(STATES)[item['state']]
        rows.append(
            f'<article class="row {band(item["percent"])}" id="{esc(item["id"])}">'
            f'<h3>{esc(item["name"])}</h3>'
            f'<div class="meter" role="meter" aria-valuemin="0" aria-valuemax="100" aria-valuenow="{item["percent"]}" '
            f'aria-label="{esc(item["name"])} completion"><span style="--p:{item["percent"]}"></span></div>'
            f'<p class="row-value"><strong>{item["percent"]}%</strong> <span>{esc(label)}, weight {item["weight"]}</span></p>'
            f'<p class="row-detail">{esc(item["detail"])}</p></article>')
    return '\n'.join(rows)


def progress_summary(progress):
    percent = overall(progress)
    return (f'<section class="progress-summary" aria-labelledby="progress-title">'
            f'<h2 id="progress-title">openASC is <span class="figure">{percent:.0f}%</span> of the way to Assetto Corsa</h2>'
            f'<p>Estimated across {len(progress["subsystems"])} subsystems, weighted by how much each matters to a playable 1:1 port. '
            f'Updated {esc(progress["updated"])}.</p>{progress_board(progress)}'
            f'<p class="legend"><span class="key high"></span>60% and above <span class="key mid"></span>30 to 59% <span class="key low"></span>below 30%. '
            f'<a href="./progress.html">Every subsystem, with what is missing →</a></p></section>')


def progress_page(progress):
    return (f'<article class="document wide"><h1>Progress</h1>'
            f'<p class="lead">{overall(progress):.1f}% overall. {esc(progress["note"])}</p>'
            f'{progress_board(progress)}<div class="rows">{progress_rows(progress)}</div>'
            f'<p>The numbers change with each commit that completes a subsystem. Source: '
            f'<a href="{REPOSITORY}/blob/main/web/data/progress.json">web/data/progress.json</a>.</p></article>')


def picture(shot, lazy=True):
    loading = ' loading="lazy" decoding="async"' if lazy else ''
    return (f'<img src="./images/{esc(shot["image"])}" width="{shot["width"]}" height="{shot["height"]}"{loading} '
            f'alt="{esc(shot.get("alt", shot.get("label", shot.get("title", "openASC capture"))))}">')


def compare(entry):
    before, after = entry['before'], entry['after']
    before['alt'] = f'{before["label"]} (before)'
    after['alt'] = f'{after["label"]} (after)'
    return (f'<figure class="compare-card"><div class="compare" data-compare style="--pos:50%">'
            f'<div class="compare-layer">{picture(after)}</div>'
            f'<div class="compare-layer compare-top">{picture(before)}</div>'
            f'<label class="compare-control"><span class="visually-hidden">Reveal {esc(entry["title"])}</span>'
            f'<input type="range" min="0" max="100" value="50" step="1"></label>'
            f'<span class="compare-tag left">{esc(before["label"])}</span><span class="compare-tag right">{esc(after["label"])}</span></div>'
            f'<figcaption><strong>{esc(entry["title"])}</strong> {esc(entry["caption"])}</figcaption></figure>')


def pending(entry):
    ours = entry['ours']
    ours['alt'] = ours['label']
    return (f'<figure class="compare-card pending"><div class="compare pending-pair">'
            f'<div class="compare-layer">{picture(ours)}<span class="compare-tag left">{esc(ours["label"])}</span></div>'
            f'<div class="compare-layer empty"><p><strong>Original capture needed</strong><br>Assetto Corsa, same car, track and camera.</p></div></div>'
            f'<figcaption><strong>{esc(entry["title"])}</strong> {esc(entry["caption"])}</figcaption></figure>')


def gallery_page(gallery):
    shots = ''.join(
        f'<figure><a href="./images/{esc(s["image"])}">{picture(s)}</a>'
        f'<figcaption><strong>{esc(s["title"])}</strong> {esc(s["caption"])}</figcaption></figure>'
        for s in gallery['shots'])
    return ('<article class="document wide gallery"><h1>Gallery</h1>'
            '<p class="lead">Real captures from the native Rust renderer, read back from the GPU. They show locally installed Assetto Corsa content; none of it is distributed here.</p>'
            '<h2 id="milestones">Milestone comparisons</h2><p>Drag the handle to compare an earlier openASC capture with a later one.</p>'
            f'<div class="compare-grid">{"".join(compare(c) for c in gallery["comparisons"])}</div>'
            '<h2 id="original">Against the original game</h2>'
            '<p>No side-by-side capture of the original simulator is published yet, so openASC does not claim visual parity. Slots below fill in as matching captures are made.</p>'
            f'<div class="compare-grid">{"".join(pending(p) for p in gallery["original_pending"])}</div>'
            f'<h2 id="captures">All captures</h2><div class="screenshot-grid">{shots}</div></article>')


def launcher_page():
    assets = [
        ('Windows x64', 'openasc-launcher-windows-x64.zip', 'Windows 10 or 11, a GPU with Vulkan or DirectX 12'),
        ('Linux x86-64', 'openasc-launcher-linux-x86_64.tar.gz', 'glibc, X11 or Wayland, Vulkan GPU'),
    ]
    rows = ''.join(
        f'<tr><td>{esc(name)}</td><td><a class="button primary small" href="{RELEASES}/{esc(file)}">Download</a></td><td>{esc(need)}</td></tr>'
        for name, file, need in assets)
    return ('<article class="document wide"><h1>Launcher</h1>'
            '<p class="lead">One program that finds your Assetto Corsa install, keeps the openASC engine up to date, installs mods and starts a drive.</p>'
            '<div class="table-scroll" tabindex="0" role="region" aria-label="Launcher downloads"><table><thead><tr><th scope="col">Platform</th><th scope="col">File</th><th scope="col">Needs</th></tr></thead>'
            f'<tbody>{rows}</tbody></table></div>'
            '<p>You also need your own copy of Assetto Corsa (2014). openASC contains no game content. Releases are listed on '
            f'<a href="{REPOSITORY}/releases">GitHub</a>; the launcher is built from <a href="{REPOSITORY}/tree/main/crates/asc-launcher">crates/asc-launcher</a>.</p>'
            '<h2 id="what-it-does">What it does</h2><ul>'
            '<li><strong>Finds the game.</strong> Looks in the Steam library folders, or you pick the folder.</li>'
            '<li><strong>Updates the engine.</strong> Downloads the newest openASC build from GitHub Releases and checks its SHA-256 before installing.</li>'
            '<li><strong>Installs mods.</strong> Browse the <a href="./mods.html">mod catalog</a>, install, update or remove with one click. Every download is checked against the catalog hash.</li>'
            '<li><strong>Starts a drive.</strong> Prepares a workspace that points at your game folder, then runs the engine with the scenario you choose.</li>'
            '</ul><h2 id="limits">Current limits</h2>'
            '<p>The engine is a driving workbench, not a full game yet (see <a href="./progress.html">progress</a>). The launcher starts the scenarios the engine can run today. The launcher has not been code-signed, so Windows SmartScreen may warn on first start.</p></article>')


def mods_page(index):
    mods = index['mods']
    if mods:
        cards = ''.join(
            f'<article class="mod-card" id="{esc(m["id"])}"><h3>{esc(m["name"])} <small>{esc(m["version"])}</small></h3>'
            f'<p>{esc(m["description"])}</p><p class="mod-meta">By {esc(m["author"])}. {m["size_bytes"] / 1024:.0f} KiB. '
            f'<a href="./mods/{esc(m["file"])}">Download zip</a></p></article>' for m in mods)
        body = f'<div class="mod-grid">{cards}</div>'
    else:
        body = '<p class="empty">The catalog has no mods yet. Publish the first one with the steps below.</p>'
    return ('<article class="document wide"><h1>Mods</h1>'
            '<p class="lead">Scenario packs for the openASC engine. The launcher reads this catalog and installs packs for you.</p>'
            f'{body}<h2 id="publish">Publish a mod</h2><ol>'
            f'<li>Add a folder under <code>mods/</code> in the <a href="{REPOSITORY}">repository</a> with a <code>mod.json</code> (<code>id</code>, <code>name</code>, <code>version</code>, <code>author</code>, <code>description</code>) and your scenario files.</li>'
            '<li>Include no Assetto Corsa files. Scenarios refer to the user\'s own install through <code>../assettocorsa/</code> paths.</li>'
            '<li>Open a pull request. The site build zips the folder, records its SHA-256 and adds it to the catalog.</li></ol>'
            f'<p>The catalog the launcher reads is <a href="./mods/index.json">mods/index.json</a>.</p></article>')
