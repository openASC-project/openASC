"""Build static documentation pages from the repository's Markdown subset."""
import argparse
import html
from pathlib import Path
import re
from string import Template

ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = 'https://github.com/openASC-project/openASC'
PAGES = [
    ('guide', 'User guide', 'USER_GUIDE.md', '▷'),
    ('formats', 'Content formats', 'FORMATS.md', '▤'),
    ('telemetry', 'Telemetry research', 'TELEMETRY.md', '∿'),
    ('architecture', 'Architecture', 'ARCHITECTURE.md', '⌘'),
    ('roadmap', 'Roadmap & feasibility', 'FEASIBILITY.md', '↗'),
    ('development', 'Development', 'DEVELOPMENT.md', '⚒'),
    ('publishing', 'GitHub Pages', 'GITHUB_PAGES.md', '▧'),
]
ROUTES = {filename: f'./{slug}.html' for slug, _, filename, _ in PAGES}


def link_target(target):
    """Only emit local references or HTTP(S) links from Markdown."""
    path, marker, fragment = target.partition('#')
    route = ROUTES.get(path.removeprefix('./'))
    if route is not None:
        return route + (marker + fragment if marker else '')
    if path == '../LICENSE':
        return f'{REPOSITORY}/blob/main/LICENSE' + (marker + fragment if marker else '')
    if re.match(r'^https?://', target) or re.match(r'^[\w./#-]+$', target):
        return target
    return '#'


def inline(text):
    tokens = []

    def protect(value):
        tokens.append(value)
        return f'\x00{len(tokens) - 1}\x00'

    text = re.sub(r'`([^`]+)`', lambda m: protect(f'<code>{html.escape(m[1])}</code>'), text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', lambda m: protect(
        f'<a href="{html.escape(link_target(m[2]), quote=True)}">{html.escape(m[1])}</a>'), text)
    text = html.escape(text)
    text = re.sub(r'\*\*(.+?)\*\*', r'<strong>\1</strong>', text)
    return re.sub(r'\x00(\d+)\x00', lambda m: tokens[int(m[1])], text)


def markdown(text):
    lines = text.splitlines()
    output = []
    heading_ids = set()
    index = 0
    while index < len(lines):
        line = lines[index].strip()
        index += 1
        if not line:
            continue
        if line.startswith('```'):
            block = []
            while index < len(lines) and not lines[index].strip().startswith('```'):
                block.append(lines[index])
                index += 1
            if index == len(lines):
                raise ValueError('Unterminated fenced code block')
            index += 1
            output.append('<pre><code>' + html.escape('\n'.join(block)) + '</code></pre>')
        elif re.match(r'^#{1,3} ', line):
            marker, title = line.split(' ', 1)
            level = len(marker)
            base = re.sub(r'\s+', '-', re.sub(r'[^\w\s-]', '', title.lower())).strip('-') or 'section'
            anchor, suffix = base, 0
            while anchor in heading_ids:
                suffix += 1
                anchor = f'{base}-{suffix}'
            heading_ids.add(anchor)
            output.append(f'<h{level} id="{html.escape(anchor, quote=True)}">{inline(title)}</h{level}>')
        elif line.startswith('|'):
            rows = [line]
            while index < len(lines) and lines[index].strip().startswith('|'):
                rows.append(lines[index].strip())
                index += 1
            cells = [[cell.strip() for cell in row.strip('|').split('|')] for row in rows]
            output.append('<div class="table-scroll" tabindex="0" role="region" aria-label="Documentation table"><table><thead><tr>')
            output.extend(f'<th scope="col">{inline(cell)}</th>' for cell in cells[0])
            output.append('</tr></thead><tbody>')
            for row in cells[2:]:
                output.append('<tr>' + ''.join(f'<td>{inline(cell)}</td>' for cell in row) + '</tr>')
            output.append('</tbody></table></div>')
        elif re.match(r'^(?:- |\d+\. )', line):
            ordered = bool(re.match(r'^\d+\. ', line))
            tag = 'ol' if ordered else 'ul'
            pattern = r'^\d+\. ' if ordered else r'^- '
            items = [re.sub(pattern, '', line)]
            while index < len(lines) and re.match(pattern, lines[index].strip()):
                items.append(re.sub(pattern, '', lines[index].strip()))
                index += 1
            output.append(f'<{tag}>' + ''.join(f'<li>{inline(item)}</li>' for item in items) + f'</{tag}>')
        else:
            paragraph = [line]
            while index < len(lines) and lines[index].strip():
                if re.match(r'^(?:#|```|\||- |\d+\. )', lines[index].strip()):
                    break
                paragraph.append(lines[index].strip())
                index += 1
            output.append('<p>' + inline(' '.join(paragraph)) + '</p>')
    return '\n'.join(output)


def navigation(active):
    def item(slug, title, symbol, url=None):
        current = ' aria-current="page"' if slug == active else ''
        return f'<a href="{url or "./" + slug + ".html"}"{current}><span class="nav-icon" aria-hidden="true">{symbol}</span>{title}</a>'

    result = ['<nav aria-label="Primary">', item('index', 'Overview', '⌂'), '</nav>',
              '<nav aria-labelledby="docs-label"><h2 id="docs-label">Documentation</h2>']
    result.extend(item(slug, title, symbol) for slug, title, _, symbol in PAGES)
    result.extend(['</nav><nav aria-labelledby="project-label"><h2 id="project-label">Project</h2>',
                   item('source', 'Source code', '&lt;/&gt;', REPOSITORY),
                   item('issues', 'Issues', '◇', REPOSITORY + '/issues'),
                   item('releases', 'Releases', '↓', REPOSITORY + '/releases'),
                   item('inspiration', 'openOMSI', '↗', 'https://openomsi.org/'), '</nav>'])
    return '\n'.join(result)


def build(check=False):
    template = Template((ROOT / 'web/layout.html').read_text(encoding='utf-8'))
    documents = [('index', 'Overview', 'web/overview.html',
                  (ROOT / 'web/overview.html').read_text(encoding='utf-8'), 'overview')]
    for slug, title, filename, _ in PAGES:
        content = markdown((ROOT / 'docs' / filename).read_text(encoding='utf-8'))
        source_link = f'<a class="edit-link" href="{REPOSITORY}/blob/main/docs/{filename}">View this document on GitHub ↗</a>'
        documents.append((slug, title, f'docs/{filename}', f'<article class="document">{content}{source_link}</article>', 'documentation'))
    stale = []
    for slug, title, source, content, page_class in documents:
        result = template.substitute(title=title, source=source, content=content,
                                     description=f'openASC {title.lower()}: independent Assetto Corsa engine research and content tools.',
                                     navigation=navigation(slug), page_class=page_class)
        target = ROOT / 'site' / f'{slug}.html'
        if check:
            if not target.exists() or target.read_text(encoding='utf-8') != result:
                stale.append(target.name)
        else:
            target.write_text(result, encoding='utf-8', newline='\n')
    if stale:
        raise ValueError('Stale generated pages: ' + ', '.join(stale) + '. Run python tools/build_site.py.')
    print(f'{"Checked" if check else "Built"} {len(documents)} pages.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Fail if generated HTML differs from its source')
    arguments = parser.parse_args()
    try:
        build(arguments.check)
    except ValueError as error:
        parser.exit(1, f'Error: {error}\n')
