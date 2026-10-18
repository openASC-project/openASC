"""Navigation regressions for the generated documentation, without a browser."""
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
import unittest
from urllib.parse import unquote, urlsplit

from tools import build_site


class Page(HTMLParser):
    def __init__(self, text):
        super().__init__()
        self.ids, self.links = [], []
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        if values.get('id'):
            self.ids.append(values['id'])
        if tag == 'a' and values.get('href'):
            self.links.append(values['href'])


class DocumentationNavigationTests(unittest.TestCase):
    def test_cross_document_links_keep_fragments_and_reject_script_urls(self):
        self.assertEqual(build_site.link_target('FORMATS.md#native-vehicle-and-acd-path'),
                         './formats.html#native-vehicle-and-acd-path')
        self.assertEqual(build_site.link_target('./USER_GUIDE.md#requirements'), './guide.html#requirements')
        self.assertEqual(build_site.link_target('https://example.org/a#b'), 'https://example.org/a#b')
        self.assertEqual(build_site.link_target('javascript:alert(1)'), '#')

    def test_headings_create_unique_stable_ids_and_escape_display_markup(self):
        page = Page(build_site.markdown('## Native vehicle and ACD path\n## Native vehicle and ACD path\n## Native `data.acd`\n## <script>\n'))
        self.assertEqual(page.ids, ['native-vehicle-and-acd-path', 'native-vehicle-and-acd-path-1', 'native-dataacd', 'script'])
        self.assertIn('&lt;script&gt;', build_site.markdown('## <script>'))

    def test_every_local_fragment_link_targets_an_existing_unique_id(self):
        root = Path(build_site.ROOT / 'site')
        pages = {path.resolve(): Page(path.read_text(encoding='utf-8')) for path in root.glob('*.html')}
        checked = 0
        for path, page in pages.items():
            duplicates = [name for name, count in Counter(page.ids).items() if count > 1]
            self.assertFalse(duplicates, f'{path.name}: duplicate ids {duplicates}')
            for link in page.links:
                target = urlsplit(link)
                if target.scheme or target.netloc or not target.fragment:
                    continue
                destination = (path.parent / unquote(target.path)).resolve() if target.path else path
                self.assertIn(destination, pages, f'{path.name}: missing page for {link}')
                self.assertIn(unquote(target.fragment), pages[destination].ids,
                              f'{path.name}: missing section for {link}')
                checked += 1
        self.assertGreater(checked, 5)
