"""Read the ordinary folder-keyed ACD record format in memory, without extraction."""
from dataclasses import dataclass, replace
from pathlib import Path
import re
import struct

from .config import ContentError

MAX_ARCHIVE = 64 * 1024 ** 2
MAX_ENTRY = 8 * 1024 ** 2
MAX_DECODED = 16 * 1024 ** 2
MAX_ENTRIES = 4096


def data_name(name):
    """The supported car-data namespace is flat, portable ASCII filenames."""
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,254}', name):
        raise ContentError('car_data_name', 'Car data references require a flat ASCII filename without path separators.', '<car data>')
    return name.lower()


def folder_key(identifier):
    """Compute the eight legacy name checksums and their decimal ASCII key.

    This is format interoperability, not an authenticated archive or DRM scheme.
    Other protected variants are not recognized or guessed.
    """
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', identifier):
        raise ContentError('acd_identifier', 'Legacy ACD keys support 1 to 128 ASCII letters, digits, underscores or hyphens.', '<car data>')
    chars = list(identifier.lower().encode('ascii'))
    count = len(chars)
    pair, triple, product, remainder, adjacent = 0, 0, 66, 171, 171
    for i in range(0, count - 1, 2):
        pair = (pair * chars[i] - chars[i + 1]) & 255
    for i in range(1, count - 3, 3):
        value = (triple * chars[i] + 2 ** 31) % 2 ** 32 - 2 ** 31
        divisor = chars[i + 1] + 27
        quotient = abs(value) // divisor * (-1 if value < 0 else 1)
        triple = quotient - 27 - chars[i - 1]
    for i in range(1, count - 4, 4):
        product = ((chars[i] + 15) * product * (chars[i - 1] + 15) + 22) & 255
    even = chars[:max(0, count - 2):2]
    for character in even:
        remainder %= character
    for a, b in zip(chars, chars[1:]):
        adjacent = adjacent // a + b
    checksums = (sum(chars), pair, triple, 5763 - sum(chars[1:]),
                 product, 101 - sum(even), remainder, adjacent)
    return '-'.join(str(value & 255) for value in checksums).encode('ascii')


class AcdError(ContentError):
    def __init__(self, code, message, source, offset):
        super().__init__(code, message, source)
        self.diagnostic = replace(self.diagnostic, byte_offset=offset)


@dataclass(frozen=True)
class Entry:
    name: str
    offset: int
    length: int


class Archive:
    def __init__(self, data, identifier, source='<acd>'):
        self.source, self.key = source, folder_key(identifier)
        self.data, self.entries, self.header = data, {}, None
        cursor, total = 0, 0

        def fail(code, message):
            raise AcdError(code, message, source, cursor)

        def take(length):
            nonlocal cursor
            if length < 0 or length > len(data) - cursor:
                fail('acd_truncated', 'Record field exceeds the remaining archive bytes.')
            value = data[cursor:cursor + length]
            cursor += length
            return value

        def integer():
            return struct.unpack('<i', take(4))[0]

        if len(data) > MAX_ARCHIVE:
            fail('acd_limit', 'ACD exceeds the 64 MiB source budget.')
        if len(data) >= 4 and struct.unpack_from('<i', data)[0] == -1111:
            integer()
            self.header = integer()  # Preserve the legacy extra word without assigning semantics.
        while cursor < len(data):
            if len(self.entries) >= MAX_ENTRIES:
                fail('acd_limit', 'ACD exceeds the 4096 entry budget.')
            length = integer()
            if not 1 <= length <= 255:
                fail('acd_name', 'Archive entry name length must be between 1 and 255 bytes.')
            try:
                name = take(length).decode('ascii')
                key = data_name(name)
            except (UnicodeError, ContentError):
                fail('acd_name', 'Archive entry name is outside the supported flat ASCII namespace.')
            if key in self.entries:
                fail('acd_duplicate', f'Archive has ambiguous case-insensitive entry {name!r}.')
            length = integer()
            if not 0 <= length <= MAX_ENTRY or total + length > MAX_DECODED:
                fail('acd_limit', 'Entry exceeds the 8 MiB entry or 16 MiB combined decoded budget.')
            total += length
            offset = cursor
            if length * 4 > len(data) - cursor:
                fail('acd_truncated', 'Encoded entry payload is truncated.')
            cursor += length * 4
            self.entries[key] = Entry(name, offset, length)
        if not self.entries:
            fail('acd_empty', 'An ACD must contain at least one supported record.')

    def read(self, name):
        entry = self.entries.get(data_name(name))
        if entry is None:
            raise ContentError('car_data_missing', f'Required car data file {name!r} is absent.', self.source)
        payload = self.data[entry.offset:entry.offset + entry.length * 4:4]
        return bytes((value - self.key[i % len(self.key)]) & 255 for i, value in enumerate(payload))


def read_archive(path, identifier, source=None):
    path = Path(path)
    with path.open('rb') as stream:
        data = stream.read(MAX_ARCHIVE + 1)
    return Archive(data, identifier, source or path.name)
