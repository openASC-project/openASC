//! Read ordinary folder-keyed ACD records in memory, without extraction.
use serde::Serialize;
use std::{collections::BTreeMap, fmt, io::Read, path::Path};

pub const MAX_ARCHIVE: usize = 64 * 1024 * 1024;
pub const MAX_ENTRY: usize = 8 * 1024 * 1024;
pub const MAX_DECODED: usize = 16 * 1024 * 1024;
pub const MAX_ENTRIES: usize = 4096;

#[derive(Debug, Serialize)]
pub struct Error {
    pub code: &'static str,
    pub source: String,
    pub byte_offset: usize,
    pub message: String,
}
impl Error {
    fn new(code: &'static str, source: &str, byte_offset: usize, message: &str) -> Self {
        Self {
            code,
            source: source.into(),
            byte_offset,
            message: message.into(),
        }
    }
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} at byte {}: {}: {}",
            self.source, self.byte_offset, self.code, self.message
        )
    }
}
impl std::error::Error for Error {}
pub type Result<T> = std::result::Result<T, Error>;

pub fn data_name(name: &str) -> Result<String> {
    let bytes = name.as_bytes();
    if bytes.is_empty()
        || bytes.len() > 255
        || !(bytes[0].is_ascii_alphanumeric() || bytes[0] == b'_')
        || !bytes
            .iter()
            .all(|c| c.is_ascii_alphanumeric() || b"_.-".contains(c))
    {
        return Err(Error::new(
            "car_data_name",
            "<car data>",
            0,
            "Expected a flat portable ASCII filename.",
        ));
    }
    Ok(name.to_ascii_lowercase())
}
pub fn folder_key(identifier: &str) -> Result<Vec<u8>> {
    if identifier.is_empty()
        || identifier.len() > 128
        || !identifier
            .bytes()
            .all(|c| c.is_ascii_alphanumeric() || b"_-".contains(&c))
    {
        return Err(Error::new(
            "acd_identifier",
            "<car data>",
            0,
            "Expected 1..128 ASCII letters, digits, underscores or hyphens.",
        ));
    }
    let chars = identifier.to_ascii_lowercase().into_bytes();
    let n = chars.len();
    let (mut pair, mut triple, mut product, mut remainder, mut adjacent) =
        (0i32, 0i32, 66i32, 171i32, 171i32);
    for i in (0..n.saturating_sub(1)).step_by(2) {
        pair = (pair * i32::from(chars[i]) - i32::from(chars[i + 1])) & 255;
    }
    for i in (1..n.saturating_sub(3)).step_by(3) {
        triple = triple.wrapping_mul(i32::from(chars[i])) / (i32::from(chars[i + 1]) + 27)
            - 27
            - i32::from(chars[i - 1]);
    }
    for i in (1..n.saturating_sub(4)).step_by(4) {
        product =
            ((i32::from(chars[i]) + 15) * product * (i32::from(chars[i - 1]) + 15) + 22) & 255;
    }
    let even = &chars[..n.saturating_sub(2)];
    for &v in even.iter().step_by(2) {
        remainder %= i32::from(v);
    }
    for v in chars.windows(2) {
        adjacent = adjacent / i32::from(v[0]) + i32::from(v[1]);
    }
    let values = [
        chars.iter().map(|&c| i32::from(c)).sum(),
        pair,
        triple,
        5763 - chars[1..].iter().map(|&c| i32::from(c)).sum::<i32>(),
        product,
        101 - even.iter().step_by(2).map(|&c| i32::from(c)).sum::<i32>(),
        remainder,
        adjacent,
    ];
    Ok(values.map(|v| (v as u8).to_string()).join("-").into_bytes())
}

#[derive(Debug, Serialize)]
pub struct Entry {
    pub name: String,
    pub offset: usize,
    pub length: usize,
}
pub struct Archive {
    pub source: String,
    pub header: Option<i32>,
    entries: BTreeMap<String, Entry>,
    data: Vec<u8>,
    key: Vec<u8>,
}
struct Reader<'a> {
    data: &'a [u8],
    cursor: usize,
    source: &'a str,
}
impl Reader<'_> {
    fn error(&self, code: &'static str, message: &str) -> Error {
        Error::new(code, self.source, self.cursor, message)
    }
    fn take(&mut self, length: usize) -> Result<&[u8]> {
        if length > self.data.len() - self.cursor {
            return Err(self.error(
                "acd_truncated",
                "Record field exceeds remaining archive bytes.",
            ));
        }
        let start = self.cursor;
        self.cursor += length;
        Ok(&self.data[start..self.cursor])
    }
    fn integer(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
}
impl Archive {
    pub fn entries(&self) -> &BTreeMap<String, Entry> {
        &self.entries
    }
    pub fn open(path: &Path, identifier: &str, source: &str) -> Result<Self> {
        let metadata =
            std::fs::metadata(path).map_err(|e| Error::new("acd_io", source, 0, &e.to_string()))?;
        if !metadata.is_file() {
            return Err(Error::new(
                "acd_io",
                source,
                0,
                "Expected a regular archive file.",
            ));
        }
        let mut data = Vec::new();
        std::fs::File::open(path)
            .and_then(|f| f.take(MAX_ARCHIVE as u64 + 1).read_to_end(&mut data))
            .map_err(|e| Error::new("acd_io", source, 0, &e.to_string()))?;
        Self::parse(data, identifier, source)
    }
    pub fn parse(data: Vec<u8>, identifier: &str, source: &str) -> Result<Self> {
        let key = folder_key(identifier)?;
        let mut r = Reader {
            data: &data,
            cursor: 0,
            source,
        };
        if data.len() > MAX_ARCHIVE {
            return Err(r.error("acd_limit", "Archive exceeds 64 MiB."));
        }
        let header = if data.starts_with(&(-1111i32).to_le_bytes()) {
            r.integer()?;
            Some(r.integer()?)
        } else {
            None
        };
        let mut entries = BTreeMap::new();
        let mut total = 0;
        while r.cursor < data.len() {
            if entries.len() >= MAX_ENTRIES {
                return Err(r.error("acd_limit", "Archive exceeds 4096 entries."));
            }
            let length = r.integer()?;
            if !(1..=255).contains(&length) {
                return Err(r.error("acd_name", "Name length must be 1..255 bytes."));
            }
            let name = std::str::from_utf8(r.take(length as usize)?)
                .unwrap_or("")
                .to_owned();
            let normalized = data_name(&name).map_err(|_| {
                r.error(
                    "acd_name",
                    "Entry name is outside the flat ASCII namespace.",
                )
            })?;
            if entries.contains_key(&normalized) {
                return Err(r.error("acd_duplicate", "Ambiguous case-insensitive archive name."));
            }
            let length = r.integer()?;
            if length < 0 || length as usize > MAX_ENTRY || total + length as usize > MAX_DECODED {
                return Err(r.error(
                    "acd_limit",
                    "Entry exceeds 8 MiB or combined decoded data exceeds 16 MiB.",
                ));
            }
            let length = length as usize;
            total += length;
            let offset = r.cursor;
            if length * 4 > data.len() - offset {
                return Err(r.error("acd_truncated", "Encoded entry payload is truncated."));
            }
            r.cursor += length * 4;
            entries.insert(
                normalized,
                Entry {
                    name,
                    offset,
                    length,
                },
            );
        }
        if entries.is_empty() {
            return Err(r.error("acd_empty", "Archive contains no supported records."));
        }
        Ok(Self {
            source: source.into(),
            header,
            entries,
            data,
            key,
        })
    }
    pub fn read(&self, name: &str) -> Result<Vec<u8>> {
        let normalized = data_name(name)?;
        let entry = self.entries.get(&normalized).ok_or_else(|| {
            Error::new(
                "car_data_missing",
                &self.source,
                0,
                &format!("Required car data file {name:?} is absent."),
            )
        })?;
        Ok(self.data[entry.offset..entry.offset + entry.length * 4]
            .iter()
            .step_by(4)
            .enumerate()
            .map(|(i, &v)| v.wrapping_sub(self.key[i % self.key.len()]))
            .collect())
    }
}
