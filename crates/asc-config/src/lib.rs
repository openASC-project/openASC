//! Bounded UTF-8 INI/LUT readers with source locations and strict ambiguity checks.
use serde::Serialize;
use std::{collections::BTreeMap, fmt, io::Read, path::Path};

const MAX_TEXT: usize = 4 * 1024 * 1024;
const MAX_RECORDS: usize = 100_000;

#[derive(Debug, Serialize)]
pub struct Error {
    pub code: &'static str,
    pub source: String,
    pub line: usize,
    pub message: String,
}
impl Error {
    pub fn new(code: &'static str, source: &str, line: usize, message: &str) -> Self {
        Self {
            code,
            source: source.into(),
            line,
            message: message.into(),
        }
    }
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}:{}: {}: {}",
            self.source, self.line, self.code, self.message
        )
    }
}
impl std::error::Error for Error {}
pub type Result<T> = std::result::Result<T, Error>;

pub fn read_text(path: &Path) -> Result<String> {
    let source = path.to_string_lossy();
    let mut bytes = Vec::new();
    std::fs::File::open(path)
        .and_then(|file| file.take(MAX_TEXT as u64 + 1).read_to_end(&mut bytes))
        .map_err(|e| Error::new("text_io", &source, 0, &e.to_string()))?;
    if bytes.len() > MAX_TEXT {
        return Err(Error::new("text_limit", &source, 0, "Text exceeds 4 MiB."));
    }
    String::from_utf8(bytes).map_err(|_| Error::new("text_encoding", &source, 0, "Expected UTF-8, optionally with a BOM; legacy encodings are unsupported in the native reader."))
}
fn lines<'a>(text: &'a str, source: &str) -> Result<Vec<(usize, &'a str)>> {
    if text.len() > MAX_TEXT {
        return Err(Error::new("text_limit", source, 0, "Text exceeds 4 MiB."));
    }
    let mut result = Vec::new();
    for (index, raw) in text.trim_start_matches('\u{feff}').lines().enumerate() {
        let number = index + 1;
        let raw = raw.trim();
        if raw.starts_with('#') || raw.starts_with("//") {
            continue;
        }
        let mut quoted = false;
        let mut end = raw.len();
        for (offset, ch) in raw.char_indices() {
            if ch == '"' {
                quoted = !quoted;
            }
            if ch == ';' && !quoted {
                end = offset;
                break;
            }
        }
        if quoted {
            return Err(Error::new(
                "unclosed_quote",
                source,
                number,
                "Unclosed double-quoted value.",
            ));
        }
        let line = raw[..end].trim();
        if !line.is_empty() {
            if result.len() >= MAX_RECORDS {
                return Err(Error::new(
                    "text_limit",
                    source,
                    number,
                    "Text exceeds 100000 records.",
                ));
            }
            result.push((number, line));
        }
    }
    Ok(result)
}

#[derive(Debug, Serialize)]
pub struct Value {
    pub text: String,
    pub line: usize,
}
#[derive(Debug, Serialize)]
pub struct Section {
    pub name: String,
    pub line: usize,
    pub values: BTreeMap<String, Value>,
}
#[derive(Debug, Serialize)]
pub struct Ini {
    pub source: String,
    pub sections: Vec<Section>,
}
impl Section {
    pub fn get(&self, key: &str) -> Option<&Value> {
        self.values.get(&key.to_uppercase())
    }
}
pub fn parse_ini(text: &str, source: &str) -> Result<Ini> {
    let mut sections: Vec<Section> = Vec::new();
    let mut names = std::collections::HashSet::new();
    for (number, line) in lines(text, source)? {
        if line.starts_with('[') {
            if !line.ends_with(']') || line[1..line.len() - 1].contains(['[', ']']) {
                return Err(Error::new(
                    "invalid_section",
                    source,
                    number,
                    "Expected a single [SECTION] header.",
                ));
            }
            let name = line[1..line.len() - 1].trim().to_uppercase();
            if name.is_empty() {
                return Err(Error::new(
                    "empty_section",
                    source,
                    number,
                    "Section name cannot be empty.",
                ));
            }
            if !names.insert(name.clone()) {
                return Err(Error::new(
                    "duplicate_section",
                    source,
                    number,
                    "Duplicate section.",
                ));
            }
            sections.push(Section {
                name,
                line: number,
                values: BTreeMap::new(),
            });
            continue;
        }
        let current = sections.last_mut().ok_or_else(|| {
            Error::new(
                "missing_section",
                source,
                number,
                "A value must belong to a section.",
            )
        })?;
        let (key, value) = line.split_once('=').ok_or_else(|| {
            Error::new("invalid_assignment", source, number, "Expected KEY=VALUE.")
        })?;
        let key = key.trim().to_uppercase();
        let value = value.trim();
        if key.is_empty() || key.chars().any(char::is_whitespace) || key.contains(['[', ']', '"']) {
            return Err(Error::new(
                "invalid_key",
                source,
                number,
                "Expected a nonempty key without whitespace, brackets or quotes.",
            ));
        }
        if current.values.contains_key(&key) {
            return Err(Error::new(
                "duplicate_key",
                source,
                number,
                "Duplicate key in section.",
            ));
        }
        let value = if value.starts_with('"')
            && value.ends_with('"')
            && value.len() >= 2
            && !value[1..value.len() - 1].contains('"')
        {
            &value[1..value.len() - 1]
        } else if value.contains('"') {
            return Err(Error::new(
                "invalid_quote",
                source,
                number,
                "Only a whole value may be double-quoted.",
            ));
        } else {
            value
        };
        current.values.insert(
            key,
            Value {
                text: value.into(),
                line: number,
            },
        );
    }
    Ok(Ini {
        source: source.into(),
        sections,
    })
}
pub fn read_ini(path: &Path) -> Result<Ini> {
    parse_ini(&read_text(path)?, &path.to_string_lossy())
}

pub fn finite_number(value: &str, source: &str, line: usize) -> Result<f64> {
    let number = value
        .trim()
        .parse::<f64>()
        .map_err(|_| Error::new("invalid_number", source, line, "Expected a number."))?;
    if !number.is_finite() {
        return Err(Error::new(
            "nonfinite_number",
            source,
            line,
            "NaN and infinity are unsupported.",
        ));
    }
    Ok(number)
}
pub struct Lut {
    pub source: String,
    points: Vec<[f64; 2]>,
}
impl Lut {
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
    pub fn sample(&self, x: f64) -> Result<f64> {
        if !x.is_finite() {
            return Err(Error::new(
                "nonfinite_number",
                &self.source,
                0,
                "Sample must be finite.",
            ));
        }
        let upper = self.points.partition_point(|p| p[0] <= x);
        if upper == 0 {
            return Ok(self.points[0][1]);
        }
        if upper == self.points.len() {
            return Ok(self.points[upper - 1][1]);
        }
        let [x0, y0] = self.points[upper - 1];
        let [x1, y1] = self.points[upper];
        let span = x1 - x0;
        let t = (x - x0) / span;
        let result = (1. - t) * y0 + t * y1;
        if !span.is_finite() || !result.is_finite() {
            return Err(Error::new(
                "numeric_range",
                &self.source,
                0,
                "Interpolation exceeds the supported numeric range.",
            ));
        }
        Ok(result)
    }
}
pub fn parse_lut(text: &str, source: &str) -> Result<Lut> {
    let mut points: Vec<[f64; 2]> = Vec::new();
    for (number, line) in lines(text, source)? {
        let fields: Vec<_> = line.split('|').collect();
        if fields.len() != 2 {
            return Err(Error::new(
                "invalid_lut_row",
                source,
                number,
                "Expected exactly x|y.",
            ));
        }
        let x = finite_number(fields[0], source, number)?;
        let y = finite_number(fields[1], source, number)?;
        if points.last().is_some_and(|p| x <= p[0]) {
            return Err(Error::new(
                "unordered_lut",
                source,
                number,
                "LUT x coordinates must strictly increase.",
            ));
        }
        points.push([x, y]);
    }
    if points.is_empty() {
        return Err(Error::new(
            "empty_lut",
            source,
            0,
            "A LUT must contain at least one point.",
        ));
    }
    Ok(Lut {
        source: source.into(),
        points,
    })
}
pub fn read_lut(path: &Path) -> Result<Lut> {
    parse_lut(&read_text(path)?, &path.to_string_lossy())
}
