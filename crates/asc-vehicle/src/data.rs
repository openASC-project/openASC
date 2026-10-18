use crate::{Diagnostic, Result};
use asc_config::Error;
use std::{
    collections::BTreeMap,
    io::Read,
    path::{Path, PathBuf},
};

#[derive(Clone, Copy)]
pub enum Mode {
    Auto,
    Packed,
    Unpacked,
}
enum Source {
    Packed(asc_acd::Archive),
    Unpacked(BTreeMap<String, PathBuf>),
}
pub struct CarData {
    pub car: String,
    pub root: PathBuf,
    pub mode: &'static str,
    pub diagnostics: Vec<Diagnostic>,
    source: Source,
    cache: BTreeMap<String, String>,
    bytes: usize,
}
pub fn confined(root: &Path, relative: &str, source: &str, line: usize) -> Result<PathBuf> {
    let portable = relative.replace('\\', "/");
    if portable.is_empty()
        || portable.starts_with('/')
        || portable.contains([':', '\0'])
        || portable.split('/').any(|v| v == "..")
    {
        return Err(Error::new(
            "resource_escape",
            source,
            line,
            "Expected a confined relative resource path without drives or parent traversal.",
        )
        .into());
    }
    let path = root
        .join(portable)
        .canonicalize()
        .map_err(|e| Error::new("resource_io", source, line, &e.to_string()))?;
    if !path.starts_with(root) {
        return Err(Error::new(
            "resource_escape",
            source,
            line,
            "Resource resolves outside the selected content root.",
        )
        .into());
    }
    Ok(path)
}
impl CarData {
    pub fn open(directory: &Path, mode: Mode) -> Result<Self> {
        let car = directory
            .file_name()
            .and_then(|v| v.to_str())
            .ok_or_else(|| {
                Error::new(
                    "acd_identifier",
                    "<car>",
                    0,
                    "Expected a car directory identifier.",
                )
            })?
            .to_owned();
        asc_acd::folder_key(&car)?;
        let source_name = format!("content/cars/{car}");
        let root = directory
            .canonicalize()
            .map_err(|e| Error::new("car_data_io", &source_name, 0, &e.to_string()))?;
        let metadata = |name: &str| -> Result<Option<std::fs::Metadata>> {
            match std::fs::metadata(root.join(name)) {
                Ok(value) => Ok(Some(value)),
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(None),
                Err(e) => Err(Error::new("car_data_io", &source_name, 0, &e.to_string()).into()),
            }
        };
        let unpacked = metadata("data")?.is_some_and(|v| v.is_dir());
        let packed = metadata("data.acd")?.is_some_and(|v| v.is_file());
        let mut diagnostics = Vec::new();
        if matches!(mode, Mode::Auto) && unpacked && packed {
            diagnostics.push(Diagnostic::warning("car_data_precedence",&source_name,"Both sources exist; unpacked data is selected as a whole without per-file archive fallback."));
        }
        let unpacked = matches!(mode, Mode::Unpacked) || (matches!(mode, Mode::Auto) && unpacked);
        let source = if unpacked {
            let directory = confined(&root, "data", &source_name, 0)?;
            let mut names = BTreeMap::new();
            for entry in std::fs::read_dir(&directory)? {
                let path = entry?.path();
                if !path.is_file() {
                    continue;
                }
                let name = path.file_name().and_then(|v| v.to_str()).ok_or_else(|| {
                    Error::new(
                        "car_data_name",
                        &source_name,
                        0,
                        "Non-UTF-8 car data filename.",
                    )
                })?;
                let key = asc_acd::data_name(name)?;
                if names.insert(key, path).is_some() {
                    return Err(Error::new(
                        "car_data_duplicate",
                        &source_name,
                        0,
                        "Ambiguous case-insensitive unpacked filename.",
                    )
                    .into());
                }
                if names.len() > asc_acd::MAX_ENTRIES {
                    return Err(Error::new(
                        "car_data_limit",
                        &source_name,
                        0,
                        "Unpacked data exceeds 4096 files.",
                    )
                    .into());
                }
            }
            Source::Unpacked(names)
        } else {
            let path = confined(&root, "data.acd", &source_name, 0)?;
            Source::Packed(asc_acd::Archive::open(
                &path,
                &car,
                &format!("{source_name}/data.acd"),
            )?)
        };
        Ok(Self {
            car,
            root,
            mode: if unpacked { "unpacked" } else { "packed" },
            diagnostics,
            source,
            cache: BTreeMap::new(),
            bytes: 0,
        })
    }
    pub fn source(&self, name: &str) -> String {
        if self.mode == "packed" {
            format!("content/cars/{}/data.acd::{name}", self.car)
        } else {
            format!("content/cars/{}/data/{name}", self.car)
        }
    }
    pub fn text(&mut self, name: &str) -> Result<&str> {
        let key = asc_acd::data_name(name)?;
        if !self.cache.contains_key(&key) {
            let source = self.source(name);
            let data = match &self.source {
                Source::Packed(archive) => archive.read(&key)?,
                Source::Unpacked(names) => {
                    let path = names
                        .get(&key)
                        .ok_or_else(|| {
                            Error::new(
                                "car_data_missing",
                                &source,
                                0,
                                "Required car data file is absent.",
                            )
                        })?
                        .canonicalize()?;
                    let directory = self.root.join("data").canonicalize()?;
                    if !path.starts_with(&directory) || !path.starts_with(&self.root) {
                        return Err(Error::new(
                            "resource_escape",
                            &source,
                            0,
                            "Car data file resolves outside the selected directory.",
                        )
                        .into());
                    }
                    let mut bytes = Vec::new();
                    std::fs::File::open(path)?
                        .take(asc_acd::MAX_ENTRY as u64 + 1)
                        .read_to_end(&mut bytes)?;
                    bytes
                }
            };
            if data.len() > asc_acd::MAX_ENTRY || self.bytes + data.len() > asc_acd::MAX_DECODED {
                return Err(Error::new(
                    "car_data_limit",
                    &source,
                    0,
                    "Car data exceeds 8 MiB per file or 16 MiB loaded bytes.",
                )
                .into());
            }
            let text=String::from_utf8(data).map_err(|_|Error::new("text_encoding",&source,0,"Expected UTF-8 with an optional BOM; verify the original car folder key. Legacy encodings are unsupported."))?;
            self.bytes += text.len();
            self.cache.insert(key.clone(), text);
        }
        Ok(&self.cache[&key])
    }
    pub fn ini(&mut self, name: &str) -> Result<asc_config::Ini> {
        let source = self.source(name);
        Ok(asc_config::parse_ini(self.text(name)?, &source)?)
    }
}
