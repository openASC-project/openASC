use crate::{FieldSource, Result};
use asc_config::{Error, Ini, finite_number};
use std::collections::BTreeMap;

pub struct Fields {
    pub document: Ini,
    pub used: BTreeMap<String, FieldSource>,
}
impl Fields {
    pub fn new(document: Ini) -> Self {
        Self {
            document,
            used: BTreeMap::new(),
        }
    }
    pub fn value(&mut self, section: &str, key: &str) -> Result<String> {
        let block = self.document.sections.iter().find(|s| s.name == section);
        let value = block.and_then(|s| s.get(key)).ok_or_else(|| {
            Error::new(
                "vehicle_field",
                &self.document.source,
                block.map_or(0, |s| s.line),
                &format!("[{section}] requires {key}."),
            )
        })?;
        self.used.insert(
            format!("{section}.{key}"),
            FieldSource {
                source: self.document.source.clone(),
                line: value.line,
            },
        );
        Ok(value.text.clone())
    }
    pub fn line(&self, section: &str, key: &str) -> usize {
        self.document
            .sections
            .iter()
            .find(|s| s.name == section)
            .and_then(|s| s.get(key))
            .map_or(0, |v| v.line)
    }
    pub fn error(
        &self,
        section: &str,
        key: &str,
        code: &'static str,
        message: &str,
    ) -> Box<dyn std::error::Error> {
        Error::new(
            code,
            &self.document.source,
            self.line(section, key),
            message,
        )
        .into()
    }
    pub fn number(
        &mut self,
        section: &str,
        key: &str,
        min: f64,
        max: f64,
        positive: bool,
    ) -> Result<f64> {
        let text = self.value(section, key)?;
        let value = finite_number(&text, &self.document.source, self.line(section, key))?;
        if value < min || value > max || (positive && value <= 0.) {
            return Err(self.error(
                section,
                key,
                "vehicle_range",
                "Value is outside the supported range.",
            ));
        }
        Ok(value)
    }
    pub fn integer(&mut self, section: &str, key: &str, min: u32, max: u32) -> Result<u32> {
        let value = self.number(section, key, min as f64, max as f64, false)?;
        if value.fract() != 0. {
            return Err(self.error(section, key, "vehicle_range", "Expected an integer."));
        }
        Ok(value as u32)
    }
    pub fn vector(&mut self, section: &str, key: &str, positive: bool) -> Result<[f64; 3]> {
        let text = self.value(section, key)?;
        let fields: Vec<_> = text.split(',').take(4).collect();
        if fields.len() != 3 {
            return Err(self.error(
                section,
                key,
                "vehicle_vector",
                "Expected three comma-separated coordinates.",
            ));
        }
        let mut values = [0.; 3];
        for (v, field) in values.iter_mut().zip(fields) {
            *v = finite_number(field, &self.document.source, self.line(section, key))?;
        }
        if positive && values.iter().any(|v| *v <= 0.) {
            return Err(self.error(
                section,
                key,
                "vehicle_range",
                "Expected positive dimensions.",
            ));
        }
        Ok(values)
    }
    pub fn version(&mut self, supported: &[u32]) -> Result<()> {
        let version = self.integer("HEADER", "VERSION", 0, 10000)?;
        if !supported.contains(&version) {
            return Err(self.error(
                "HEADER",
                "VERSION",
                "vehicle_version",
                "Unsupported vehicle configuration version.",
            ));
        }
        Ok(())
    }
}
