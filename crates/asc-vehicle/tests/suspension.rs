use asc_vehicle::{data::Mode, suspension::load};
use std::path::{Path, PathBuf};
struct Directory(PathBuf);
impl Drop for Directory {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
        if let Some(parent) = self.0.parent() {
            let _ = std::fs::remove_dir(parent);
        }
    }
}
fn source() -> String {
    let mut text =
        "[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=3\nCG_LOCATION=.4\n[ARB]\nFRONT=3000\nREAR=1500\n"
            .to_owned();
    for (section, kind) in [("FRONT", "STRUT"), ("REAR", "DWB")] {
        text += &format!(
            "[{section}]\nTYPE={kind}\nTRACK=1.5\nBASEY=-.2\nROD_LENGTH=-.02\nHUB_MASS=30\n"
        );
        for name in [
            "WBCAR_BOTTOM_FRONT",
            "WBCAR_BOTTOM_REAR",
            "WBTYRE_BOTTOM",
            "WBCAR_STEER",
            "WBTYRE_STEER",
        ] {
            text += &format!("{name}=0.3,-0.1,0.2\n");
        }
        for name in if kind == "STRUT" {
            vec!["STRUT_CAR", "STRUT_TYRE"]
        } else {
            vec!["WBCAR_TOP_FRONT", "WBCAR_TOP_REAR", "WBTYRE_TOP"]
        } {
            text += &format!("{name}=0.1,0.2,0\n");
        }
        text += "SPRING_RATE=0\nPROGRESSIVE_SPRING_RATE=100\nBUMP_STOP_RATE=50000\nBUMPSTOP_UP=.1\nBUMPSTOP_DN=.08\nPACKER_RANGE=.15\nDAMP_BUMP=2000\nDAMP_FAST_BUMP=1000\nDAMP_FAST_BUMPTHRESHOLD=.12\nDAMP_REBOUND=3000\nDAMP_FAST_REBOUND=1500\nDAMP_FAST_REBOUNDTHRESHOLD=.15\nTOE_OUT=-.001\nSTATIC_CAMBER=-2\n";
    }
    text
}
fn archive(text: &str) -> Vec<u8> {
    let key = asc_acd::folder_key("fixture").unwrap();
    let name = "suspensions.ini";
    let mut bytes = Vec::new();
    bytes.extend((name.len() as i32).to_le_bytes());
    bytes.extend(name.as_bytes());
    bytes.extend((text.len() as i32).to_le_bytes());
    for (i, c) in text.bytes().enumerate() {
        bytes.extend([c.wrapping_add(key[i % key.len()]), 0, 0, 0]);
    }
    bytes
}
fn directory(packed: bool, text: &str) -> Directory {
    let root = asc_test_support::temp_path("suspension").join("fixture");
    std::fs::create_dir_all(&root).unwrap();
    if packed {
        std::fs::write(root.join("data.acd"), archive(text)).unwrap();
    } else {
        std::fs::create_dir(root.join("data")).unwrap();
        std::fs::write(root.join("data/suspensions.ini"), text).unwrap();
    }
    Directory(root)
}
fn replace(root: &Path, text: &str) {
    std::fs::write(root.join("data/suspensions.ini"), text).unwrap();
}
#[test]
fn packed_unpacked_records_preserve_both_geometry_families_source_numbers_and_lines() {
    let text = source();
    for packed in [false, true] {
        let dir = directory(packed, &text);
        let before = std::fs::read_dir(&dir.0).unwrap().count();
        let report = load(&dir.0, Mode::Auto).unwrap();
        assert!(!report.simulation_ready);
        assert_eq!(
            report.data_source,
            if packed { "packed" } else { "unpacked" }
        );
        assert_eq!(report.wheelbase_m, 3.);
        assert_eq!(report.front_weight_fraction, 0.4);
        assert_eq!(report.axles["front"].kind, "STRUT");
        assert_eq!(report.axles["front"].hardpoints.len(), 7);
        assert_eq!(report.axles["rear"].kind, "DWB");
        assert_eq!(report.axles["rear"].hardpoints.len(), 8);
        assert_eq!(report.axles["front"].rod_length_m, -0.02);
        assert_eq!(report.axles["front"].source_parameters["SPRING_RATE"], 0.);
        assert_eq!(
            report.axles["rear"].source_parameters["DAMP_FAST_REBOUNDTHRESHOLD"],
            0.15
        );
        assert_eq!(report.antiroll_source_parameters["FRONT"], 3000.);
        assert_eq!(
            report.field_sources["FRONT.STRUT_CAR"].line,
            text.lines()
                .position(|s| s.starts_with("STRUT_CAR="))
                .unwrap()
                + 1
        );
        assert_eq!(std::fs::read_dir(&dir.0).unwrap().count(), before);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.code == "suspension_configuration_scope")
        );
    }
}
#[test]
fn source_selection_is_whole_and_never_fills_in_a_missing_field_from_the_archive() {
    let dir = directory(false, &source());
    std::fs::write(dir.0.join("data.acd"), archive(&source())).unwrap();
    replace(
        &dir.0,
        &source().replace("DAMP_FAST_BUMPTHRESHOLD=.12\n", ""),
    );
    assert!(
        load(&dir.0, Mode::Auto)
            .err()
            .unwrap()
            .to_string()
            .contains("vehicle_field")
    );
    assert_eq!(load(&dir.0, Mode::Packed).unwrap().data_source, "packed");
    replace(&dir.0, &source());
    assert!(
        load(&dir.0, Mode::Auto)
            .unwrap()
            .diagnostics
            .iter()
            .any(|d| d.code == "car_data_precedence")
    );
}
#[test]
fn malformed_fields_unsupported_types_and_geometry_ranges_are_source_aware_errors() {
    let text = source();
    let dir = directory(false, &text);
    for (from, to, code) in [
        ("TYPE=STRUT", "TYPE=AXLE", "suspension_type"),
        ("WBTYRE_TOP=0.1,0.2,0\n", "", "vehicle_field"),
        (
            "STRUT_CAR=0.1,0.2,0",
            "STRUT_CAR=NaN,0,0",
            "nonfinite_number",
        ),
        (
            "STRUT_CAR=0.1,0.2,0",
            "STRUT_CAR=1001,0,0",
            "suspension_geometry_range",
        ),
        ("DAMP_BUMP=2000", "DAMP_BUMP=-1", "vehicle_range"),
        ("CG_LOCATION=.4", "CG_LOCATION=1", "suspension_range"),
        ("VERSION=1", "VERSION=999", "vehicle_version"),
    ] {
        assert!(text.contains(from));
        replace(&dir.0, &text.replace(from, to));
        let error = load(&dir.0, Mode::Auto).err().unwrap().to_string();
        assert!(error.contains(code), "{error}");
        assert!(error.contains("suspensions.ini"), "{error}");
    }
}
#[test]
fn optional_arb_and_uninterpreted_sections_are_not_implicit_solver_defaults() {
    let text = source()
        .replace("VERSION=1", "VERSION=2")
        .replace("[ARB]\nFRONT=3000\nREAR=1500\n", "")
        + "[HEAVE_FRONT]\nSPRING_RATE=10000\n";
    let dir = directory(false, &text);
    let report = load(&dir.0, Mode::Auto).unwrap();
    assert!(report.antiroll_source_parameters.is_empty());
    assert_eq!(report.header_version, 2);
    assert!(!report.simulation_ready);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|d| d.code == "suspension_fields_uninterpreted")
    );
}
