use asc_vehicle::{
    data::{CarData, Mode},
    load,
};
use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
};

struct Fixture(PathBuf);
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn files() -> BTreeMap<&'static str, String> {
    asc_test_support::vehicle_files()
}
fn archive(files: &BTreeMap<&str, String>) -> Vec<u8> {
    let mut bytes = Vec::new();
    let key = asc_acd::folder_key("fixture").unwrap();
    for (name, text) in files {
        bytes.extend((name.len() as i32).to_le_bytes());
        bytes.extend(name.as_bytes());
        bytes.extend((text.len() as i32).to_le_bytes());
        for (i, v) in text.bytes().enumerate() {
            bytes.extend([v.wrapping_add(key[i % key.len()]), 0, 0, 0]);
        }
    }
    bytes
}
fn install(packed: bool) -> (Fixture, PathBuf) {
    let root = asc_test_support::temp_path("vehicle");
    let car = root.join("fixture");
    std::fs::create_dir_all(&car).unwrap();
    let data = files();
    if packed {
        std::fs::write(car.join("data.acd"), archive(&data)).unwrap();
    } else {
        std::fs::create_dir(car.join("data")).unwrap();
        for (name, text) in data {
            std::fs::write(car.join("data").join(name), text).unwrap();
        }
    }
    let mut model = b"sc6969".to_vec();
    model.extend(6u32.to_le_bytes());
    model.extend(0u32.to_le_bytes());
    std::fs::write(car.join("original.kn5"), model).unwrap();
    (Fixture(root), car)
}
fn replace(car: &Path, name: &str, from: &str, to: &str) {
    let path = car.join("data").join(name);
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains(from));
    std::fs::write(path, text.replace(from, to)).unwrap();
}
#[test]
fn packed_and_unpacked_sources_have_typed_values_and_field_locations_without_extraction() {
    for packed in [false, true] {
        let (_root, car) = install(packed);
        let before: Vec<_> = std::fs::read_dir(&car)
            .unwrap()
            .map(|e| e.unwrap().file_name())
            .collect();
        let v = load(&car, Mode::Auto, None).unwrap();
        assert_eq!((v.mass_kg, v.wheelbase_m), (900., 3.));
        assert_eq!(v.drivetrain.forward_ratios, [3., 2., 1.2]);
        assert_eq!(v.inertia_dimensions_m, [1.5, 1.2, 3.5]);
        assert_eq!(v.field_sources["car.ini"]["BASIC.TOTALMASS"].line, 4);
        assert!(!v.simulation_ready);
        assert_eq!(v.model.kn5.scope, "header_only");
        assert!((v.wheel_offsets_m["front_left"][2] - 1.8).abs() < 1e-12);
        assert!((v.wheel_offsets_m["rear_right"][2] + 1.2).abs() < 1e-12);
        assert_eq!(v.data_source, if packed { "packed" } else { "unpacked" });
        assert_eq!(
            before,
            std::fs::read_dir(&car)
                .unwrap()
                .map(|e| e.unwrap().file_name())
                .collect::<Vec<_>>()
        );
    }
}
#[test]
fn source_selection_is_whole_directory_and_does_not_fall_back_per_file() {
    let (_root, car) = install(false);
    std::fs::write(car.join("data.acd"), archive(&files())).unwrap();
    let v = load(&car, Mode::Auto, None).unwrap();
    assert_eq!(v.data_source, "unpacked");
    assert!(
        v.diagnostics
            .iter()
            .any(|d| d.code == "car_data_precedence")
    );
    std::fs::remove_file(car.join("data/engine.ini")).unwrap();
    assert!(load(&car, Mode::Auto, None).is_err());
    assert!(load(&car, Mode::Packed, None).is_ok());
}
#[test]
fn compounds_versions_ranges_and_curve_coverage_fail_explicitly() {
    let (_root, car) = install(false);
    assert_eq!(
        load(&car, Mode::Auto, Some(1)).unwrap().axles["front"].tyre_radius_m,
        0.32
    );
    assert!(load(&car, Mode::Auto, Some(2)).is_err());
    assert!(load(&car, Mode::Auto, Some(100)).is_err());
    for (name, from, to, code) in [
        (
            "car.ini",
            "TOTALMASS=900",
            "TOTALMASS=nan",
            "nonfinite_number",
        ),
        (
            "car.ini",
            "INERTIA=1.5,1.2,3.5",
            "INERTIA=1,0,2",
            "vehicle_range",
        ),
        ("tyres.ini", "VERSION=10", "VERSION=999", "vehicle_version"),
        ("drivetrain.ini", "GEAR_R=-3", "GEAR_R=3", "vehicle_range"),
        (
            "engine.ini",
            "LIMITER=6000",
            "LIMITER=7000",
            "vehicle_power_domain",
        ),
        (
            "suspensions.ini",
            "CG_LOCATION=.4",
            "CG_LOCATION=1",
            "vehicle_range",
        ),
    ] {
        replace(&car, name, from, to);
        assert!(
            load(&car, Mode::Auto, None)
                .err()
                .unwrap()
                .to_string()
                .contains(code)
        );
        replace(&car, name, to, from);
    }
}
#[test]
fn model_and_lut_paths_are_confined_and_model_check_remains_header_only() {
    let (_root, car) = install(false);
    for file in ["../outside.kn5", "C:\\outside.kn5", "/outside.kn5"] {
        replace(&car, "lods.ini", "original.kn5", file);
        assert!(
            load(&car, Mode::Auto, None)
                .err()
                .unwrap()
                .to_string()
                .contains("resource_escape")
        );
        replace(&car, "lods.ini", file, "original.kn5");
    }
    replace(
        &car,
        "engine.ini",
        "POWER_CURVE=power.lut",
        "POWER_CURVE=../power.lut",
    );
    assert!(
        load(&car, Mode::Auto, None)
            .err()
            .unwrap()
            .to_string()
            .contains("car_data_name")
    );
}
#[test]
fn unsupported_turbo_and_unused_fields_are_reported_not_simulated() {
    let (_root, car) = install(false);
    replace(
        &car,
        "engine.ini",
        "INERTIA=.15",
        "INERTIA=.15\n[TURBO_0]\nMAX_BOOST=1.2",
    );
    let v = load(&car, Mode::Auto, None).unwrap();
    assert!(
        v.diagnostics
            .iter()
            .any(|d| d.code == "vehicle_turbo_unimplemented")
    );
    assert!(
        v.diagnostics
            .iter()
            .any(|d| d.code == "vehicle_fields_uninterpreted")
    );
}
#[test]
fn unpacked_text_encoding_and_loaded_file_budget_are_bounded() {
    let (_root, car) = install(false);
    std::fs::write(car.join("data/car.ini"), [0xff]).unwrap();
    assert!(
        load(&car, Mode::Auto, None)
            .err()
            .unwrap()
            .to_string()
            .contains("text_encoding")
    );
    let file = std::fs::File::create(car.join("data/huge.txt")).unwrap();
    file.set_len(asc_acd::MAX_ENTRY as u64 + 1).unwrap();
    let mut data = CarData::open(&car, Mode::Unpacked).unwrap();
    assert!(
        data.text("huge.txt")
            .err()
            .unwrap()
            .to_string()
            .contains("car_data_limit")
    );
}
#[cfg(unix)]
#[test]
fn symlinked_model_and_data_file_cannot_escape_selected_roots() {
    let (_root, car) = install(false);
    let (_outside, other) = install(false);
    std::fs::remove_file(car.join("data/car.ini")).unwrap();
    std::os::unix::fs::symlink(other.join("data/car.ini"), car.join("data/car.ini")).unwrap();
    assert!(
        load(&car, Mode::Auto, None)
            .err()
            .unwrap()
            .to_string()
            .contains("resource_escape")
    );
    std::fs::remove_file(car.join("data/car.ini")).unwrap();
    std::fs::write(car.join("data/car.ini"), &files()["car.ini"]).unwrap();
    std::fs::remove_file(car.join("original.kn5")).unwrap();
    std::os::unix::fs::symlink(other.join("original.kn5"), car.join("original.kn5")).unwrap();
    assert!(
        load(&car, Mode::Auto, None)
            .err()
            .unwrap()
            .to_string()
            .contains("resource_escape")
    );
}

#[cfg(unix)]
#[test]
fn ambiguous_case_insensitive_unpacked_names_are_rejected() {
    let (_root, car) = install(false);
    std::fs::write(car.join("data/CAR.INI"), &files()["car.ini"]).unwrap();
    assert!(
        CarData::open(&car, Mode::Unpacked)
            .err()
            .unwrap()
            .to_string()
            .contains("car_data_duplicate")
    );
}

#[cfg(unix)]
#[test]
fn a_whole_data_directory_symlink_cannot_escape_the_car_root() {
    let (_root, car) = install(true);
    let (_other, outside) = install(false);
    std::os::unix::fs::symlink(outside.join("data"), car.join("data")).unwrap();
    assert!(
        CarData::open(&car, Mode::Auto)
            .err()
            .unwrap()
            .to_string()
            .contains("resource_escape")
    );
    assert!(load(&car, Mode::Packed, None).is_ok());
}
