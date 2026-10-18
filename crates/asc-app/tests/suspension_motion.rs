use std::{fs, path::PathBuf, process::Command};
struct Fixture(PathBuf);
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = fs::remove_file(self.0.join("data/suspensions.ini"));
        let _ = fs::remove_dir(self.0.join("data"));
        let _ = fs::remove_dir(&self.0);
    }
}
fn fixture() -> Fixture {
    let root = asc_test_support::temp_path("motion-cli");
    fs::create_dir_all(root.join("data")).unwrap();
    let mut text = "[HEADER]\nVERSION=1\n[BASIC]\nWHEELBASE=2.4\nCG_LOCATION=.5\n".to_owned();
    for (axle, kind) in [("FRONT", "STRUT"), ("REAR", "DWB")] {
        text +=
            &format!("[{axle}]\nTYPE={kind}\nTRACK=1.5\nBASEY=-.2\nROD_LENGTH=.1\nHUB_MASS=30\n");
        text += "WBCAR_TOP_FRONT=-.3,.2,.12\nWBCAR_TOP_REAR=-.3,.2,-.12\nWBTYRE_TOP=0,.2,0\nWBCAR_BOTTOM_FRONT=-.3,-.2,.12\nWBCAR_BOTTOM_REAR=-.3,-.2,-.12\nWBTYRE_BOTTOM=0,-.2,0\nWBCAR_STEER=-.3,0,.15\nWBTYRE_STEER=0,0,.15\nSTRUT_CAR=-.2,.4,0\nSTRUT_TYRE=0,-.2,0\n";
        for key in [
            "SPRING_RATE",
            "PROGRESSIVE_SPRING_RATE",
            "BUMP_STOP_RATE",
            "BUMPSTOP_UP",
            "BUMPSTOP_DN",
            "PACKER_RANGE",
            "DAMP_BUMP",
            "DAMP_FAST_BUMP",
            "DAMP_FAST_BUMPTHRESHOLD",
            "DAMP_REBOUND",
            "DAMP_FAST_REBOUND",
            "DAMP_FAST_REBOUNDTHRESHOLD",
            "TOE_OUT",
            "STATIC_CAMBER",
        ] {
            text += &format!("{key}=0\n");
        }
    }
    fs::write(root.join("data/suspensions.ini"), text).unwrap();
    Fixture(root)
}
#[test]
fn cli_solves_original_source_fixture_and_rejects_unsupported_or_ambiguous_requests() {
    let fixture = fixture();
    let file = fixture.0.join("data/suspensions.ini");
    let before = fs::read(&file).unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("suspension-motion")
        .arg(&fixture.0)
        .args(["rear", "0.05", "0", "1", "0", "--data", "unpacked"])
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let value: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(value["original_kinematics_validated"], false);
    assert_eq!(value["simulation_ready"], false);
    assert!(
        (value["solution"]["position_m"][0].as_f64().unwrap()
            - (-0.3 + (0.09_f64 - 0.0025).sqrt()))
        .abs()
            < 1e-9
    );
    let front = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("suspension-motion")
        .arg(&fixture.0)
        .args(["front", "0.025", "0", "1", "0", "--rack", "0.002", "0", "0"])
        .output()
        .unwrap();
    assert!(
        front.status.success(),
        "{}",
        String::from_utf8_lossy(&front.stderr)
    );
    let front: serde_json::Value = serde_json::from_slice(&front.stdout).unwrap();
    assert_eq!(front["type"], "STRUT");
    assert_eq!(front["scope"], "native_strut_raw_design_geometry");
    assert_eq!(front["rest_link_lengths_m"].as_array().unwrap().len(), 3);
    assert!(
        front["solution"]["coaxial_errors_m"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| v.as_f64().unwrap().abs() <= 1e-10)
    );
    assert!(front["solution"]["strut_length_m"].as_f64().unwrap() > 0.);
    assert_eq!(front["original_kinematics_validated"], false);
    for args in [
        vec!["unknown", "0.05", "0", "1", "0"],
        vec!["rear", "0.05", "0", "0", "0"],
        vec!["rear", "NaN", "0", "1", "0"],
        vec![
            "rear", "0.05", "0", "1", "0", "--rack", "0", "0", "0", "--rack", "0", "0", "0",
        ],
    ] {
        let bad = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-motion")
            .arg(&fixture.0)
            .args(args)
            .output()
            .unwrap();
        assert!(!bad.status.success());
        assert!(bad.stdout.is_empty());
    }
    assert_eq!(fs::read(file).unwrap(), before);
}
