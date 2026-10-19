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

#[test]
fn cli_exposes_explicit_rates_loads_and_massless_balance_for_both_topologies() {
    let fixture = fixture();
    let rates = [0.17, 0.025, -0.012, 0.009];
    let force = [120., 2300., -300.];
    let torque = [40., -15., 25.];
    for axle in ["front", "rear"] {
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-motion")
            .arg(&fixture.0)
            .args([
                axle, "0.035", "0", "1", "0", "--rack", "0.004", "0", "0", "--rates", "0.17",
                "0.025", "-0.012", "0.009", "--load", "120", "2300", "-300", "40", "-15", "25",
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let value: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(value["simulation_ready"], false);
        let probe = &value["differential_probe"];
        assert_eq!(probe["mapping"]["columns"].as_array().unwrap().len(), 4);
        assert_eq!(probe["dynamic_vehicle_simulation"], false);
        assert_eq!(probe["generated_tyre_or_spring_loads"], false);
        assert_eq!(probe["support_reaction_excludes_travel_actuator"], true);
        assert!(
            (probe["relative_velocity"]["velocity_m_s"][1]
                .as_f64()
                .unwrap()
                - rates[0])
                .abs()
                < 1e-9
        );
        let v = &probe["relative_velocity"]["velocity_m_s"];
        let w = &probe["relative_velocity"]["angular_velocity_rad_s"];
        let power: f64 = (0..3)
            .map(|i| force[i] * v[i].as_f64().unwrap() + torque[i] * w[i].as_f64().unwrap())
            .sum();
        let q = &probe["generalized_loads"];
        let actuator = q["travel_n"].as_f64().unwrap() * rates[0]
            + (0..3)
                .map(|i| q["steering_anchor_n"][i].as_f64().unwrap() * rates[i + 1])
                .sum::<f64>();
        assert!((power - actuator).abs() < 1e-8);
        for key in ["force_balance_residual_n", "torque_balance_residual_nm"] {
            assert!(
                probe["held_massless_balance"][key]
                    .as_array()
                    .unwrap()
                    .iter()
                    .all(|v| v.as_f64().unwrap().abs() < 1e-8)
            );
        }
    }
    for extra in [
        vec!["--rates", "NaN", "0", "0", "0"],
        vec!["--load", "1e19", "0", "0", "0", "0", "0"],
        vec!["--rates", "1", "2"],
        vec!["--differential", "--differential"],
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-motion")
            .arg(&fixture.0)
            .args(["front", "0.035", "0", "1", "0"])
            .args(extra)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}

#[test]
fn cli_samples_explicit_spring_mounts_and_rejects_incomplete_or_invalid_laws() {
    let fixture = fixture();
    for axle in ["front", "rear"] {
        for damping in [
            vec![],
            vec!["--damping", "80", "20", "0.05", "140", "30", "0.07"],
        ] {
            let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
                .arg("suspension-motion")
                .arg(&fixture.0)
                .args([
                    axle, "0.03", "0", "1", "0", "--rates", "0.17", "0.025", "-0.012", "0.009",
                    "--spring", "0", "0.6", "0", "0.07", "-0.08", "0.1", "0.65", "12000",
                ])
                .args(&damping)
                .output()
                .unwrap();
            assert!(
                output.status.success(),
                "{}",
                String::from_utf8_lossy(&output.stderr)
            );
            let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
            let probe = &report["differential_probe"];
            let spring = &probe["explicit_spring_probe"];
            assert_eq!(probe["generated_tyre_or_spring_loads"], true);
            assert_eq!(probe["dynamic_vehicle_simulation"], false);
            assert_eq!(spring["automatic_source_parameters"], false);
            assert_eq!(spring["source_force_mapping_validated"], false);
            assert_eq!(spring["separate_from_explicit_probe_load"], true);
            assert!(spring["sample"]["length_m"].as_f64().unwrap() > 0.);
            let sample = &spring["sample"];
            let rate = sample["extension_rate_m_s"].as_f64().unwrap();
            let power = sample["generalized_loads"]["travel_n"].as_f64().unwrap() * 0.17
                + [0.025, -0.012, 0.009]
                    .into_iter()
                    .enumerate()
                    .map(|(i, v)| {
                        sample["generalized_loads"]["steering_anchor_n"][i]
                            .as_f64()
                            .unwrap()
                            * v
                    })
                    .sum::<f64>();
            let dissipated = sample["dissipated_power_w"].as_f64().unwrap();
            assert!(
                (power + sample["elastic_tension_n"].as_f64().unwrap() * rate + dissipated).abs()
                    < 1e-8
            );
            if damping.is_empty() {
                assert_eq!(dissipated, 0.);
            } else {
                assert!(dissipated > 0.);
            }
            for key in ["force_balance_residual_n", "torque_balance_residual_nm"] {
                assert!(
                    spring["held_massless_balance"][key]
                        .as_array()
                        .unwrap()
                        .iter()
                        .all(|v| v.as_f64().unwrap().abs() < 1e-8)
                );
            }
        }
    }
    for extra in [
        vec!["--damping", "1", "1", "0", "1", "1", "0"],
        vec!["--spring", "0", "0.6", "0", "0", "0", "0", "0.6", "-1"],
        vec!["--spring", "0", "0.6", "0"],
        vec!["--spring", "NaN", "0.6", "0", "0", "0", "0", "0.6", "1"],
        vec![
            "--spring",
            "0",
            "0.6",
            "0",
            "0",
            "0",
            "0",
            "0.6",
            "1",
            "--damping",
            "-1",
            "1",
            "0",
            "1",
            "1",
            "0",
        ],
        vec![
            "--spring", "0", "0.6", "0", "0", "0", "0", "0.6", "1", "--spring", "0", "0.6", "0",
            "0", "0", "0", "0.6", "1",
        ],
    ] {
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-motion")
            .arg(&fixture.0)
            .args(["front", "0.03", "0", "1", "0"])
            .args(extra)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}

#[test]
fn cli_runs_bounded_fixed_frame_dynamics_for_both_topologies_without_changing_source() {
    let fixture = fixture();
    let configuration = fixture.0.join("explicit-run.json");
    let text = include_str!("../../../examples/suspension-travel.json");
    fs::write(&configuration, text).unwrap();
    let source = fixture.0.join("data/suspensions.ini");
    let original = fs::read(&source).unwrap();
    for axle in ["front", "rear"] {
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-run")
            .arg(&fixture.0)
            .arg(axle)
            .arg(&configuration)
            .args(["--data", "unpacked"])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(report["scope"], "native_fixed_frame_upright_dynamics");
        assert_eq!(report["simulation_ready"], false);
        assert_eq!(report["automatic_source_mass_or_force_parameters"], false);
        assert_eq!(report["fixed_frame"], true);
        assert_eq!(report["fixed_rack"], true);
        let records = report["records"].as_array().unwrap();
        assert_eq!(records.len(), 26);
        assert_eq!(records.last().unwrap()["step"], 250);
        assert_eq!(records.last().unwrap()["time_s"], 0.25);
        let first = &records[0]["sample"];
        let last = &records.last().unwrap()["sample"];
        assert!((last["state"]["travel_m"].as_f64().unwrap() - 0.025).abs() > 0.001);
        let energy = |s: &serde_json::Value| {
            s["kinetic_energy_j"].as_f64().unwrap() + s["potential_energy_j"].as_f64().unwrap()
        };
        assert!(energy(last) < energy(first));
        for record in records {
            assert!(
                record["sample"]["constraint_balance"]["travel_actuator_n"]
                    .as_f64()
                    .unwrap()
                    .abs()
                    < 1e-7
            );
        }
    }
    assert_eq!(fs::read(source).unwrap(), original);
}

#[test]
fn cli_rejects_bad_dynamic_schema_resource_limits_and_failed_runs_without_partial_json() {
    let fixture = fixture();
    let configuration = fixture.0.join("explicit-run.json");
    let template: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/suspension-travel.json")).unwrap();
    let mutations: Vec<(&str, serde_json::Value)> = vec![
        ("schema_version", 2.into()),
        ("record_every", 0.into()),
        ("steps", 10001.into()),
        ("step_s", 0.1.into()),
        ("unrecognized", true.into()),
        ("mass_kg", (-1).into()),
        (
            "inertia_rows_kg_m2",
            serde_json::json!([[1, 0, 0], [0, 1, 0], [0, 0, 10]]),
        ),
        (
            "springs",
            serde_json::Value::Array(vec![template["springs"][0].clone(); 17]),
        ),
    ];
    let mut cases = Vec::new();
    for (key, value) in mutations {
        let mut config = template.clone();
        config[key] = value;
        cases.push(config.to_string());
    }
    let mut missing = template.clone();
    missing.as_object_mut().unwrap().remove("gravity_m_s2");
    cases.push(missing.to_string());
    let mut records = template.clone();
    records["steps"] = 10000.into();
    records["record_every"] = 1.into();
    cases.push(records.to_string());
    let mut failed = template.clone();
    failed["springs"] = serde_json::json!([]);
    failed["initial_travel_m"] = 0.into();
    failed["step_s"] = 0.02.into();
    failed["steps"] = 1.into();
    failed["record_every"] = 1.into();
    failed["external_force_n"] = serde_json::json!([0, 100000, 0]);
    cases.push(failed.to_string());
    cases.push(" ".repeat(16385));
    for text in cases {
        fs::write(&configuration, text).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("suspension-run")
            .arg(&fixture.0)
            .arg("front")
            .arg(&configuration)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}
