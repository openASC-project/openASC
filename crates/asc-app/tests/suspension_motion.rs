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

fn coupled_configuration() -> serde_json::Value {
    let mut config: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/chassis-suspension.json")).unwrap();
    for upright in config["uprights"].as_array_mut().unwrap() {
        upright["car_directory"] = ".".into();
    }
    config
}
#[test]
fn cli_runs_four_corner_chassis_recoil_with_explicit_relative_paths_and_mass_mapping() {
    let fixture = fixture();
    let config = fixture.0.join("coupled.json");
    fs::write(&config, coupled_configuration().to_string()).unwrap();
    let source = fixture.0.join("data/suspensions.ini");
    let original = fs::read(&source).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("chassis-run")
        .arg(&config)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["scope"], "native_coupled_chassis_upright_dynamics");
    assert_eq!(report["moving_chassis"], true);
    assert_eq!(report["simulation_ready"], false);
    assert_eq!(
        report["automatic_source_frames_mass_or_force_parameters"],
        false
    );
    assert_eq!(report["source_geometry"].as_array().unwrap().len(), 4);
    let records = report["records"].as_array().unwrap();
    assert_eq!(records.len(), 26);
    let first = &records[0]["sample"];
    let last = &records.last().unwrap()["sample"];
    assert_eq!(last["total_mass_kg"], 1140.);
    assert!(last["chassis"]["position_m"][1].as_f64().unwrap().abs() > 1e-4);
    let energy = |s: &serde_json::Value| {
        s["kinetic_energy_j"].as_f64().unwrap() + s["potential_energy_j"].as_f64().unwrap()
    };
    assert!(energy(last) < energy(first));
    for record in records {
        let s = &record["sample"];
        assert!(
            s["linear_momentum_world_kg_m_s"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| v.as_f64().unwrap().abs() < 0.005)
        );
        for u in s["uprights"].as_array().unwrap() {
            assert!(
                u["constraint_balance_in_source_frame"]["travel_actuator_n"]
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
fn cli_rejects_coupled_schema_frame_resource_limits_and_failed_run_without_partial_json() {
    let fixture = fixture();
    let path = fixture.0.join("coupled.json");
    let template = coupled_configuration();
    let mut cases = Vec::new();
    for (key, value) in [
        ("schema_version", 2.into()),
        ("record_every", 0.into()),
        ("steps", 10001.into()),
        ("step_s", 0.1.into()),
        ("unrecognized", true.into()),
        ("uprights", serde_json::json!([])),
    ] {
        let mut c = template.clone();
        c[key] = value;
        cases.push(c);
    }
    let mut count = template.clone();
    count["uprights"] = serde_json::Value::Array(vec![template["uprights"][0].clone(); 5]);
    cases.push(count);
    let mut frame = template.clone();
    frame["uprights"][0]["frame_orientation_xyzw"] = serde_json::json!([0, 0, 0, 0]);
    cases.push(frame);
    let mut mode = template.clone();
    mode["uprights"][0]["data"] = "other".into();
    cases.push(mode);
    let mut records = template.clone();
    records["steps"] = 10000.into();
    records["record_every"] = 1.into();
    cases.push(records);
    let mut missing = template.clone();
    missing["chassis"]
        .as_object_mut()
        .unwrap()
        .remove("mass_kg");
    cases.push(missing);
    let mut failed = template.clone();
    failed["step_s"] = 0.02.into();
    failed["steps"] = 1.into();
    failed["record_every"] = 1.into();
    for u in failed["uprights"].as_array_mut().unwrap() {
        u["springs"] = serde_json::json!([]);
        u["initial_travel_m"] = 0.into();
    }
    failed["uprights"][3]["external_force_world_n"] = serde_json::json!([0, 100000, 0]);
    cases.push(failed);
    let mut texts: Vec<_> = cases.into_iter().map(|c| c.to_string()).collect();
    texts.push(" ".repeat(65537));
    for text in texts {
        fs::write(&path, text).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}

fn wheel_configuration() -> serde_json::Value {
    let mut config: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/chassis-wheels.json")).unwrap();
    for u in config["uprights"].as_array_mut().unwrap() {
        u["car_directory"] = ".".into();
    }
    config
}
#[test]
fn cli_couples_four_explicit_wheels_motor_reactions_and_additional_mass() {
    let fixture = fixture();
    let path = fixture.0.join("wheels.json");
    fs::write(&path, wheel_configuration().to_string()).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("chassis-run")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["wheel_spin_dofs"], 4);
    assert_eq!(report["wheel_mass_is_additional_to_upright"], true);
    assert_eq!(report["simulation_ready"], false);
    let records = report["records"].as_array().unwrap();
    assert_eq!(records.len(), 26);
    assert_eq!(records[0]["sample"]["total_mass_kg"], 1140.);
    for u in records.last().unwrap()["sample"]["uprights"]
        .as_array()
        .unwrap()
    {
        let wheel = &u["wheel"];
        assert!(wheel["state"]["rate_rad_s"].as_f64().unwrap().abs() > 0.1);
        assert!(wheel["state"]["angle_rad"].as_f64().unwrap().abs() > 0.01);
        assert!(wheel["spin_balance_residual_nm"].as_f64().unwrap().abs() < 1e-7);
        let drive = wheel["drive_torque_nm"].as_f64().unwrap();
        for i in 0..3 {
            assert!(
                (wheel["motor_reaction_on_upright_world_nm"][i]
                    .as_f64()
                    .unwrap()
                    + drive * wheel["axis_world"][i].as_f64().unwrap())
                .abs()
                    < 1e-8
            );
        }
    }
}
#[test]
fn cli_rejects_invalid_rotor_inertia_axis_state_and_unknown_wheel_fields() {
    let fixture = fixture();
    let path = fixture.0.join("wheels.json");
    let template = wheel_configuration();
    let mut cases = Vec::new();
    for (key, value) in [
        ("axle_in_upright", serde_json::json!([0, 0, 0])),
        ("mass_kg", (-1).into()),
        ("axial_inertia_kg_m2", 10.into()),
        ("initial_rate_rad_s", 100001.into()),
        ("initial_angle_rad", 1e10.into()),
        ("unknown", true.into()),
    ] {
        let mut c = template.clone();
        c["uprights"][0]["wheel"][key] = value;
        cases.push(c);
    }
    let mut missing = template.clone();
    missing["uprights"][0]["wheel"]
        .as_object_mut()
        .unwrap()
        .remove("transverse_inertia_kg_m2");
    cases.push(missing);
    for c in cases {
        fs::write(&path, c.to_string()).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}

fn normal_configuration() -> serde_json::Value {
    let mut config: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/chassis-normal-plane.json")).unwrap();
    for u in config["uprights"].as_array_mut().unwrap() {
        u["car_directory"] = ".".into();
    }
    config["steps"] = 200.into();
    config
}

#[test]
fn cli_brakes_actuate_airborne_wheels_without_requiring_a_support_provider() {
    let fixture = fixture();
    let path = fixture.0.join("airborne-brake.json");
    let mut c: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/chassis-wheels.json")).unwrap();
    c["steps"] = 100.into();
    c["record_every"] = 10.into();
    c["controls"] = serde_json::json!([{"time_s":0,"drive_fraction":0,"brake_fraction":1}]);
    for (i, u) in c["uprights"].as_array_mut().unwrap().iter_mut().enumerate() {
        u["car_directory"] = ".".into();
        u["wheel"]["drive_torque_nm"] = 0.into();
        u["wheel"]["initial_rate_rad_s"] = if i % 2 == 0 { 10 } else { -10 }.into();
        u["wheel"]["brake"] = serde_json::json!({"maximum_torque_nm":20,"transition_rate_rad_s":5});
    }
    fs::write(&path, c.to_string()).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("chassis-run")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let r: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(r["normal_contact_wheels"], 0);
    assert_eq!(r["wheel_brake_models"], 4);
    let rows = r["records"].as_array().unwrap();
    for row in rows {
        let d = row["brake_dissipated_power_w"].as_f64().unwrap();
        assert!(d > 0.);
        assert!(
            (row["total_dissipated_power_w"].as_f64().unwrap()
                - row["sample"]["dissipated_power_w"].as_f64().unwrap()
                - d)
                .abs()
                < 1e-8
        );
        assert_eq!(row["motor_power_w"], 0.);
    }
    for u in rows.last().unwrap()["sample"]["uprights"]
        .as_array()
        .unwrap()
    {
        assert!(u["wheel"]["state"]["rate_rad_s"].as_f64().unwrap().abs() < 9.);
    }
    fs::remove_file(path).unwrap();
}

#[test]
fn cli_scheduled_drive_brake_reports_actual_torque_power_and_slows_the_vehicle() {
    let fixture = fixture();
    let path = fixture.0.join("brake.json");
    let mut c = normal_configuration();
    c["support_plane"]["friction_scale"] = 1.into();
    c["steps"] = 2000.into();
    c["record_every"] = 20.into();
    c["controls"] = serde_json::json!([
        {"time_s":0,"drive_fraction":1,"brake_fraction":0},
        {"time_s":0.4,"drive_fraction":1,"brake_fraction":0},
        {"time_s":0.5,"drive_fraction":0,"brake_fraction":1},
        {"time_s":1,"drive_fraction":0,"brake_fraction":1}]);
    for (i, u) in c["uprights"].as_array_mut().unwrap().iter_mut().enumerate() {
        u["wheel"]["tangential_contact"] = serde_json::json!({"longitudinal_viscosity_n_s_m":3000,"lateral_viscosity_n_s_m":4000,"longitudinal_friction":0.9,"lateral_friction":0.9});
        u["wheel"]["drive_torque_nm"] = if i % 2 == 0 { 100 } else { -100 }.into();
        u["wheel"]["brake"] =
            serde_json::json!({"maximum_torque_nm":400,"transition_rate_rad_s":5});
    }
    let invoke = |c: &serde_json::Value| {
        fs::write(&path, c.to_string()).unwrap();
        Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap()
    };
    let output = invoke(&c);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let r: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(r["scheduled_wheel_controls"], true);
    assert_eq!(r["wheel_brake_models"], 4);
    assert_eq!(r["static_wheel_lock"], false);
    let records = r["records"].as_array().unwrap();
    let peak = records
        .iter()
        .map(|r| {
            r["sample"]["chassis"]["velocity_world_m_s"][2]
                .as_f64()
                .unwrap()
        })
        .fold(0., f64::max);
    let final_speed = records.last().unwrap()["sample"]["chassis"]["velocity_world_m_s"][2]
        .as_f64()
        .unwrap();
    assert!(peak > 0.2);
    assert!(
        final_speed >= -1e-5 && final_speed < 0.3 * peak,
        "{final_speed}/{peak}"
    );
    for record in records {
        let time = record["time_s"].as_f64().unwrap();
        let control = &record["controls"];
        let drive = control["drive_fraction"].as_f64().unwrap();
        let brake = control["brake_fraction"].as_f64().unwrap();
        if time <= 0.4 {
            assert_eq!(drive, 1.);
            assert_eq!(brake, 0.);
        } else if time >= 0.5 {
            assert_eq!(drive, 0.);
            assert_eq!(brake, 1.);
        }
        let mut dissipation = 0.;
        let mut motor_power = 0.;
        for (i, (b, u)) in record["wheel_brakes"]
            .as_array()
            .unwrap()
            .iter()
            .zip(record["sample"]["uprights"].as_array().unwrap())
            .enumerate()
        {
            let rate = u["wheel"]["state"]["rate_rad_s"].as_f64().unwrap();
            let torque = b["torque_on_wheel_nm"].as_f64().unwrap();
            let motor = if i % 2 == 0 { 100. } else { -100. } * drive;
            assert!(torque * rate <= 0.);
            dissipation -= torque * rate;
            motor_power += motor * rate;
            assert!(
                (u["wheel"]["drive_torque_nm"].as_f64().unwrap() - motor - torque).abs() < 1e-9
            );
        }
        assert!((dissipation - record["brake_dissipated_power_w"].as_f64().unwrap()).abs() < 1e-8);
        assert!((motor_power - record["motor_power_w"].as_f64().unwrap()).abs() < 1e-8);
    }
    for case in 0..5 {
        let mut bad = c.clone();
        match case {
            0 => bad["controls"] = serde_json::json!([]),
            1 => bad["controls"][1]["time_s"] = 0.into(),
            2 => bad["controls"][0]["brake_fraction"] = 2.into(),
            3 => bad["uprights"][0]["wheel"]["brake"]["maximum_torque_nm"] = (-1).into(),
            _ => bad["uprights"][0]["wheel"]["brake"]["transition_rate_rad_s"] = 0.into(),
        }
        let r = invoke(&bad);
        assert!(!r.status.success());
        assert!(r.stdout.is_empty());
    }
    fs::remove_file(path).unwrap();
}

#[test]
fn cli_tangential_contact_moves_vehicle_reports_passivity_and_rejects_bad_laws() {
    let fixture = fixture();
    let path = fixture.0.join("friction.json");
    let mut config = normal_configuration();
    config["support_plane"]["friction_scale"] = 1.into();
    for (i, u) in config["uprights"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .enumerate()
    {
        u["wheel"]["tangential_contact"] = serde_json::json!({"longitudinal_viscosity_n_s_m":3000,"lateral_viscosity_n_s_m":4000,"longitudinal_friction":0.9,"lateral_friction":0.9});
        u["wheel"]["drive_torque_nm"] = if i % 2 == 0 { 50 } else { -50 }.into();
    }
    let invoke = |c: &serde_json::Value| {
        fs::write(&path, c.to_string()).unwrap();
        Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap()
    };
    let output = invoke(&config);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["tangential_contact_wheels"], 4);
    let records = report["records"].as_array().unwrap();
    assert!(
        records.last().unwrap()["sample"]["linear_momentum_world_kg_m_s"][2]
            .as_f64()
            .unwrap()
            > 10.
    );
    for r in records {
        let sum = r["tangential_contacts"]
            .as_array()
            .unwrap()
            .iter()
            .map(|t| {
                let d = t["dissipated_power_w"].as_f64().unwrap();
                assert!(d >= 0.);
                d
            })
            .sum::<f64>();
        assert!((sum - r["tangential_dissipated_power_w"].as_f64().unwrap()).abs() < 1e-8);
        assert!(r["total_dissipated_power_w"].as_f64().unwrap() >= sum);
    }
    for case in 0..5 {
        let mut c = config.clone();
        match case {
            0 => {
                c["support_plane"]
                    .as_object_mut()
                    .unwrap()
                    .remove("friction_scale");
            }
            1 => c["support_plane"]["friction_scale"] = 11.into(),
            2 => {
                c["uprights"][0]["wheel"]["tangential_contact"]["longitudinal_viscosity_n_s_m"] =
                    (-1).into()
            }
            3 => c["uprights"][0]["wheel"]["tangential_contact"]["unknown"] = true.into(),
            _ => {
                c["uprights"][0]["wheel"]
                    .as_object_mut()
                    .unwrap()
                    .remove("normal_contact");
            }
        }
        let r = invoke(&c);
        assert!(!r.status.success());
        assert!(r.stdout.is_empty());
    }
    fs::remove_file(path).unwrap();
}

#[test]
fn cli_track_faces_match_plane_and_spawn_placement_retains_diagnostics() {
    use asc_test_support as f;
    let fixture = fixture();
    let road = f::named(
        f::mesh(
            &[
                [-50., 0., -50.],
                [50., 0., -50.],
                [-50., 0., 50.],
                [50., 0., 50.],
            ],
            &[0, 2, 1, 1, 2, 3],
            2,
            0,
            0,
            0.,
            0,
        ),
        "1ROAD",
    );
    let marker = f::named(
        f::base(&[], glam::Mat4::IDENTITY.to_cols_array(), 1),
        "AC_PIT_0",
    );
    fs::write(
        fixture.0.join("a.kn5"),
        f::file(
            f::base(&[road, marker], glam::Mat4::IDENTITY.to_cols_array(), 1),
            6,
        ),
    )
    .unwrap();
    fs::write(
        fixture.0.join("models.ini"),
        "[MODEL_0]\nFILE=a.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    fs::write(
        fixture.0.join("surfaces.ini"),
        "[SURFACE_0]\nKEY=ROAD\nFRICTION=.9\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n",
    )
    .unwrap();
    let path = fixture.0.join("track.json");
    let invoke = |c: &serde_json::Value| {
        fs::write(&path, c.to_string()).unwrap();
        Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap()
    };
    let plane = normal_configuration();
    let plane_output = invoke(&plane);
    assert!(plane_output.status.success());
    let expected: serde_json::Value = serde_json::from_slice(&plane_output.stdout).unwrap();
    let mut track = plane.clone();
    track.as_object_mut().unwrap().remove("support_plane");
    track["support_track"] = serde_json::json!({"models_ini":"models.ini","surface_files":["surfaces.ini"],"min_normal_y":0.5});
    track["initial_spawn"] =
        serde_json::json!({"name":"AC_PIT_0","chassis_com_offset_in_spawn_m":[0,0.29,0]});
    track["chassis"]["position_world_m"] = serde_json::json!([0, 0, 0]);
    let output = invoke(&track);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let actual: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(actual["normal_contact_uses_track_geometry"], true);
    assert_eq!(actual["edge_vertex_response"], false);
    assert_eq!(actual["continuous_collision_detection"], false);
    for (a, b) in actual["records"]
        .as_array()
        .unwrap()
        .iter()
        .zip(expected["records"].as_array().unwrap())
    {
        assert_eq!(a["normal_contacts"], b["normal_contacts"]);
        assert_eq!(a["sample"], b["sample"]);
        for face in a["track_contact_faces"].as_array().unwrap() {
            assert_eq!(face["mesh"], "1ROAD");
            assert_eq!(face["surface_key"], "ROAD");
        }
    }
    for modification in 0..6 {
        let mut c = track.clone();
        match modification {
            0 => c["support_plane"] = plane["support_plane"].clone(),
            1 => c["support_track"]["min_normal_y"] = 0.into(),
            2 => c["support_track"]["surface_files"] = serde_json::json!([]),
            3 => c["initial_spawn"]["name"] = "AC_PIT_99".into(),
            4 => c["chassis"]["position_world_m"] = serde_json::json!([0, 1, 0]),
            _ => {
                c["initial_spawn"]["chassis_com_offset_in_spawn_m"] = serde_json::json!([0, 101, 0])
            }
        }
        let r = invoke(&c);
        assert!(!r.status.success(), "case {modification}");
        assert!(r.stdout.is_empty());
    }
    for name in ["a.kn5", "models.ini", "surfaces.ini", "track.json"] {
        fs::remove_file(fixture.0.join(name)).unwrap();
    }
}
#[test]
fn cli_recomputes_plane_support_and_records_contact_energy_with_gravity() {
    let fixture = fixture();
    let path = fixture.0.join("normal.json");
    fs::write(&path, normal_configuration().to_string()).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
        .arg("chassis-run")
        .arg(&path)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["normal_contact_wheels"], 4);
    assert_eq!(report["normal_contact_uses_track_geometry"], false);
    assert_eq!(report["simulation_ready"], false);
    let records = report["records"].as_array().unwrap();
    assert_eq!(records.len(), 11);
    let initial = records[0]["mechanical_energy_with_normal_contact_j"]
        .as_f64()
        .unwrap();
    let last = records.last().unwrap()["mechanical_energy_with_normal_contact_j"]
        .as_f64()
        .unwrap();
    assert!(last < initial);
    for record in records {
        let contacts = record["normal_contacts"].as_array().unwrap();
        assert_eq!(contacts.len(), 4);
        let potential = contacts
            .iter()
            .map(|s| s["potential_energy_j"].as_f64().unwrap())
            .sum::<f64>();
        assert!(
            (potential
                - record["normal_contact_potential_energy_j"]
                    .as_f64()
                    .unwrap())
            .abs()
                < 1e-8
        );
        let force = contacts
            .iter()
            .map(|s| s["normal_force_n"].as_f64().unwrap())
            .sum::<f64>();
        assert!(force > 0.);
        for contact in contacts {
            assert!(contact["normal_force_n"].as_f64().unwrap() >= 0.);
            assert!(contact["dissipated_power_w"].as_f64().unwrap() >= 0.);
        }
        assert!(
            record["total_dissipated_power_w"].as_f64().unwrap()
                >= record["sample"]["dissipated_power_w"].as_f64().unwrap()
        );
    }
}
#[test]
fn cli_rejects_missing_plane_bad_normal_law_and_intermediate_penetration_without_json() {
    let fixture = fixture();
    let path = fixture.0.join("normal.json");
    let template = normal_configuration();
    let mut cases = Vec::new();
    let mut missing = template.clone();
    missing.as_object_mut().unwrap().remove("support_plane");
    cases.push(missing);
    let mut unused = template.clone();
    for u in unused["uprights"].as_array_mut().unwrap() {
        u["wheel"].as_object_mut().unwrap().remove("normal_contact");
    }
    cases.push(unused);
    let mut normal = template.clone();
    normal["support_plane"]["normal_world"] = serde_json::json!([0, 0, 0]);
    cases.push(normal);
    for (key, value) in [
        ("radius_m", 0.into()),
        ("stiffness_n_m", (-1).into()),
        ("max_penetration_m", 0.2.into()),
        ("unknown", true.into()),
    ] {
        let mut c = template.clone();
        c["uprights"][0]["wheel"]["normal_contact"][key] = value;
        cases.push(c);
    }
    let mut deep = template.clone();
    deep["chassis"]["velocity_world_m_s"] = serde_json::json!([0, -100, 0]);
    deep["steps"] = 1.into();
    deep["record_every"] = 1.into();
    deep["step_s"] = 0.01.into();
    cases.push(deep);
    for c in cases {
        fs::write(&path, c.to_string()).unwrap();
        let output = Command::new(env!("CARGO_BIN_EXE_openasc"))
            .arg("chassis-run")
            .arg(&path)
            .output()
            .unwrap();
        assert!(!output.status.success());
        assert!(output.stdout.is_empty());
    }
}
