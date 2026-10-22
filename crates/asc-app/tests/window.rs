use asc_test_support as fixture;
use glam::Mat4;
use std::process::Command;

#[test]
#[ignore = "Requires desktop/GPU; Linux CI executes under Xvfb."]
fn native_drive_window_moves_the_gpu_root_from_coupled_keyboard_commands_and_brakes() {
    let root = fixture::temp_path("native-drive-window");
    let car = root.join("fixture");
    std::fs::create_dir_all(car.join("data")).unwrap();
    for (name, text) in fixture::vehicle_files() {
        std::fs::write(car.join("data").join(name), text).unwrap();
    }
    std::fs::write(
        car.join("data/suspensions.ini"),
        fixture::suspension_motion_file(),
    )
    .unwrap();
    let body = fixture::named(
        fixture::mesh(
            &[
                [-0.5, 0.3, -1.],
                [0.5, 0.3, -1.],
                [-0.5, 0.3, 1.],
                [0.5, 0.3, 1.],
            ],
            &[0, 2, 1, 1, 2, 3],
            2,
            1,
            1,
            0.,
            0,
        ),
        "original_body",
    );
    std::fs::write(
        car.join("original.kn5"),
        fixture::file_texture(
            fixture::base(&[body], Mat4::IDENTITY.to_cols_array(), 1),
            6,
            &fixture::bc1(0xf800),
        ),
    )
    .unwrap();
    let road = fixture::named(
        fixture::mesh(
            &[
                [-20., 0., -20.],
                [20., 0., -20.],
                [-20., 0., 20.],
                [20., 0., 20.],
            ],
            &[0, 2, 1, 1, 2, 3],
            2,
            1,
            1,
            0.,
            0,
        ),
        "1ROAD",
    );
    let marker = fixture::named(
        fixture::base(
            &[],
            Mat4::from_translation(glam::Vec3::new(0., 0.554, 0.)).to_cols_array(),
            1,
        ),
        "AC_PIT_0",
    );
    std::fs::write(
        root.join("track.kn5"),
        fixture::file_texture(
            fixture::base(&[road, marker], Mat4::IDENTITY.to_cols_array(), 1),
            6,
            &fixture::bc1(0x001f),
        ),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=track.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    std::fs::write(
        root.join("surfaces.ini"),
        "[SURFACE_0]\nKEY=ROAD\nFRICTION=1\nDAMPING=0\nIS_VALID_TRACK=1\nIS_PITLANE=0\n",
    )
    .unwrap();
    let mut dynamics: serde_json::Value = serde_json::from_str(include_str!(
        "../../../examples/chassis-steering-track.json"
    ))
    .unwrap();
    dynamics["support_track"] = serde_json::json!({"models_ini":"models.ini","surface_files":["surfaces.ini"],"min_normal_y":0.5});
    for u in dynamics["uprights"].as_array_mut().unwrap() {
        u["car_directory"] = "fixture".into();
    }
    std::fs::write(root.join("dynamics.json"), dynamics.to_string()).unwrap();
    let mut drive: serde_json::Value =
        serde_json::from_str(include_str!("../../../examples/native-drive.json")).unwrap();
    drive["dynamics_config"] = "dynamics.json".into();
    std::fs::write(root.join("drive.json"), drive.to_string()).unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_openasc"));
    command
        .arg("drive")
        .arg(root.join("drive.json"))
        .arg("--smoke");
    if std::env::var_os("OPENASC_TEST_FALLBACK").is_some() {
        command.arg("--fallback");
    }
    let output = command.output().unwrap();
    let _ = std::fs::remove_dir_all(&root);
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(
        output.status.success(),
        "{stdout}\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find(|l| l.starts_with("{\"native_drive_smoke\":"))
            .unwrap(),
    )
    .unwrap();
    let r = &report["native_drive_smoke"];
    assert_eq!(r["root_instances"], 2);
    let rows = r["records"].as_array().unwrap();
    assert_eq!(rows.len(), 4);
    let speed = |r: &serde_json::Value| {
        let v = r["body"]["velocity_world_m_s"].as_array().unwrap();
        (v[0].as_f64().unwrap().powi(2) + v[2].as_f64().unwrap().powi(2)).sqrt()
    };
    assert!(speed(&rows[2]) > 0.2);
    assert!(speed(&rows[3]) < 0.5 * speed(&rows[2]));
    assert!(
        rows[2]["body"]["angular_velocity_world_rad_s"][1]
            .as_f64()
            .unwrap()
            .abs()
            > 1e-4
    );
    assert_ne!(rows[0]["pixel_hash"], rows[3]["pixel_hash"]);
}

#[test]
#[ignore = "Requires a desktop display and GPU/software Vulkan; Linux CI uses Xvfb."]
fn native_window_presents_resizes_and_reads_back_changed_cameras() {
    run_window(false, TextureMode::Diffuse);
}

#[test]
#[ignore = "Requires a desktop display and GPU/software Vulkan; Linux CI uses Xvfb."]
fn native_window_presents_a_composed_models_ini_scene() {
    run_window(true, TextureMode::Diffuse);
}

#[test]
#[ignore = "Requires a desktop display and GPU/software Vulkan; Linux CI uses Xvfb."]
fn native_window_can_force_rgba_decode_of_retained_bc_sources() {
    run_window(false, TextureMode::Rgba);
}

enum TextureMode {
    Diffuse,
    Rgba,
    Layers,
}
#[test]
#[ignore = "Requires a desktop display and GPU/software Vulkan; Linux CI uses Xvfb."]
fn native_window_presents_and_toggles_multilayer_base_colour() {
    run_window(false, TextureMode::Layers);
}

#[test]
#[ignore = "Requires desktop/GPU; Linux CI executes under Xvfb."]
fn native_session_window_presents_independent_car_and_track_roots() {
    let root = fixture::temp_path("preview-window");
    let car = root.join("fixture");
    std::fs::create_dir_all(car.join("data")).unwrap();
    for (name, text) in fixture::vehicle_files() {
        std::fs::write(car.join("data").join(name), text).unwrap();
    }
    let mesh = fixture::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    std::fs::write(
        car.join("original.kn5"),
        fixture::file_texture(
            fixture::base(
                std::slice::from_ref(&mesh),
                Mat4::IDENTITY.to_cols_array(),
                1,
            ),
            6,
            &fixture::bc1(0xf800),
        ),
    )
    .unwrap();
    let marker = fixture::named(
        fixture::base(
            &[],
            Mat4::from_translation(glam::Vec3::new(0., 0.5, 0.)).to_cols_array(),
            1,
        ),
        "AC_PIT_0",
    );
    std::fs::write(
        root.join("track.kn5"),
        fixture::file_texture(
            fixture::base(&[mesh, marker], Mat4::IDENTITY.to_cols_array(), 1),
            6,
            &fixture::bc1(0x001f),
        ),
    )
    .unwrap();
    std::fs::write(
        root.join("models.ini"),
        "[MODEL_0]\nFILE=track.kn5\nPOSITION=0,0,0\nROTATION=0,0,0\n",
    )
    .unwrap();
    let mut command = Command::new(env!("CARGO_BIN_EXE_openasc"));
    command
        .arg("preview")
        .arg(&car)
        .arg(root.join("models.ini"))
        .arg("--smoke");
    if std::env::var_os("OPENASC_TEST_FALLBACK").is_some() {
        command.arg("--fallback");
    }
    let output = command.output().unwrap();
    let _ = std::fs::remove_dir_all(&root);
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(
        output.status.success(),
        "{stdout}\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let native: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find(|v| v.starts_with("{\"native_window\":"))
            .unwrap(),
    )
    .unwrap();
    assert_eq!(native["native_window"]["root_instances"], 2);
    let metadata: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find(|v| v.starts_with("{\"session_preview\":"))
            .unwrap(),
    )
    .unwrap();
    assert_eq!(metadata["session_preview"]["simulation_ready"], false);
    let smoke: serde_json::Value = serde_json::from_str(
        stdout
            .lines()
            .find(|v| v.starts_with("{\"window_smoke\":"))
            .unwrap(),
    )
    .unwrap();
    assert_eq!(smoke["window_smoke"]["presented_frames"], 5);
    assert_eq!(smoke["window_smoke"]["toggle_changed_pixels"], true);
}
fn run_window(composed: bool, mode: TextureMode) {
    let rgba = matches!(mode, TextureMode::Rgba);
    let layered = matches!(mode, TextureMode::Layers);
    let path = fixture::temp_path("window").with_extension("kn5");
    let mesh = fixture::mesh(
        &[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]],
        &[0, 2, 1],
        2,
        1,
        1,
        0.,
        0,
    );
    let mut header = [0u32; 31];
    header[0] = 124;
    header[1] = 0x1007;
    header[2] = 4;
    header[3] = 4;
    header[6] = 1;
    header[18] = 32;
    header[19] = 4;
    header[20] = u32::from_le_bytes(*b"DXT1");
    let mut dds = b"DDS ".to_vec();
    for v in header {
        dds.extend(v.to_le_bytes());
    }
    dds.extend([0, 248, 0, 0, 0, 0, 0, 0]);
    let node = fixture::base(&[mesh], Mat4::IDENTITY.to_cols_array(), 1);
    std::fs::write(
        &path,
        if layered {
            fixture::multilayer_file(node)
        } else {
            fixture::file_texture(node, 6, &dds)
        },
    )
    .unwrap();
    let config = path.with_extension("ini");
    if composed {
        let name = path.file_name().unwrap().to_str().unwrap();
        std::fs::write(&config, format!("[MODEL_0]\nFILE={name}\nPOSITION=0,0,0\nROTATION=0,0,0\n[MODEL_1]\nFILE={name}\nPOSITION=2,0,0\nROTATION=0,0,0\n")).unwrap();
    }
    let mut command = Command::new(env!("CARGO_BIN_EXE_openasc"));
    let capture = path.with_extension("png");
    command
        .arg("view")
        .arg(if composed { &config } else { &path })
        .arg("--smoke");
    command.arg("--capture").arg(&capture);
    if rgba {
        command.arg("--rgba-textures");
    }
    if std::env::var_os("OPENASC_TEST_FALLBACK").is_some() {
        command.arg("--fallback");
    }
    let output = command.output().unwrap();
    let _ = std::fs::remove_file(&path);
    if composed {
        let _ = std::fs::remove_file(&config);
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "Native window failed: {stdout}\n{stderr}"
    );
    if layered {
        let native: serde_json::Value = serde_json::from_str(
            stdout
                .lines()
                .find(|l| l.starts_with("{\"native_window\":"))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(native["native_window"]["scene"]["multilayer_draws"], 1);
    }
    if rgba {
        let native: serde_json::Value = serde_json::from_str(
            stdout
                .lines()
                .find(|l| l.starts_with("{\"native_window\":"))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(native["native_window"]["scene"]["bc_enabled"], false);
        assert_eq!(native["native_window"]["scene"]["compressed_textures"], 0);
        assert_eq!(native["native_window"]["scene"]["texture_upload_bytes"], 64);
    }
    if composed {
        let manifest: serde_json::Value = serde_json::from_str(
            stdout
                .lines()
                .find(|l| l.starts_with("{\"manifest\":"))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(manifest["manifest"]["models"].as_array().unwrap().len(), 2);
        let native: serde_json::Value = serde_json::from_str(
            stdout
                .lines()
                .find(|l| l.starts_with("{\"native_window\":"))
                .unwrap(),
        )
        .unwrap();
        assert_eq!(native["native_window"]["scene"]["triangles"], 2);
        assert_eq!(native["native_window"]["scene"]["textured_draws"], 2);
    }
    let png = std::fs::read(&capture).unwrap();
    std::fs::remove_file(&capture).unwrap();
    assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
    assert_eq!(u32::from_be_bytes(png[16..20].try_into().unwrap()), 319);
    assert_eq!(u32::from_be_bytes(png[20..24].try_into().unwrap()), 241);
    let mut reader = png::Decoder::new(std::io::Cursor::new(png))
        .read_info()
        .unwrap();
    let mut pixels = vec![0; reader.output_buffer_size().unwrap()];
    reader.next_frame(&mut pixels).unwrap();
    assert!(
        pixels[..4]
            .iter()
            .zip([29, 35, 44, 255])
            .all(|(&v, b)| v.abs_diff(b) <= 1),
        "Presented BGRA readback must become canonical RGBA: {:?}",
        &pixels[..4]
    );
    let line = stdout
        .lines()
        .find(|l| l.starts_with("{\"window_smoke\":"))
        .expect("Window smoke report missing");
    let report: serde_json::Value = serde_json::from_str(line).unwrap();
    let report = &report["window_smoke"];
    assert_eq!(report["presented_frames"], 5);
    assert_eq!(report["size"], serde_json::json!([640, 480]));
    assert!(report["configurations"].as_u64().unwrap() >= 2);
    assert_eq!(report["camera_changed"], true);
    assert_eq!(report["readback_frames"], 4);
    assert!(
        report["non_background_pixels"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| v.as_u64().unwrap() > 0)
    );
    assert_eq!(
        report["diffuse_modes"],
        serde_json::json!([true, true, false, true])
    );
    assert_eq!(report["toggle_changed_pixels"], true);
    println!("{report}");
}
