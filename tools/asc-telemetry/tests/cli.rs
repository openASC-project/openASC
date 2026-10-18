use std::{fs, process::Command};

#[test]
fn help_and_invalid_arguments_do_not_create_output() {
    let binary = env!("CARGO_BIN_EXE_asc-telemetry");
    assert!(
        Command::new(binary)
            .arg("--help")
            .output()
            .unwrap()
            .status
            .success()
    );
    let path = asc_test_support::temp_path("telemetry-invalid.jsonl");
    let output = Command::new(binary)
        .arg("capture")
        .arg(&path)
        .args([
            "--expect-ac-version",
            "1.16.4",
            "--expect-sm-version",
            "1.7",
            "--samples",
            "6001",
        ])
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(!path.exists());
}

#[test]
fn an_existing_output_is_never_replaced() {
    let path = asc_test_support::temp_path("telemetry-existing.jsonl");
    fs::write(&path, b"original fixture bytes").unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_asc-telemetry"))
        .arg("capture")
        .arg(&path)
        .args([
            "--expect-ac-version",
            "1.16.4",
            "--expect-sm-version",
            "1.7",
            "--samples",
            "2",
        ])
        .output()
        .unwrap();
    assert!(!result.status.success());
    assert_eq!(fs::read(&path).unwrap(), b"original fixture bytes");
    fs::remove_file(path).unwrap();
}
