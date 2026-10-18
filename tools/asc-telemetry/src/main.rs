use std::{error::Error, ffi::OsString, path::PathBuf, process::ExitCode};

const USAGE: &str = "Usage: asc-telemetry capture NEW.jsonl --expect-ac-version VERSION --expect-sm-version VERSION [--samples 1000] [--interval-ms 10]. Windows only; reads existing original-AC shared memory. See docs/TELEMETRY.md.";
struct Options {
    output: PathBuf,
    ac_version: String,
    sm_version: String,
    samples: u32,
    interval_ms: u64,
}
fn options(args: &[OsString]) -> Result<Options, Box<dyn Error>> {
    if args.len() < 2 || args[0] != "capture" {
        return Err(USAGE.into());
    }
    let mut result = Options {
        output: PathBuf::from(&args[1]),
        ac_version: String::new(),
        sm_version: String::new(),
        samples: 1000,
        interval_ms: 10,
    };
    let mut flags = args[2..].chunks_exact(2);
    let mut seen = std::collections::HashSet::new();
    for pair in &mut flags {
        let key = pair[0].to_str().ok_or(USAGE)?;
        let value = pair[1].to_str().ok_or(USAGE)?;
        if !seen.insert(key) {
            return Err("Duplicate capture option".into());
        }
        match key {
            "--expect-ac-version" => result.ac_version = value.into(),
            "--expect-sm-version" => result.sm_version = value.into(),
            "--samples" => result.samples = value.parse()?,
            "--interval-ms" => result.interval_ms = value.parse()?,
            _ => return Err(USAGE.into()),
        }
    }
    let valid_version = |s: &str| {
        s.len() <= 14
            && s.starts_with("1.")
            && s.split('.')
                .all(|part| !part.is_empty() && part.bytes().all(|b| b.is_ascii_digit()))
    };
    if !flags.remainder().is_empty()
        || !valid_version(&result.ac_version)
        || !valid_version(&result.sm_version)
    {
        return Err("Specify explicit original-AC 1.x and shared-memory 1.x versions; unknown versions are not inferred".into());
    }
    if !(2..=6000).contains(&result.samples)
        || !(1..=1000).contains(&result.interval_ms)
        || u64::from(result.samples) * result.interval_ms > 60_000
    {
        return Err("Use 2..6000 samples, 1..1000 ms interval and at most 60000 ms total".into());
    }
    Ok(result)
}
#[cfg(windows)]
fn capture(options: Options) -> Result<(), Box<dyn Error>> {
    use asc_telemetry::{journal::Journal, windows::Connection};
    use std::{
        fs::OpenOptions,
        time::{Duration, Instant},
    };
    if options.output.exists() {
        return Err("Output already exists; choose a new JSONL path".into());
    }
    let connection = Connection::open()?;
    let started = Instant::now();
    let initial = connection.snapshot()?;
    let initial_received = started.elapsed();
    if initial.identity.game_version != options.ac_version
        || initial.identity.shared_memory_version != options.sm_version
    {
        return Err(format!("Version mismatch: expected AC {} / SM {}, received AC {} / SM {}. Output was not created.",
            options.ac_version, options.sm_version, initial.identity.game_version, initial.identity.shared_memory_version).into());
    }
    asc_telemetry::validate_live(&initial, &initial.identity)?;
    // Creation happens only after successful attach and initial validation.
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&options.output)?;
    let mut journal = Journal::new(file, &initial, options.samples, options.interval_ms)?;
    let interval = Duration::from_millis(options.interval_ms);
    journal.sample(&initial, u64::try_from(initial_received.as_nanos())?)?;
    for _ in 1..options.samples {
        // Do not emit a catch-up burst after a delayed wake. Actual receive time
        // is recorded; Windows scheduling is not a simulation clock.
        std::thread::sleep(interval);
        let snapshot = connection.snapshot()?;
        let elapsed = started.elapsed();
        if elapsed > Duration::from_secs(60) {
            return Err("Capture exceeded 60 seconds; output is incomplete".into());
        }
        journal.sample(&snapshot, u64::try_from(elapsed.as_nanos())?)?;
    }
    let summary = journal.finish()?;
    println!("{}", serde_json::to_string_pretty(&summary)?);
    Ok(())
}
#[cfg(not(windows))]
fn capture(options: Options) -> Result<(), Box<dyn Error>> {
    let _ = (
        options.output,
        options.ac_version,
        options.sm_version,
        options.samples,
        options.interval_ms,
    );
    Err(
        "Original-AC named shared-memory acquisition requires Windows. No output was created."
            .into(),
    )
}
fn main() -> ExitCode {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args == [OsString::from("--help")] {
        println!("{USAGE}");
        return ExitCode::SUCCESS;
    }
    match options(&args).and_then(capture) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!(
                "Telemetry capture failed: {error}. Any created output without a complete footer is incomplete."
            );
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn args(extra: &[&str]) -> Vec<OsString> {
        [
            "capture",
            "new.jsonl",
            "--expect-ac-version",
            "1.16.4",
            "--expect-sm-version",
            "1.7",
        ]
        .into_iter()
        .chain(extra.iter().copied())
        .map(OsString::from)
        .collect()
    }
    #[test]
    fn explicit_versions_bounded_duration_and_unambiguous_flags() {
        assert_eq!(options(&args(&[])).unwrap().samples, 1000);
        for extra in [
            &["--samples", "0"][..],
            &["--samples", "6001"],
            &["--interval-ms", "0"],
            &["--interval-ms", "1000"],
            &["--expect-ac-version", "2.0"],
            &["--unknown", "3"],
            &["--samples"],
        ] {
            assert!(options(&args(extra)).is_err(), "{extra:?}");
        }
        assert!(options(&["capture".into(), "new.jsonl".into()]).is_err());
    }
}
