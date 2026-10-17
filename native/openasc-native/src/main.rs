use openasc_native::{kn5, render};
use std::{error::Error, path::Path, process::ExitCode};

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let usage = "Usage: openasc-native inspect MODEL.kn5 | render MODEL.kn5 NEW.png [--fallback]";
    let command = args.first().and_then(|v| v.to_str()).ok_or(usage)?;
    match command {
        "inspect" if args.len() == 2 => {
            let scene = kn5::load(Path::new(&args[1]))?;
            println!("{}", serde_json::to_string_pretty(&scene.summary)?);
        }
        "render" if args.len() == 3 || (args.len() == 4 && args[3] == "--fallback") => {
            let output = Path::new(&args[2]);
            if output.exists() {
                return Err("Output already exists; choose a new PNG path.".into());
            }
            let scene = kn5::load(Path::new(&args[1]))?;
            let frame = pollster::block_on(render::render(&scene, 1240, 900, args.len() == 4))?;
            if frame.report.non_background_pixels == 0 {
                return Err("GPU produced no non-background geometry pixels.".into());
            }
            render::save_png(&frame, output)?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"scene": scene.summary, "render": frame.report})
                )?
            );
        }
        _ => return Err(usage.into()),
    }
    Ok(())
}
fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("openasc-native: {error}");
            ExitCode::FAILURE
        }
    }
}
