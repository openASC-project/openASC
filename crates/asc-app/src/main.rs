use asc_kn5 as kn5;
use asc_render as render;
use std::{error::Error, path::Path, process::ExitCode};
mod app;
use app::viewer;

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let usage = "Usage: openasc inspect MODEL.kn5 | render MODEL.kn5 NEW.png [--fallback] [--diagnostic] | view MODEL.kn5 [--fallback] [--diagnostic] [--captures DIR] [--capture NEW.png] [--smoke]";
    let command = args.first().and_then(|v| v.to_str()).ok_or(usage)?;
    match command {
        "view" if args.len() >= 2 => {
            let (mut fallback, mut diagnostic, mut smoke, mut captures) =
                (false, false, false, None);
            let mut flags = args[2..].iter();
            let mut first_capture = None;
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--fallback") => fallback = true,
                    Some("--diagnostic") => diagnostic = true,
                    Some("--smoke") => smoke = true,
                    Some("--captures") => {
                        captures = Some(std::path::PathBuf::from(flags.next().ok_or(usage)?))
                    }
                    Some("--capture") => {
                        let path = std::path::PathBuf::from(flags.next().ok_or(usage)?);
                        if path.exists() {
                            return Err("Output already exists; choose a new PNG path.".into());
                        }
                        first_capture = Some(path);
                    }
                    _ => return Err(usage.into()),
                }
            }
            let scene = kn5::load(Path::new(&args[1]))?;
            let resources = if diagnostic {
                None
            } else {
                Some(render::load_textures(&scene, Path::new(&args[1])))
            };
            println!("{}", serde_json::to_string(&scene.summary)?);
            viewer::run(
                scene,
                resources,
                viewer::Options {
                    fallback,
                    smoke,
                    captures,
                    first_capture,
                },
            )?;
        }
        "inspect" if args.len() == 2 => {
            let scene = kn5::load(Path::new(&args[1]))?;
            println!("{}", serde_json::to_string_pretty(&scene.summary)?);
        }
        "render"
            if (3..=5).contains(&args.len())
                && args[3..]
                    .iter()
                    .all(|v| v == "--fallback" || v == "--diagnostic") =>
        {
            let output = Path::new(&args[2]);
            if output.exists() {
                return Err("Output already exists; choose a new PNG path.".into());
            }
            let scene = kn5::load(Path::new(&args[1]))?;
            let resources = if args[3..].iter().any(|v| v == "--diagnostic") {
                None
            } else {
                Some(render::load_textures(&scene, Path::new(&args[1])))
            };
            let frame = pollster::block_on(render::render_with_textures(
                &scene,
                resources.as_ref(),
                1240,
                900,
                args[3..].iter().any(|v| v == "--fallback"),
            ))?;
            if frame.report.non_background_pixels == 0 {
                return Err("GPU produced no non-background geometry pixels.".into());
            }
            render::save_png(&frame, output)?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"scene": scene.summary, "render": frame.report, "texture_resources": resources.as_ref().map(|r| r.textures.iter().map(|t| &t.info).collect::<Vec<_>>())})
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
            eprintln!("openasc: {error}");
            ExitCode::FAILURE
        }
    }
}
