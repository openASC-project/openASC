use asc_kn5 as kn5;
use asc_render as render;
use std::{error::Error, path::Path, process::ExitCode};
mod app;
use app::viewer;

struct Input {
    scene: kn5::Scene,
    resources: Option<render::Resources>,
    manifest: Option<asc_scene::Manifest>,
}
fn load_input(path: &Path, diagnostic: bool) -> Result<Input, Box<dyn Error>> {
    if path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("ini"))
    {
        let track = asc_scene::load(path)?;
        let resources = (!diagnostic).then(|| render::prepare_track_textures(&track));
        Ok(Input {
            scene: track.scene,
            resources,
            manifest: Some(track.manifest),
        })
    } else {
        let scene = kn5::load(path)?;
        let resources = (!diagnostic).then(|| render::prepare_textures(&scene, path));
        Ok(Input {
            scene,
            resources,
            manifest: None,
        })
    }
}

fn run() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let usage = "Usage: openasc inspect SCENE | render SCENE NEW.png [--fallback] [--diagnostic] [--rgba-textures] | view SCENE [--fallback] [--diagnostic] [--rgba-textures] [--captures DIR] [--capture NEW.png] [--smoke] | ini FILE.ini | lut FILE.lut [--sample X]. SCENE is a KN5 file or an explicit models.ini/models_LAYOUT.ini path.";
    let command = args.first().and_then(|v| v.to_str()).ok_or(usage)?;
    match command {
        "ini" if args.len() == 2 => {
            println!(
                "{}",
                serde_json::to_string_pretty(&asc_config::read_ini(Path::new(&args[1]))?)?
            );
        }
        "lut" if args.len() == 2 || (args.len() == 4 && args[2] == "--sample") => {
            let table = asc_config::read_lut(Path::new(&args[1]))?;
            let sample = if args.len() == 4 {
                let x =
                    asc_config::finite_number(args[3].to_str().ok_or(usage)?, &table.source, 0)?;
                Some(serde_json::json!({"x":x,"y":table.sample(x)?}))
            } else {
                None
            };
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"source":table.source,"points":table.points(),"sample":sample})
                )?
            );
        }
        "view" if args.len() >= 2 => {
            let (mut fallback, mut diagnostic, mut smoke, mut captures) =
                (false, false, false, None);
            let mut flags = args[2..].iter();
            let mut first_capture = None;
            let mut rgba_textures = false;
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--fallback") => fallback = true,
                    Some("--diagnostic") => diagnostic = true,
                    Some("--rgba-textures") => rgba_textures = true,
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
            let Input {
                scene,
                mut resources,
                manifest,
            } = load_input(Path::new(&args[1]), diagnostic)?;
            if let Some(resources) = resources.as_mut() {
                resources.allow_bc = !rgba_textures;
            }
            if let Some(manifest) = manifest {
                println!(
                    "{}",
                    serde_json::to_string(&serde_json::json!({"manifest":manifest}))?
                );
            }
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
            let input = load_input(Path::new(&args[1]), true)?;
            if let Some(manifest) = input.manifest {
                println!(
                    "{}",
                    serde_json::to_string_pretty(
                        &serde_json::json!({"manifest":manifest,"scene":input.scene.summary})
                    )?
                );
            } else {
                println!("{}", serde_json::to_string_pretty(&input.scene.summary)?);
            }
        }
        "render"
            if (3..=6).contains(&args.len())
                && args[3..].iter().all(|v| {
                    v == "--fallback" || v == "--diagnostic" || v == "--rgba-textures"
                }) =>
        {
            let output = Path::new(&args[2]);
            if output.exists() {
                return Err("Output already exists; choose a new PNG path.".into());
            }
            let Input {
                scene,
                mut resources,
                manifest,
            } = load_input(
                Path::new(&args[1]),
                args[3..].iter().any(|v| v == "--diagnostic"),
            )?;
            if let Some(resources) = resources.as_mut() {
                resources.allow_bc = !args[3..].iter().any(|v| v == "--rgba-textures");
            }
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
                    &serde_json::json!({"manifest":manifest,"scene": scene.summary, "render": frame.report, "texture_resources": resources.as_ref().map(|r| r.textures.iter().map(|t| &t.info).collect::<Vec<_>>())})
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
