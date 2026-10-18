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
    let usage = "Usage: openasc suspension-motion CAR_DIRECTORY AXLE TRAVEL_M AXIS_X AXIS_Y AXIS_Z [--rack X Y Z] [--data auto|packed|unpacked] | preview CAR_DIRECTORY MODELS.ini [--spawn NAME] [--data auto|packed|unpacked] [--compound N] [--capture NEW.png | --smoke | --captures DIR] [--fallback] [--diagnostic] [--rgba-textures] | suspension CAR_DIRECTORY [--data auto|packed|unpacked] | spawns MODELS.ini | place CAR_DIRECTORY MODELS.ini SURFACES.ini [--defaults SURFACES.ini] [--spawn NAME] [--data auto|packed|unpacked] [--compound N] | ground MODELS.ini SURFACES.ini [--defaults SURFACES.ini] [--probe X Y Z]... [--distance D] | car CAR_DIRECTORY [--data auto|packed|unpacked] [--compound N] | acd FILE.acd CAR_ID | inspect SCENE | materials SCENE | render SCENE NEW.png [--fallback] [--diagnostic] [--rgba-textures] | view SCENE [--fallback] [--diagnostic] [--rgba-textures] [--captures DIR] [--capture NEW.png] [--smoke] | ini FILE.ini | lut FILE.lut [--sample X]. SCENE is a KN5 file or an explicit models.ini/models_LAYOUT.ini path.";
    let command = args.first().and_then(|v| v.to_str()).ok_or(usage)?;
    match command {
        "suspension-motion" if args.len() >= 7 => {
            let axle = args[2].to_str().ok_or(usage)?;
            if !["front", "rear"].contains(&axle) {
                return Err("Choose front or rear for suspension-motion".into());
            }
            let number = |value: &std::ffi::OsString| -> Result<f64, Box<dyn Error>> {
                Ok(value.to_str().ok_or(usage)?.parse()?)
            };
            let travel = number(&args[3])?;
            let axis = [number(&args[4])?, number(&args[5])?, number(&args[6])?];
            let mut mode = asc_vehicle::data::Mode::Auto;
            let mut rack = [0.; 3];
            let mut flags = args[7..].iter();
            let mut seen = std::collections::HashSet::new();
            while let Some(flag) = flags.next() {
                let flag = flag.to_str().ok_or(usage)?;
                if !seen.insert(flag) {
                    return Err("Duplicate suspension-motion option".into());
                }
                match flag {
                    "--rack" => {
                        rack = [
                            number(flags.next().ok_or(usage)?)?,
                            number(flags.next().ok_or(usage)?)?,
                            number(flags.next().ok_or(usage)?)?,
                        ];
                    }
                    "--data" => {
                        mode = match flags.next().and_then(|v| v.to_str()) {
                            Some("auto") => asc_vehicle::data::Mode::Auto,
                            Some("packed") => asc_vehicle::data::Mode::Packed,
                            Some("unpacked") => asc_vehicle::data::Mode::Unpacked,
                            _ => return Err(usage.into()),
                        };
                    }
                    _ => return Err(usage.into()),
                }
            }
            println!(
                "{}",
                serde_json::to_string_pretty(&app::suspension::motion(
                    Path::new(&args[1]),
                    axle,
                    travel,
                    axis,
                    rack,
                    mode
                )?)?
            );
        }
        "preview" if args.len() >= 3 => {
            let (mut fallback, mut diagnostic, mut rgba, mut smoke) = (false, false, false, false);
            let (mut capture, mut captures) = (None, None);
            let mut mode = asc_vehicle::data::Mode::Auto;
            let mut compound = None;
            let mut spawn = "AC_PIT_0";
            let mut flags = args[3..].iter();
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--fallback") => fallback = true,
                    Some("--diagnostic") => diagnostic = true,
                    Some("--rgba-textures") => rgba = true,
                    Some("--smoke") => smoke = true,
                    Some("--capture") => {
                        capture = Some(std::path::PathBuf::from(flags.next().ok_or(usage)?))
                    }
                    Some("--captures") => {
                        captures = Some(std::path::PathBuf::from(flags.next().ok_or(usage)?))
                    }
                    Some("--spawn") => {
                        spawn = flags.next().and_then(|v| v.to_str()).ok_or(usage)?
                    }
                    Some("--compound") => {
                        compound = Some(
                            flags
                                .next()
                                .and_then(|v| v.to_str())
                                .ok_or(usage)?
                                .parse::<u32>()?,
                        )
                    }
                    Some("--data") => {
                        mode = match flags.next().and_then(|v| v.to_str()) {
                            Some("auto") => asc_vehicle::data::Mode::Auto,
                            Some("packed") => asc_vehicle::data::Mode::Packed,
                            Some("unpacked") => asc_vehicle::data::Mode::Unpacked,
                            _ => return Err(usage.into()),
                        }
                    }
                    _ => return Err(usage.into()),
                }
            }
            if capture.is_some() && (smoke || captures.is_some()) {
                return Err(
                    "Offscreen --capture cannot be combined with window --smoke/--captures.".into(),
                );
            }
            if capture.as_ref().is_some_and(|p| p.exists()) {
                return Err("Output already exists; choose a new PNG path.".into());
            }
            let preview = app::session::load(
                Path::new(&args[1]),
                Path::new(&args[2]),
                spawn,
                mode,
                compound,
            )?;
            let mut resources =
                (!diagnostic).then(|| render::prepare_track_textures(&preview.track));
            if let Some(r) = &mut resources {
                r.allow_bc = !rgba;
            }
            if let Some(path) = capture {
                let camera = render::scene::orbit::Orbit::new(
                    preview.focus,
                    preview
                        .track
                        .scene
                        .summary
                        .bounds
                        .ok_or("No preview bounds.")?,
                )?;
                let frame = pollster::block_on(render::gpu::capture::render_with_view(
                    &preview.track.scene,
                    resources.as_ref(),
                    std::slice::from_ref(&preview.instance),
                    camera.view(1240, 900, !diagnostic)?,
                    (1240, 900),
                    fallback,
                ))?;
                if frame.report.non_background_pixels == 0 {
                    return Err("GPU produced no non-background session pixels.".into());
                }
                render::save_png(&frame, &path)?;
                println!(
                    "{}",
                    serde_json::to_string_pretty(
                        &serde_json::json!({"session_preview":preview.metadata,"render":frame.report})
                    )?
                );
            } else {
                println!(
                    "{}",
                    serde_json::json!({"session_preview":preview.metadata})
                );
                viewer::run_instanced(
                    preview.track.scene,
                    resources,
                    viewer::Options {
                        fallback,
                        smoke,
                        captures,
                        first_capture: None,
                    },
                    vec![preview.instance],
                    Some(preview.focus),
                )?;
            }
        }
        "suspension" if args.len() == 2 || (args.len() == 4 && args[2] == "--data") => {
            let mode = if args.len() == 2 {
                asc_vehicle::data::Mode::Auto
            } else {
                match args[3].to_str() {
                    Some("auto") => asc_vehicle::data::Mode::Auto,
                    Some("packed") => asc_vehicle::data::Mode::Packed,
                    Some("unpacked") => asc_vehicle::data::Mode::Unpacked,
                    _ => return Err(usage.into()),
                }
            };
            println!(
                "{}",
                serde_json::to_string_pretty(&asc_vehicle::suspension::load(
                    Path::new(&args[1]),
                    mode
                )?)?
            );
        }
        "spawns" if args.len() == 2 => {
            let report = asc_scene::spawns::load(Path::new(&args[1]))?;
            println!("{}", serde_json::to_string_pretty(&report)?);
            if report.has_errors {
                return Err("Spawn report contains errors; inspect its diagnostics.".into());
            }
        }
        "place" if args.len() >= 4 => {
            let mut defaults = None;
            let mut spawn = "AC_PIT_0";
            let mut mode = asc_vehicle::data::Mode::Auto;
            let mut compound = None;
            let mut flags = args[4..].iter();
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--defaults") => defaults = Some(Path::new(flags.next().ok_or(usage)?)),
                    Some("--spawn") => {
                        spawn = flags.next().and_then(|v| v.to_str()).ok_or(usage)?
                    }
                    Some("--compound") => {
                        compound = Some(
                            flags
                                .next()
                                .and_then(|v| v.to_str())
                                .ok_or(usage)?
                                .parse::<u32>()?,
                        )
                    }
                    Some("--data") => {
                        mode = match flags.next().and_then(|v| v.to_str()) {
                            Some("auto") => asc_vehicle::data::Mode::Auto,
                            Some("packed") => asc_vehicle::data::Mode::Packed,
                            Some("unpacked") => asc_vehicle::data::Mode::Unpacked,
                            _ => return Err(usage.into()),
                        }
                    }
                    _ => return Err(usage.into()),
                }
            }
            let mut surfaces = Vec::new();
            if let Some(path) = defaults {
                surfaces.push(path);
            }
            surfaces.push(Path::new(&args[3]));
            let report = asc_session::place(
                Path::new(&args[1]),
                Path::new(&args[2]),
                &surfaces,
                spawn,
                mode,
                compound,
            )?;
            println!("{}", serde_json::to_string_pretty(&report)?);
            if report.has_errors {
                return Err("Placement report contains errors; inspect its diagnostics.".into());
            }
        }
        "ground" if args.len() >= 3 => {
            let config = Path::new(&args[1]);
            let selected = Path::new(&args[2]);
            let mut defaults = None;
            let mut probes = Vec::new();
            let mut distance = 1000.;
            let mut flags = args[3..].iter();
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--defaults") => defaults = Some(Path::new(flags.next().ok_or(usage)?)),
                    Some("--distance") => {
                        distance = asc_config::finite_number(
                            flags.next().and_then(|v| v.to_str()).ok_or(usage)?,
                            "<ground query>",
                            0,
                        )?
                    }
                    Some("--probe") => {
                        let mut point = [0.; 3];
                        for axis in &mut point {
                            *axis = asc_config::finite_number(
                                flags.next().and_then(|v| v.to_str()).ok_or(usage)?,
                                "<ground query>",
                                0,
                            )?;
                        }
                        if probes.len() >= 4096 {
                            return Err("Ground command exceeds 4096 probes.".into());
                        }
                        probes.push(point);
                    }
                    _ => return Err(usage.into()),
                }
            }
            if !distance.is_finite() || !(0. ..=10_000_000.).contains(&distance) {
                return Err("ground_query_range: distance must be in 0..10000000.".into());
            }
            let mut paths = Vec::new();
            if let Some(path) = defaults {
                paths.push(path);
            }
            paths.push(selected);
            let ground = asc_ground::load(config, &paths)?;
            let hits = probes
                .into_iter()
                .map(|origin| {
                    ground
                        .probe(origin, distance)
                        .map(|hit| serde_json::json!({"origin":origin,"hit":hit}))
                })
                .collect::<Result<Vec<_>, _>>()?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"summary":ground.summary(),"probes":hits})
                )?
            );
        }
        "acd" if args.len() == 3 => {
            let path = Path::new(&args[1]);
            let archive = asc_acd::Archive::open(
                path,
                args[2].to_str().ok_or(usage)?,
                &path.to_string_lossy(),
            )?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"scope":"acd_index_only","header":archive.header,"entries":archive.entries(),"authenticated":false})
                )?
            );
        }
        "car" if args.len() >= 2 => {
            let mut mode = asc_vehicle::data::Mode::Auto;
            let mut compound = None;
            let mut flags = args[2..].iter();
            while let Some(flag) = flags.next() {
                match flag.to_str() {
                    Some("--data") => {
                        mode = match flags.next().and_then(|v| v.to_str()) {
                            Some("auto") => asc_vehicle::data::Mode::Auto,
                            Some("packed") => asc_vehicle::data::Mode::Packed,
                            Some("unpacked") => asc_vehicle::data::Mode::Unpacked,
                            _ => return Err(usage.into()),
                        }
                    }
                    Some("--compound") => {
                        compound = Some(
                            flags
                                .next()
                                .and_then(|v| v.to_str())
                                .ok_or(usage)?
                                .parse::<u32>()?,
                        )
                    }
                    _ => return Err(usage.into()),
                }
            }
            println!(
                "{}",
                serde_json::to_string_pretty(&asc_vehicle::load(
                    Path::new(&args[1]),
                    mode,
                    compound
                )?)?
            );
        }
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
        "materials" if args.len() == 2 => {
            let input = load_input(Path::new(&args[1]), true)?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &serde_json::json!({"manifest":input.manifest,"materials":input.scene.materials})
                )?
            );
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
