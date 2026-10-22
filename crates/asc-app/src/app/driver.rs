//! Native interactive driving workbench. Synthetic force laws; no original-game parity claim.
use super::{
    chassis_run::{self, Workbench},
    drive_input::{Keys, SteeringActuator},
    session,
};
use asc_physics::FixedClock;
use asc_render::{
    Resources,
    gpu::{
        presenter::{Outcome, Presenter},
        scene::Result,
    },
    scene::{orbit::Orbit, pose::RigidPose},
};
use glam::DVec3;
use serde::Deserialize;
use std::{
    io::Read,
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use winit::{
    application::ApplicationHandler,
    dpi::PhysicalSize,
    event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    dynamics_config: String,
    model_origin_in_chassis_m: [f64; 3],
    model_orientation_in_chassis_xyzw: [f64; 4],
    compound: Option<u32>,
    steering: SteeringActuator,
}
pub struct Options {
    pub fallback: bool,
    pub diagnostic: bool,
    pub rgba: bool,
    pub smoke: bool,
    pub captures: Option<PathBuf>,
}
struct State {
    window: Arc<Window>,
    presenter: Presenter,
    camera: Orbit,
    home: Orbit,
    keys: Keys,
    paused: bool,
    reason: Option<String>,
    clock: FixedClock,
    last: Instant,
    last_position: DVec3,
    initial_position: DVec3,
    follow: bool,
    cursor: Option<(f64, f64)>,
    mouse_left: bool,
    mouse_right: bool,
    capture: bool,
    frames: u32,
    retries: u32,
    physics_wall: Duration,
}
struct App {
    bench: Workbench,
    scene: asc_kn5::Scene,
    instance: asc_render::gpu::scene::InstanceDescriptor,
    focus: [[f32; 3]; 2],
    correction: RigidPose,
    resources: Option<Resources>,
    settings: SteeringActuator,
    options: Options,
    hz: u32,
    state: Option<State>,
    error: Option<String>,
    started: Instant,
    smoke_records: Vec<serde_json::Value>,
    metadata: serde_json::Value,
    origin: DVec3,
}
pub fn run(path: &Path, options: Options) -> Result<()> {
    if options.smoke {
        if let Some(dir) = &options.captures {
            if (0..4).any(|i| dir.join(format!("drive-smoke-{i}.png")).exists()) {
                return Err("Driving smoke capture already exists; choose a new directory".into());
            }
        }
    }
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(8193)
        .read_to_end(&mut bytes)?;
    if bytes.len() > 8192 {
        return Err("Native drive configuration exceeds 8192 bytes".into());
    }
    let config: Configuration = serde_json::from_slice(&bytes)?;
    config.steering.validate()?;
    let parent = path.parent().unwrap_or(Path::new("."));
    let bench = chassis_run::load(&parent.join(&config.dynamics_config))?;
    let hz = (1. / bench.step_s()).round();
    if !(30. ..=10000.).contains(&hz) || (hz * bench.step_s() - 1.).abs() > 1e-10 {
        return Err(
            "Native drive requires step_s to be the reciprocal of an integer 30..10000 Hz rate"
                .into(),
        );
    }
    let (car, track, mode) = bench.visual_paths()?;
    let state = bench.system.kinematics()?.chassis;
    let (preview, correction) = session::load_body(
        &car,
        &track,
        mode,
        config.compound,
        RigidPose::new(state.position_m, state.orientation_xyzw)?,
        RigidPose::new(
            config.model_origin_in_chassis_m,
            config.model_orientation_in_chassis_xyzw,
        )?,
    )?;
    let mut resources =
        (!options.diagnostic).then(|| asc_render::prepare_track_textures(&preview.track));
    if let Some(r) = &mut resources {
        r.allow_bc = !options.rgba;
    }
    let event_loop = EventLoop::new()?;
    let mut app = App {
        bench,
        scene: preview.track.scene,
        instance: preview.instance,
        focus: preview.focus,
        correction,
        resources,
        settings: config.steering,
        options,
        hz: hz as u32,
        state: None,
        error: None,
        started: Instant::now(),
        smoke_records: Vec::new(),
        metadata: preview.metadata,
        origin: DVec3::from_array(state.position_m),
    };
    println!(
        "Native drive: Enter pause/resume; W throttle; S/Space brake; A/D steer; R direction while paused; F5 reset; C follow; mouse orbit/pan/zoom; Home car fit; F12 capture; Escape close."
    );
    println!(
        "{}",
        serde_json::json!({"native_drive_source":app.metadata,"physics_diagnostics":app.bench.diagnostics(),"physics_hz":app.hz,"original_vehicle_dynamics_validated":false,"wheel_mesh_animation":false,"body_collision":false})
    );
    event_loop.run_app(&mut app)?;
    if let Some(e) = app.error {
        return Err(e.into());
    }
    Ok(())
}
impl App {
    fn create(&mut self, event_loop: &ActiveEventLoop) -> Result<()> {
        if self.state.is_some() {
            return Ok(());
        }
        let size = if self.options.smoke {
            PhysicalSize::new(800, 600)
        } else {
            PhysicalSize::new(1240, 900)
        };
        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("openASC | Loading native drive")
                    .with_inner_size(size)
                    .with_min_inner_size(PhysicalSize::new(160, 120))
                    .with_max_inner_size(PhysicalSize::new(4096, 4096)),
            )?,
        );
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..wgpu::InstanceDescriptor::new_with_display_handle(Box::new(
                event_loop.owned_display_handle(),
            ))
        });
        let surface = instance.create_surface(window.clone())?;
        let size = window.inner_size();
        let presenter = pollster::block_on(Presenter::with_instances(
            instance,
            surface,
            &self.scene,
            self.resources.as_ref(),
            (size.width, size.height),
            self.options.fallback,
            std::slice::from_ref(&self.instance),
        ))?;
        let mut camera = Orbit::new(
            self.focus,
            self.scene
                .summary
                .bounds
                .ok_or("Driving scene has no bounds")?,
        )?;
        let p = DVec3::from_array(self.bench.system.kinematics()?.chassis.position_m);
        camera.translate_target((p - self.origin).to_array())?;
        self.state = Some(State {
            window,
            presenter,
            camera: camera.clone(),
            home: camera,
            keys: Keys::default(),
            paused: !self.options.smoke,
            reason: None,
            clock: FixedClock::new(self.hz, 128)?,
            last: Instant::now(),
            last_position: p,
            initial_position: p,
            follow: !self.options.smoke,
            cursor: None,
            mouse_left: false,
            mouse_right: false,
            capture: false,
            frames: 0,
            retries: 0,
            physics_wall: Duration::ZERO,
        });
        let state = self.state.as_ref().unwrap();
        println!(
            "{}",
            serde_json::json!({"native_drive_window":{"adapter":state.presenter.context.info.name,
            "backend":format!("{:?}",state.presenter.context.info.backend),"format":format!("{:?}",state.presenter.format()),
            "scene":state.presenter.scene.statistics,"root_instances":state.presenter.scene.instance_count()}})
        );
        state.window.request_redraw();
        Ok(())
    }
    fn pause(&mut self, reason: Option<String>) {
        if let Some(s) = &mut self.state {
            s.paused = true;
            s.keys.clear();
            s.reason = reason;
            s.last = Instant::now();
            s.clock = FixedClock::new(self.hz, 128).unwrap();
            s.window.request_redraw();
        }
    }
    fn frame(&mut self, event_loop: &ActiveEventLoop) -> Result<()> {
        if self.options.smoke {
            let t = self.bench.time_s();
            if self.state.as_ref().unwrap().frames == 0 {
                self.pause(None);
                self.key(event_loop, KeyCode::KeyW, true, false)?;
                if self.state.as_ref().unwrap().keys.command().drive != 0. {
                    return Err("Paused input was not cleared".into());
                }
                self.key(event_loop, KeyCode::Enter, true, false)?;
            }
            self.key(event_loop, KeyCode::KeyW, t < 1.6 - 1e-9, false)?;
            self.key(
                event_loop,
                KeyCode::KeyA,
                (0.6..1.6 - 1e-9).contains(&t),
                false,
            )?;
            self.key(event_loop, KeyCode::KeyS, t >= 1.6 - 1e-9, false)?;
        }
        let s = self.state.as_mut().unwrap();
        let size = s.window.inner_size();
        if size.width == 0 || size.height == 0 {
            return Ok(());
        }
        let elapsed = if self.options.smoke {
            if s.frames == 0 {
                Duration::ZERO
            } else {
                Duration::from_millis(20)
            }
        } else {
            s.last.elapsed()
        };
        s.last = Instant::now();
        if !s.paused {
            let physics_started = Instant::now();
            let input = s.keys.command();
            let settings = self.settings;
            let bench = &mut self.bench;
            match s
                .clock
                .advance(elapsed, |_| bench.live_step(input, settings).map(|_| ()))
            {
                Ok(batch) if batch.pending_steps > u64::from(self.hz) / 4 => {
                    if self.options.smoke {
                        return Err("Native drive smoke exceeded the fixed-step work budget".into());
                    }
                    s.paused = true;
                    s.keys.clear();
                    s.reason =
                        Some("Simulation lag; paused without skipping physical state".into());
                    s.clock = FixedClock::new(self.hz, 128)?;
                }
                Err(e) => {
                    if self.options.smoke {
                        return Err(e.into());
                    }
                    println!("Native drive paused: {e}");
                    s.paused = true;
                    s.keys.clear();
                    s.reason = Some(e.to_string());
                    s.clock = FixedClock::new(self.hz, 128)?;
                }
                _ => {}
            }
            s.physics_wall += physics_started.elapsed();
        }
        let body = self.bench.system.kinematics()?.chassis;
        let p = DVec3::from_array(body.position_m);
        s.presenter.scene.set_instance_pose(
            1,
            RigidPose::new(body.position_m, body.orientation_xyzw)?.compose(self.correction)?,
        )?;
        if s.follow {
            s.camera
                .translate_target((p - s.last_position).to_array())?;
        }
        s.last_position = p;
        let speed = DVec3::from_array(body.velocity_world_m_s).length() * 3.6;
        s.window.set_title(&format!(
            "openASC | {} | {:.1} km/h | {:.2} s | {} | W gas S brake A/D steer Enter pause | {}",
            if s.paused { "PAUSED" } else { "Native drive" },
            speed,
            self.bench.time_s(),
            if s.keys.reverse { "Reverse" } else { "Forward" },
            s.reason
                .as_deref()
                .unwrap_or("Synthetic workbench; no AC parity")
        ));
        let milestones = [0., 1., 1.6, 2.4];
        let smoke_capture = self.options.smoke
            && self.smoke_records.len() < 4
            && self.bench.time_s() + 1e-9 >= milestones[self.smoke_records.len()];
        let (w, h) = s.presenter.size();
        let view = s.camera.view(w, h, self.resources.is_some())?;
        s.window.pre_present_notify();
        match pollster::block_on(s.presenter.present(&view, s.capture || smoke_capture))? {
            Outcome::Presented(frame) => {
                s.retries = 0;
                s.frames += 1;
                if let Some(frame) = frame {
                    if let Some(dir) = &self.options.captures {
                        std::fs::create_dir_all(dir)?;
                        let name = if smoke_capture {
                            format!("drive-smoke-{}.png", self.smoke_records.len())
                        } else {
                            format!(
                                "openasc-drive-{}.png",
                                SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos()
                            )
                        };
                        asc_render::save_png(&frame, &dir.join(name))?;
                    }
                    if smoke_capture {
                        if frame.report.non_background_pixels == 0 {
                            return Err("Driving frame contains no geometry".into());
                        }
                        let hash = frame.rgba.iter().fold(14695981039346656037u64, |h, &v| {
                            (h ^ u64::from(v)).wrapping_mul(1099511628211)
                        });
                        self.smoke_records.push(serde_json::json!({"time_s":self.bench.time_s(),"body":body,"input":s.keys.command(),"pixel_hash":hash,"non_background_pixels":frame.report.non_background_pixels}));
                    }
                    s.capture = false;
                }
                if self.options.smoke && self.smoke_records.len() == 4 {
                    let first = &self.smoke_records[0];
                    let last = &self.smoke_records[3];
                    let start: [f64; 3] =
                        serde_json::from_value(first["body"]["position_m"].clone())?;
                    if (p - DVec3::from_array(start)).length() < 0.05
                        || first["pixel_hash"] == last["pixel_hash"]
                    {
                        return Err("Driving smoke did not move chassis and surface pixels".into());
                    }
                    println!(
                        "{}",
                        serde_json::json!({"native_drive_smoke":{"records":self.smoke_records,"presented_frames":s.frames,"physics_wall_time_s":s.physics_wall.as_secs_f64(),"root_instances":s.presenter.scene.instance_count(),"input_injection":"diagnostic shared native key handler; physical keyboard not independently verified","original_vehicle_dynamics_validated":false}})
                    );
                    event_loop.exit();
                }
            }
            Outcome::Lost => {
                s.retries += 1;
                if s.retries > 3 {
                    return Err("Native drive surface recovery exceeded three attempts".into());
                }
                let surface = s.presenter.instance.create_surface(s.window.clone())?;
                s.presenter.replace_surface(surface)?;
            }
            Outcome::Timeout | Outcome::Reconfigured => {
                s.retries += 1;
                if s.retries > 3 {
                    return Err("Native drive surface retries exceeded three attempts".into());
                }
                s.window.request_redraw();
            }
            Outcome::Occluded => {}
        }
        Ok(())
    }
    fn key(
        &mut self,
        event_loop: &ActiveEventLoop,
        key: KeyCode,
        down: bool,
        repeat: bool,
    ) -> Result<()> {
        let s = self.state.as_mut().unwrap();
        s.keys.event(key, down, repeat, s.paused);
        if down && !repeat {
            match key {
                KeyCode::Escape => event_loop.exit(),
                KeyCode::Enter => {
                    s.paused = !s.paused;
                    s.keys.clear();
                    s.reason = None;
                    s.last = Instant::now();
                    s.clock = FixedClock::new(self.hz, 128)?;
                }
                KeyCode::KeyR if s.paused => s.keys.reverse = !s.keys.reverse,
                KeyCode::F5 => {
                    self.bench.reset();
                    self.pause(None);
                    self.state.as_mut().unwrap().keys.reverse = false;
                }
                KeyCode::KeyC => s.follow = !s.follow,
                KeyCode::Home => {
                    s.camera = s.home.clone();
                    s.camera
                        .translate_target((s.last_position - s.initial_position).to_array())?;
                }
                KeyCode::F12 => s.capture = self.options.captures.is_some(),
                _ => {}
            }
        }
        self.state.as_ref().unwrap().window.request_redraw();
        Ok(())
    }
    fn handle(&mut self, event_loop: &ActiveEventLoop, event: WindowEvent) -> Result<()> {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Focused(false) | WindowEvent::Occluded(true) => {
                if !self.options.smoke {
                    self.pause(Some("Focus lost or window hidden".into()));
                }
            }
            WindowEvent::Resized(size) => {
                self.state
                    .as_mut()
                    .unwrap()
                    .presenter
                    .resize(size.width, size.height)?;
                if !self.options.smoke {
                    self.pause(None);
                }
            }
            WindowEvent::ScaleFactorChanged { .. } => {
                let s = self.state.as_mut().unwrap();
                let size = s.window.inner_size();
                s.presenter.resize(size.width, size.height)?;
                if !self.options.smoke {
                    self.pause(None);
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(key) = event.physical_key {
                    self.key(
                        event_loop,
                        key,
                        event.state == ElementState::Pressed,
                        event.repeat,
                    )?;
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                let s = self.state.as_mut().unwrap();
                if let Some((x, y)) = s.cursor {
                    if s.mouse_left {
                        s.camera.orbit(position.x - x, position.y - y);
                    } else if s.mouse_right {
                        let (w, h) = s.presenter.size();
                        s.camera.pan(position.x - x, position.y - y, w, h);
                    }
                }
                s.cursor = Some((position.x, position.y));
                s.window.request_redraw();
            }
            WindowEvent::CursorLeft { .. } => {
                let s = self.state.as_mut().unwrap();
                s.cursor = None;
                s.mouse_left = false;
                s.mouse_right = false;
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let s = self.state.as_mut().unwrap();
                match button {
                    MouseButton::Left => s.mouse_left = state == ElementState::Pressed,
                    MouseButton::Right => s.mouse_right = state == ElementState::Pressed,
                    _ => {}
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let s = self.state.as_mut().unwrap();
                s.camera.zoom(match delta {
                    MouseScrollDelta::LineDelta(_, y) => f64::from(y),
                    MouseScrollDelta::PixelDelta(p) => p.y / 40.,
                });
                s.window.request_redraw();
            }
            WindowEvent::RedrawRequested => self.frame(event_loop)?,
            _ => {}
        }
        Ok(())
    }
}
impl ApplicationHandler for App {
    fn resumed(&mut self, e: &ActiveEventLoop) {
        if let Err(error) = self.create(e) {
            self.error = Some(error.to_string());
            e.exit();
        }
    }
    fn window_event(&mut self, e: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if self.state.as_ref().is_some_and(|s| s.window.id() == id) {
            if let Err(error) = self.handle(e, event) {
                self.error = Some(error.to_string());
                e.exit();
            }
        }
    }
    fn suspended(&mut self, _: &ActiveEventLoop) {
        self.pause(Some("Application suspended".into()));
        self.state = None;
    }
    fn about_to_wait(&mut self, e: &ActiveEventLoop) {
        if self.options.smoke && self.started.elapsed() > Duration::from_secs(120) {
            self.error = Some("Native drive smoke timed out".into());
            e.exit();
            return;
        }
        if let Some(s) = &self.state {
            if !s.paused || self.options.smoke {
                s.window.request_redraw();
                e.set_control_flow(ControlFlow::WaitUntil(
                    Instant::now() + Duration::from_millis(16),
                ));
            } else {
                e.set_control_flow(ControlFlow::Wait);
            }
        }
    }
}
