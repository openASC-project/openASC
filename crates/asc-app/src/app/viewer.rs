//! Native window lifecycle and inspection controls. No vehicle simulation runs here.
use asc_kn5::Scene;
use asc_render::{
    Resources,
    gpu::{
        presenter::{Outcome, Presenter},
        scene::Result,
    },
    scene::orbit::Orbit,
};
use std::{
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use winit::{
    application::ApplicationHandler,
    dpi::{PhysicalPosition, PhysicalSize},
    event::{ElementState, MouseButton, MouseScrollDelta, StartCause, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

pub struct Options {
    pub fallback: bool,
    pub smoke: bool,
    pub captures: Option<PathBuf>,
    pub first_capture: Option<PathBuf>,
}
struct Input {
    left: bool,
    right: bool,
    cursor: Option<PhysicalPosition<f64>>,
}
impl Input {
    fn clear(&mut self) {
        self.left = false;
        self.right = false;
        self.cursor = None;
    }
}
struct State {
    window: Arc<Window>,
    presenter: Presenter,
    camera: Orbit,
    home: Orbit,
    input: Input,
    textures: bool,
    capture: bool,
    frames: u32,
    retries: u32,
    smoke_pixels: Vec<usize>,
    smoke_matrices: Vec<[f32; 16]>,
    smoke_hashes: Vec<u64>,
    smoke_modes: Vec<bool>,
    smoke_uncaptured: bool,
}
struct App {
    scene: Scene,
    resources: Option<Resources>,
    options: Options,
    state: Option<State>,
    error: Option<String>,
    started: Instant,
    smoke_result: Option<serde_json::Value>,
}

pub fn run(scene: Scene, resources: Option<Resources>, options: Options) -> Result<()> {
    if scene.summary.bounds.is_none() {
        return Err("No visible indexed static geometry to display.".into());
    }
    let event_loop = EventLoop::new()?;
    let smoke = options.smoke;
    let mut app = App {
        scene,
        resources,
        options,
        state: None,
        error: None,
        started: Instant::now(),
        smoke_result: None,
    };
    println!(
        "Native controls: left drag orbit; right drag pan; wheel zoom; Home road fit; F scene fit; V top; P projection; T diffuse colours; F12 capture when --captures is set; Escape close."
    );
    event_loop.run_app(&mut app)?;
    if let Some(error) = app.error {
        return Err(error.into());
    }
    if smoke {
        println!(
            "{}",
            serde_json::json!({"window_smoke":app.smoke_result.ok_or("Window smoke exited before completing.")?})
        );
    }
    Ok(())
}
impl App {
    fn create(&mut self, event_loop: &ActiveEventLoop) -> Result<()> {
        if self.state.is_some() {
            return Ok(());
        }
        let size = if self.options.smoke {
            PhysicalSize::new(319, 241)
        } else {
            PhysicalSize::new(1240, 900)
        };
        let window = Arc::new(
            event_loop.create_window(
                Window::default_attributes()
                    .with_title("openASC | Loading native viewer")
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
        let actual = window.inner_size();
        let presenter = pollster::block_on(Presenter::new(
            instance,
            surface,
            &self.scene,
            self.resources.as_ref(),
            (actual.width, actual.height),
            self.options.fallback,
        ))?;
        let bounds = self.scene.summary.bounds.unwrap();
        let camera = Orbit::new(
            self.scene.summary.road_focus_bounds.unwrap_or(bounds),
            bounds,
        )?;
        let state = State {
            window,
            presenter,
            camera: camera.clone(),
            home: camera,
            input: Input {
                left: false,
                right: false,
                cursor: None,
            },
            textures: self.resources.is_some(),
            capture: self.options.first_capture.is_some(),
            frames: 0,
            retries: 0,
            smoke_pixels: Vec::new(),
            smoke_matrices: Vec::new(),
            smoke_hashes: Vec::new(),
            smoke_modes: Vec::new(),
            smoke_uncaptured: false,
        };
        println!(
            "{}",
            serde_json::json!({"native_window":{"adapter":state.presenter.context.info.name,"backend":format!("{:?}",state.presenter.context.info.backend),"format":format!("{:?}",state.presenter.format()),"scene":state.presenter.scene.statistics}})
        );
        state.title();
        state.window.request_redraw();
        self.state = Some(state);
        Ok(())
    }
    fn handle(&mut self, event_loop: &ActiveEventLoop, event: WindowEvent) -> Result<()> {
        let Some(state) = self.state.as_mut() else {
            return Ok(());
        };
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                state.presenter.resize(size.width, size.height)?;
                state.input.clear();
                if size.width > 0 && size.height > 0 {
                    state.window.request_redraw();
                }
            }
            WindowEvent::ScaleFactorChanged { .. } => {
                let size = state.window.inner_size();
                state.presenter.resize(size.width, size.height)?;
                state.input.clear();
                state.window.request_redraw();
            }
            WindowEvent::Focused(false) | WindowEvent::CursorLeft { .. } => state.input.clear(),
            WindowEvent::Occluded(false) => state.window.request_redraw(),
            WindowEvent::CursorMoved { position, .. } => {
                if let Some(previous) = state.input.cursor {
                    let (dx, dy) = (position.x - previous.x, position.y - previous.y);
                    let (w, h) = state.presenter.size();
                    if state.input.left {
                        state.camera.orbit(dx, dy);
                        state.window.request_redraw();
                    } else if state.input.right {
                        state.camera.pan(dx, dy, w, h);
                        state.window.request_redraw();
                    }
                }
                state.input.cursor = Some(position);
            }
            WindowEvent::MouseInput {
                state: button_state,
                button,
                ..
            } => {
                let down = button_state == ElementState::Pressed;
                match button {
                    MouseButton::Left => state.input.left = down,
                    MouseButton::Right => state.input.right = down,
                    _ => {}
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let lines = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(p) => p.y / 40.,
                };
                state.camera.zoom(lines);
                state.window.request_redraw();
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed && !event.repeat =>
            {
                match event.physical_key {
                    PhysicalKey::Code(KeyCode::Escape) => event_loop.exit(),
                    PhysicalKey::Code(KeyCode::Home) => state.camera = state.home.clone(),
                    PhysicalKey::Code(KeyCode::KeyF) => {
                        let bounds = self.scene.summary.bounds.unwrap();
                        state.camera = Orbit::new(bounds, bounds)?;
                    }
                    PhysicalKey::Code(KeyCode::KeyV) => state.camera.top(),
                    PhysicalKey::Code(KeyCode::KeyP) => {
                        state.camera.perspective = !state.camera.perspective
                    }
                    PhysicalKey::Code(KeyCode::KeyT) => {
                        if self.resources.is_some() {
                            state.textures = !state.textures;
                        }
                    }
                    PhysicalKey::Code(KeyCode::F12) => {
                        if self.options.captures.is_some() {
                            state.capture = true;
                        } else {
                            println!(
                                "Capture disabled: pass --captures DIRECTORY when starting the viewer."
                            );
                        }
                    }
                    _ => return Ok(()),
                }
                state.title();
                state.window.request_redraw();
            }
            WindowEvent::RedrawRequested => {
                let size = state.window.inner_size();
                if size.width == 0 || size.height == 0 {
                    return Ok(());
                }
                if self.options.smoke && state.frames == 1 && state.presenter.size() != (640, 480) {
                    return Ok(());
                }
                let (w, h) = state.presenter.size();
                let view = state.camera.view(w, h, state.textures)?;
                state.window.pre_present_notify();
                match pollster::block_on(state.presenter.present(
                    &view,
                    (state.capture || self.options.smoke)
                        && (!self.options.smoke || state.smoke_uncaptured),
                ))? {
                    Outcome::Presented(frame) => {
                        if self.options.smoke && !state.smoke_uncaptured {
                            if frame.is_some() {
                                return Err(
                                    "Uncaptured presentation unexpectedly returned pixels.".into(),
                                );
                            }
                            state.smoke_uncaptured = true;
                            state.window.request_redraw();
                            return Ok(());
                        }
                        state.frames += 1;
                        state.retries = 0;
                        if let Some(frame) = frame {
                            if state.capture {
                                let path = if let Some(path) = self.options.first_capture.take() {
                                    path
                                } else {
                                    let directory = self.options.captures.as_ref().unwrap();
                                    std::fs::create_dir_all(directory)?;
                                    let serial =
                                        SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
                                    directory.join(format!("openasc-native-{serial}.png"))
                                };
                                asc_render::save_png(&frame, &path)?;
                                println!(
                                    "{}",
                                    serde_json::json!({"capture":path,"render":frame.report})
                                );
                                state.capture = false;
                            }
                            if self.options.smoke {
                                state.smoke_pixels.push(frame.report.non_background_pixels);
                                state.smoke_matrices.push(view.matrix);
                                state.smoke_hashes.push(
                                    frame.rgba.iter().fold(14695981039346656037u64, |h, &v| {
                                        (h ^ v as u64).wrapping_mul(1099511628211)
                                    }),
                                );
                                state.smoke_modes.push(frame.report.diffuse_enabled);
                            }
                        }
                        if self.options.smoke {
                            match state.frames {
                                1 => {
                                    state.camera.orbit(40., 20.);
                                    if let Some(size) =
                                        state.window.request_inner_size(PhysicalSize::new(640, 480))
                                    {
                                        state.presenter.resize(size.width, size.height)?;
                                    }
                                    state.window.request_redraw();
                                }
                                2 => {
                                    state.textures = false;
                                    state.window.request_redraw();
                                }
                                3 => {
                                    state.camera.pan(10., -5., w, h);
                                    state.camera.zoom(2.);
                                    state.camera.perspective = false;
                                    state.textures = self.resources.is_some();
                                    state.window.request_redraw();
                                }
                                _ => {
                                    if state.smoke_pixels.len() != 4
                                        || state.smoke_pixels.contains(&0)
                                        || (state.smoke_matrices[0] == state.smoke_matrices[1]
                                            || state.smoke_matrices[1] != state.smoke_matrices[2]
                                            || state.smoke_matrices[2] == state.smoke_matrices[3])
                                    {
                                        return Err(
                                            "Native window readback/camera smoke check failed."
                                                .into(),
                                        );
                                    }
                                    if state.presenter.scene.statistics.uploaded_textures > 0
                                        && state.smoke_hashes[1] == state.smoke_hashes[2]
                                    {
                                        return Err("Diffuse toggle did not change native surface pixels at a fixed camera.".into());
                                    }
                                    self.smoke_result = Some(
                                        serde_json::json!({"presented_frames":state.frames+u32::from(state.smoke_uncaptured),"readback_frames":state.smoke_pixels.len(),"configurations":state.presenter.configurations,"size":[w,h],"non_background_pixels":state.smoke_pixels,"camera_changed":true,"diffuse_modes":state.smoke_modes,"toggle_changed_pixels":state.smoke_hashes[1]!=state.smoke_hashes[2],"format":format!("{:?}",state.presenter.format())}),
                                    );
                                    event_loop.exit();
                                }
                            }
                        }
                    }
                    Outcome::Lost => {
                        state.retries += 1;
                        if state.retries > 3 {
                            return Err("Native surface recovery exceeded three attempts.".into());
                        }
                        let surface = state
                            .presenter
                            .instance
                            .create_surface(state.window.clone())?;
                        state.presenter.replace_surface(surface)?;
                        state.window.request_redraw();
                    }
                    Outcome::Reconfigured | Outcome::Timeout => {
                        state.retries += 1;
                        if state.retries <= 3 {
                            state.window.request_redraw();
                        } else {
                            println!(
                                "Native surface retry limit reached; resize or interact to retry."
                            );
                        }
                    }
                    Outcome::Occluded => {}
                }
            }
            _ => {}
        }
        Ok(())
    }
}
impl State {
    fn title(&self) {
        self.window.set_title(&format!("openASC | Native viewer | {} | Diffuse {} | Drag orbit · RMB pan · Wheel zoom · Home fit · F12 capture",if self.camera.perspective{"Perspective"}else{"Orthographic"},if self.textures{"on"}else{"off"}));
    }
}
impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if let Err(error) = self.create(event_loop) {
            self.error = Some(error.to_string());
            event_loop.exit();
        }
    }
    fn window_event(&mut self, event_loop: &ActiveEventLoop, id: WindowId, event: WindowEvent) {
        if self.state.as_ref().is_some_and(|s| s.window.id() == id) {
            if let Err(error) = self.handle(event_loop, event) {
                self.error = Some(error.to_string());
                event_loop.exit();
            }
        }
    }
    fn suspended(&mut self, _: &ActiveEventLoop) {
        self.state = None;
    }
    fn new_events(&mut self, event_loop: &ActiveEventLoop, _: StartCause) {
        if self.options.smoke && self.started.elapsed() > Duration::from_secs(30) {
            self.error = Some("Native window smoke timed out after 30 seconds.".into());
            event_loop.exit();
        }
    }
    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        event_loop.set_control_flow(if self.options.smoke {
            ControlFlow::WaitUntil(self.started + Duration::from_secs(30))
        } else {
            ControlFlow::Wait
        });
    }
}
