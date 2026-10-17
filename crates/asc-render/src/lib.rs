//! Native rendering, organised by GPU work, pipelines, resources and scene data.
pub mod gpu;
mod pipelines;
pub mod resources;
pub mod scene;

pub use gpu::capture::{Frame, Report, render, render_with_textures, save_png};
pub use resources::texture::{Resources, load as load_textures};
pub use scene::camera::camera;
