//! Native unconstrained rigid-body foundation. This is not an Assetto Corsa force model.
mod body;
mod clock;
mod inertia;
pub use body::{Body, State, Wrench};
pub use clock::{Batch, FixedClock, Tick};
pub use inertia::MassProperties;
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub struct Error {
    pub code: &'static str,
    pub message: String,
}
impl Error {
    pub fn new(code: &'static str, message: &str) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}
impl std::error::Error for Error {}
pub type Result<T> = std::result::Result<T, Error>;
