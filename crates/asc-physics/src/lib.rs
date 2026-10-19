//! Native rigid bodies, spring forces and position-level suspension geometry.
//! Original Assetto Corsa force models and dynamic constraint response remain unimplemented.
mod body;
mod clock;
mod differential;
mod inertia;
mod linkage;
mod spring;
mod strut;
pub use body::{Body, State, Wrench};
pub use clock::{Batch, FixedClock, Tick};
pub use differential::{
    ConstraintBalance, GeneralizedLoads, LinkDifferential, LinkLoad, RelativeVelocity,
    TangentColumn,
};
pub use inertia::MassProperties;
pub use linkage::{FiveLink, LinkPose, LinkSolution};
pub use spring::{Damper, DamperBranch, SpringConnection, SpringLink, SpringSample, step_springs};
use std::fmt;
pub use strut::{StrutLink, StrutSolution};

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
