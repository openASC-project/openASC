//! Native rigid bodies, springs and coupled chassis/upright constraint dynamics.
//! Original Assetto Corsa force models and tyre/contact response remain unimplemented.
mod body;
mod chassis;
mod clock;
mod coupled_linear;
mod differential;
mod inertia;
mod link_spring;
mod linkage;
mod spring;
mod strut;
mod travel;
mod wheel;
pub use body::{Body, State, Wrench};
pub use chassis::{ChassisSample, ChassisSystem, MountedUpright, UprightSample};
pub use clock::{Batch, FixedClock, Tick};
pub use differential::{
    ConstraintBalance, GeneralizedLoads, LinkDifferential, LinkLoad, RelativeAcceleration,
    RelativeVelocity, TangentColumn,
};
pub use inertia::MassProperties;
pub use link_spring::{LinkSpring, LinkSpringSample};
pub use linkage::{FiveLink, LinkPose, LinkSolution};
pub use spring::{Damper, DamperBranch, SpringConnection, SpringLink, SpringSample, step_springs};
use std::fmt;
pub use strut::{StrutLink, StrutSolution};
pub use travel::{TravelMechanism, TravelSample, TravelState, TravelSystem};
pub use wheel::{Wheel, WheelLoad, WheelSample, WheelState};

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
