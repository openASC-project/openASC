use asc_physics::{Damper, DamperBranch, LinkSpring};
use serde::{Deserialize, Serialize};
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Branch {
    slow_n_s_m: f64,
    fast_n_s_m: f64,
    knee_m_s: f64,
}
impl Branch {
    fn build(&self) -> asc_physics::Result<DamperBranch> {
        DamperBranch::new(self.slow_n_s_m, self.fast_n_s_m, self.knee_m_s)
    }
}
#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Spring {
    fixed_anchor_m: [f64; 3],
    upright_local_point_m: [f64; 3],
    rest_length_m: f64,
    stiffness_n_m: f64,
    compression: Branch,
    rebound: Branch,
}
impl Spring {
    pub(super) fn build(&self) -> asc_physics::Result<LinkSpring> {
        LinkSpring::new(
            self.fixed_anchor_m,
            self.upright_local_point_m,
            self.rest_length_m,
            self.stiffness_n_m,
            Damper::new(self.compression.build()?, self.rebound.build()?),
        )
    }
}
