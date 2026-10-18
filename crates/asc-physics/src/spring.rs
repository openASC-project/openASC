//! Independent passive spring/damper elements; no automatic AC suspension mapping.
use crate::{Body, Error, Result, Wrench};
use glam::DVec3;
use serde::Serialize;

#[derive(Debug, Clone, Copy)]
pub struct DamperBranch {
    slow: f64,
    fast: f64,
    knee: f64,
}
impl DamperBranch {
    pub fn new(slow_n_s_m: f64, fast_n_s_m: f64, knee_m_s: f64) -> Result<Self> {
        if [slow_n_s_m, fast_n_s_m]
            .iter()
            .any(|v| !v.is_finite() || !(0. ..=1e9).contains(v))
            || !knee_m_s.is_finite()
            || !(0. ..=1e3).contains(&knee_m_s)
        {
            return Err(Error::new(
                "physics_damper",
                "Expected nonnegative finite damping slopes up to 1e9 N s/m and a knee within 0..1000 m/s.",
            ));
        }
        Ok(Self {
            slow: slow_n_s_m,
            fast: fast_n_s_m,
            knee: knee_m_s,
        })
    }
    fn magnitude(self, speed: f64) -> f64 {
        self.slow * speed.min(self.knee) + self.fast * (speed - self.knee).max(0.)
    }
}
#[derive(Debug, Clone, Copy)]
pub struct Damper {
    compression: DamperBranch,
    rebound: DamperBranch,
}
impl Damper {
    pub fn new(compression: DamperBranch, rebound: DamperBranch) -> Self {
        Self {
            compression,
            rebound,
        }
    }
    /// Signed tension at the element: negative extension rate is compression.
    pub fn force_n(self, extension_rate_m_s: f64) -> Result<f64> {
        if !extension_rate_m_s.is_finite() || extension_rate_m_s.abs() > 1e6 {
            return Err(Error::new(
                "physics_damper_velocity",
                "Expected a finite relative extension rate within +/-1000000 m/s.",
            ));
        }
        Ok(if extension_rate_m_s < 0. {
            -self.compression.magnitude(-extension_rate_m_s)
        } else {
            self.rebound.magnitude(extension_rate_m_s)
        })
    }
}
#[derive(Debug, Clone)]
pub struct SpringLink {
    point_a: [f64; 3],
    point_b: [f64; 3],
    rest: f64,
    stiffness: f64,
    damper: Damper,
}
#[derive(Debug, Clone, Serialize)]
pub struct SpringSample {
    pub point_a_world_m: [f64; 3],
    pub point_b_world_m: [f64; 3],
    pub length_m: f64,
    pub extension_m: f64,
    pub extension_rate_m_s: f64,
    pub elastic_tension_n: f64,
    pub damping_tension_n: f64,
    pub force_on_a_world_n: [f64; 3],
    pub force_on_b_world_n: [f64; 3],
    pub potential_energy_j: f64,
    pub dissipated_power_w: f64,
}
impl SpringLink {
    pub fn new(
        point_a_body_m: [f64; 3],
        point_b_body_m: [f64; 3],
        rest_length_m: f64,
        stiffness_n_m: f64,
        damper: Damper,
    ) -> Result<Self> {
        if point_a_body_m
            .into_iter()
            .chain(point_b_body_m)
            .any(|v| !v.is_finite() || v.abs() > 1e4)
            || !rest_length_m.is_finite()
            || !(1e-6..=1e4).contains(&rest_length_m)
            || !stiffness_n_m.is_finite()
            || !(0. ..=1e9).contains(&stiffness_n_m)
        {
            return Err(Error::new(
                "physics_spring",
                "Expected finite attachment coordinates within +/-10000 m, rest length 1e-6..10000 m and stiffness 0..1e9 N/m.",
            ));
        }
        Ok(Self {
            point_a: point_a_body_m,
            point_b: point_b_body_m,
            rest: rest_length_m,
            stiffness: stiffness_n_m,
            damper,
        })
    }
    pub fn loads(&self, a: &Body, b: &Body) -> Result<(Wrench, Wrench, SpringSample)> {
        let pa = a.world_point(self.point_a)?;
        let pb = b.world_point(self.point_b)?;
        let delta = DVec3::from_array(pb) - DVec3::from_array(pa);
        let length = delta.length();
        if !(1e-9..=1e4).contains(&length) {
            return Err(Error::new(
                "physics_spring_geometry",
                "Spring endpoints coincide or separation exceeds 10000 metres.",
            ));
        }
        let axis = delta / length;
        let va = DVec3::from_array(a.point_velocity_world(pa)?);
        let vb = DVec3::from_array(b.point_velocity_world(pb)?);
        let rate = (vb - va).dot(axis);
        let extension = length - self.rest;
        let elastic = self.stiffness * extension;
        let damping = self.damper.force_n(rate)?;
        let force = axis * (elastic + damping);
        let on_a = a.force_at_world_point(force.to_array(), pa)?;
        let on_b = b.force_at_world_point((-force).to_array(), pb)?;
        Ok((
            on_a,
            on_b,
            SpringSample {
                point_a_world_m: pa,
                point_b_world_m: pb,
                length_m: length,
                extension_m: extension,
                extension_rate_m_s: rate,
                elastic_tension_n: elastic,
                damping_tension_n: damping,
                force_on_a_world_n: force.to_array(),
                force_on_b_world_n: (-force).to_array(),
                potential_energy_j: 0.5 * self.stiffness * extension * extension,
                dissipated_power_w: damping * rate,
            },
        ))
    }
    /// Midpoint link forces, constant external world wrenches. Both bodies commit together.
    /// Returns the actual midpoint force sample, not a post-step endpoint sample.
    pub fn step_pair(
        &self,
        a: &mut Body,
        b: &mut Body,
        dt_seconds: f64,
        external_a: Wrench,
        external_b: Wrench,
    ) -> Result<SpringSample> {
        let mut bodies = [a.clone(), b.clone()];
        let connections = [SpringConnection::new(0, 1, self.clone())?];
        let mut samples = step_springs(
            &mut bodies,
            &connections,
            dt_seconds,
            &[external_a, external_b],
        )?;
        let [next_a, next_b] = bodies;
        *a = next_a;
        *b = next_b;
        Ok(samples.remove(0))
    }
}

#[derive(Debug, Clone)]
pub struct SpringConnection {
    a: usize,
    b: usize,
    link: SpringLink,
}
impl SpringConnection {
    pub fn new(body_a: usize, body_b: usize, link: SpringLink) -> Result<Self> {
        if body_a == body_b {
            return Err(Error::new(
                "physics_spring_graph",
                "A spring connection requires two distinct bodies.",
            ));
        }
        Ok(Self {
            a: body_a,
            b: body_b,
            link,
        })
    }
}
fn collect(
    bodies: &[Body],
    connections: &[SpringConnection],
    external: &[Wrench],
) -> Result<(Vec<Wrench>, Vec<SpringSample>)> {
    let mut loads = external.to_vec();
    let mut samples = Vec::with_capacity(connections.len());
    for c in connections {
        let a = bodies.get(c.a).ok_or_else(|| {
            Error::new(
                "physics_spring_graph",
                "Spring body index is outside the supplied state array.",
            )
        })?;
        let b = bodies.get(c.b).ok_or_else(|| {
            Error::new(
                "physics_spring_graph",
                "Spring body index is outside the supplied state array.",
            )
        })?;
        let (fa, fb, sample) = c.link.loads(a, b)?;
        loads[c.a].add(fa)?;
        loads[c.b].add(fb)?;
        samples.push(sample);
    }
    Ok((loads, samples))
}
/// Aggregate every link at common midpoint states, then commit every body once.
/// No body is modified on a topology/load/step error. Samples are in connection order.
pub fn step_springs(
    bodies: &mut [Body],
    connections: &[SpringConnection],
    dt_seconds: f64,
    external: &[Wrench],
) -> Result<Vec<SpringSample>> {
    if !dt_seconds.is_finite() || !(2e-6..=1. / 30.).contains(&dt_seconds) {
        return Err(Error::new(
            "physics_spring_step",
            "Coupled step must be finite within 2e-6..1/30 seconds.",
        ));
    }
    if bodies.is_empty()
        || bodies.len() > 4096
        || connections.len() > 16384
        || external.len() != bodies.len()
    {
        return Err(Error::new(
            "physics_spring_graph",
            "Expected 1..4096 bodies, at most 16384 links and one external wrench per body.",
        ));
    }
    let (first, _) = collect(bodies, connections, external)?;
    let mut half = bodies.to_vec();
    for (body, load) in half.iter_mut().zip(first) {
        body.step(dt_seconds * 0.5, load)?;
    }
    let (midpoint, samples) = collect(&half, connections, external)?;
    let mut next = bodies.to_vec();
    for (body, load) in next.iter_mut().zip(midpoint) {
        body.step(dt_seconds, load)?;
    }
    bodies.clone_from_slice(&next);
    Ok(samples)
}
