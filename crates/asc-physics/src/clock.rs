use crate::{Error, Result};
use serde::Serialize;
use std::time::Duration;

const CREDIT_PER_STEP: u128 = 1_000_000_000;
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Tick {
    pub index: u64,
    pub dt_seconds: f64,
    pub start_seconds: f64,
}
#[derive(Debug, Serialize)]
pub struct Batch {
    pub executed_steps: u32,
    pub completed_ticks: u64,
    pub pending_steps: u64,
    pub interpolation_alpha: Option<f64>,
}
/// Integer nanosecond credits; work is bounded per advance, elapsed time is never silently dropped.
#[derive(Debug, Clone)]
pub struct FixedClock {
    hz: u32,
    budget: u32,
    completed: u64,
    credit: u128,
}
impl FixedClock {
    pub fn new(hz: u32, max_steps_per_advance: u32) -> Result<Self> {
        if !(30..=10_000).contains(&hz) || !(1..=10_000).contains(&max_steps_per_advance) {
            return Err(Error::new(
                "physics_clock_config",
                "Rate must be 30..10000 Hz and step budget 1..10000.",
            ));
        }
        Ok(Self {
            hz,
            budget: max_steps_per_advance,
            completed: 0,
            credit: 0,
        })
    }
    pub fn completed_ticks(&self) -> u64 {
        self.completed
    }
    pub fn pending_steps(&self) -> u64 {
        (self.credit / CREDIT_PER_STEP) as u64
    }
    pub fn dt_seconds(&self) -> f64 {
        1. / f64::from(self.hz)
    }
    /// Each callback must commit its own state only on success. Successful earlier ticks stay committed.
    /// A failed tick keeps its credit and is retried at the same index on a later advance.
    pub fn advance(
        &mut self,
        elapsed: Duration,
        mut step: impl FnMut(Tick) -> Result<()>,
    ) -> Result<Batch> {
        let incoming = elapsed.as_nanos().checked_mul(u128::from(self.hz));
        let Some(credit) = incoming.and_then(|n| self.credit.checked_add(n)) else {
            return Err(Error::new(
                "physics_clock_range",
                "Elapsed-time credits overflow.",
            ));
        };
        if credit / CREDIT_PER_STEP > u128::from(u64::MAX - self.completed) {
            return Err(Error::new(
                "physics_clock_range",
                "Pending time would exceed the tick counter.",
            ));
        }
        self.credit = credit;
        let mut executed = 0;
        while self.credit >= CREDIT_PER_STEP && executed < self.budget {
            step(Tick {
                index: self.completed,
                dt_seconds: self.dt_seconds(),
                start_seconds: self.completed as f64 / f64::from(self.hz),
            })?;
            self.completed += 1;
            self.credit -= CREDIT_PER_STEP;
            executed += 1;
        }
        let pending = self.pending_steps();
        Ok(Batch {
            executed_steps: executed,
            completed_ticks: self.completed,
            pending_steps: pending,
            interpolation_alpha: (pending == 0)
                .then_some(self.credit as f64 / CREDIT_PER_STEP as f64),
        })
    }
}
