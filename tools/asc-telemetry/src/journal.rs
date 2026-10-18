use crate::{Identity, Snapshot, invalid, validate_live};
use serde::Serialize;
use std::io::{self, Write};

/// Streaming JSONL: absence of a complete footer means interrupted/failed capture.
/// A failed write may leave a partial final line; callers must retain that fact.
pub struct Journal<W: Write> {
    writer: W,
    identity: Identity,
    samples: u32,
    packet_changes: u32,
    previous: Option<(i32, u64)>,
    requested_samples: u32,
    poisoned: bool,
}
#[derive(Debug, Serialize)]
pub struct Summary {
    pub samples: u32,
    pub packet_changes: u32,
    pub duplicate_packets: u32,
    pub elapsed_ns: u64,
}
fn line(writer: &mut impl Write, value: &impl Serialize) -> io::Result<()> {
    serde_json::to_writer(&mut *writer, value)?;
    writer.write_all(b"\n")?;
    writer.flush()
}
impl<W: Write> Journal<W> {
    pub fn new(
        mut writer: W,
        initial: &Snapshot,
        requested_samples: u32,
        interval_ms: u64,
    ) -> io::Result<Self> {
        validate_live(initial, &initial.identity)?;
        if !(2..=6000).contains(&requested_samples)
            || !(1..=1000).contains(&interval_ms)
            || u64::from(requested_samples) * interval_ms > 60_000
        {
            return Err(invalid(
                "Use 2..6000 samples, 1..1000 ms interval and at most 60000 ms total",
            ));
        }
        line(
            &mut writer,
            &serde_json::json!({
                "type": "header", "schema": "openasc-ac-prefix-1",
                "identity": initial.identity, "requested_samples": requested_samples, "interval_ms": interval_ms,
                "consistency": "best_effort_equal_prefix_copies_not_atomic",
                "cross_page_synchronized": false, "original_physics_parity_established": false,
                "track_configuration_recorded": false, "car_setup_recorded": false,
                "aids_weather_tyres_recorded_completely": false,
                "time_basis": "collector_monotonic_receive_time_not_simulation_time",
                "completion_rule": "require_complete_footer_and_at_least_one_physics_packet_change"
            }),
        )?;
        Ok(Self {
            writer,
            identity: initial.identity.clone(),
            samples: 0,
            packet_changes: 0,
            previous: None,
            requested_samples,
            poisoned: false,
        })
    }
    pub fn sample(&mut self, snapshot: &Snapshot, elapsed_ns: u64) -> io::Result<()> {
        validate_live(snapshot, &self.identity)?;
        if self.poisoned
            || self.samples >= self.requested_samples
            || elapsed_ns > 60_000_000_000
            || self.previous.is_some_and(|(_, time)| elapsed_ns < time)
        {
            return Err(invalid("Sample budget or monotonic timestamp violated"));
        }
        let duplicate = self
            .previous
            .is_some_and(|(id, _)| id == snapshot.physics.packet_id);
        let result = line(
            &mut self.writer,
            &serde_json::json!({
                "type": "sample", "index": self.samples, "elapsed_ns": elapsed_ns,
                "duplicate_physics_packet": duplicate,
                "graphics": snapshot.graphics, "physics": snapshot.physics
            }),
        );
        if let Err(error) = result {
            self.poisoned = true;
            return Err(error);
        }
        if self.previous.is_some() && !duplicate {
            self.packet_changes += 1;
        }
        self.previous = Some((snapshot.physics.packet_id, elapsed_ns));
        self.samples += 1;
        Ok(())
    }
    pub fn finish(mut self) -> io::Result<Summary> {
        if self.poisoned || self.samples != self.requested_samples || self.packet_changes == 0 {
            return Err(invalid(
                "Incomplete capture or no changing physics packets; no complete footer written",
            ));
        }
        let summary = Summary {
            samples: self.samples,
            packet_changes: self.packet_changes,
            duplicate_packets: self.samples - 1 - self.packet_changes,
            elapsed_ns: self.previous.map_or(0, |(_, time)| time),
        };
        line(
            &mut self.writer,
            &serde_json::json!({ "type": "complete", "summary": summary }),
        )?;
        Ok(summary)
    }
}
