//! Original byte fixtures, authored independently of any game/SDK binary.
use asc_telemetry::{
    GRAPHICS_BYTES, PHYSICS_BYTES, STATIC_BYTES, decode, journal::Journal, repeated_copy,
};
use std::io;

fn text(bytes: &mut [u8], offset: usize, value: &str) {
    for (i, unit) in value.encode_utf16().enumerate() {
        bytes[offset + i * 2..offset + i * 2 + 2].copy_from_slice(&unit.to_le_bytes());
    }
}
fn fixture() -> (
    [u8; PHYSICS_BYTES],
    [u8; GRAPHICS_BYTES],
    [u8; STATIC_BYTES],
) {
    let mut physics = [0; PHYSICS_BYTES];
    // Distinct exact values expose field-offset errors.
    for (i, chunk) in physics.chunks_exact_mut(4).enumerate() {
        chunk.copy_from_slice(&(i as f32 + 0.25).to_le_bytes());
    }
    for (offset, integer) in [(0, i32::MAX), (16, 3), (20, 4321)] {
        physics[offset..offset + 4].copy_from_slice(&integer.to_le_bytes());
    }
    let mut graphics = [0; GRAPHICS_BYTES];
    for (i, value) in [9876_i32, 2, 0].iter().enumerate() {
        graphics[i * 4..i * 4 + 4].copy_from_slice(&value.to_le_bytes());
    }
    let mut identity = [0; STATIC_BYTES];
    text(&mut identity, 0, "1.7");
    text(&mut identity, 30, "1.16.4");
    identity[60..64].copy_from_slice(&3_i32.to_le_bytes());
    identity[64..68].copy_from_slice(&7_i32.to_le_bytes());
    text(&mut identity, 68, "original_car");
    text(&mut identity, 134, "original_track");
    (physics, graphics, identity)
}

#[test]
fn independent_golden_offsets_include_last_wheel_and_utf16_boundaries() {
    let (p, g, s) = fixture();
    let value = decode(&p, &g, &s).unwrap();
    assert_eq!(value.identity.shared_memory_version, "1.7");
    assert_eq!(value.identity.game_version, "1.16.4");
    assert_eq!(value.identity.car, "original_car");
    assert_eq!(value.identity.track, "original_track");
    assert_eq!(
        (value.physics.packet_id, value.graphics.packet_id),
        (i32::MAX, 9876)
    );
    assert_eq!((value.physics.gear, value.physics.rpms), (3, 4321));
    let json = serde_json::to_value(&value.physics).unwrap();
    for (key, expected) in [
        ("gas", 1.25),
        ("brake", 2.25),
        ("fuel", 3.25),
        ("steer_angle", 6.25),
        ("speed_kmh", 7.25),
    ] {
        assert_eq!(json[key].as_f64().unwrap(), expected, "{key}");
    }
    for (key, expected) in [
        ("velocity", vec![8.25, 9.25, 10.25]),
        ("acc_g", vec![11.25, 12.25, 13.25]),
        ("wheel_slip", vec![14.25, 15.25, 16.25, 17.25]),
        ("wheel_load", vec![18.25, 19.25, 20.25, 21.25]),
        ("wheels_pressure", vec![22.25, 23.25, 24.25, 25.25]),
        ("wheel_angular_speed", vec![26.25, 27.25, 28.25, 29.25]),
        ("tyre_wear", vec![30.25, 31.25, 32.25, 33.25]),
        ("tyre_dirty_level", vec![34.25, 35.25, 36.25, 37.25]),
        ("tyre_core_temperature", vec![38.25, 39.25, 40.25, 41.25]),
        ("camber_rad", vec![42.25, 43.25, 44.25, 45.25]),
        ("suspension_travel", vec![46.25, 47.25, 48.25, 49.25]),
    ] {
        assert_eq!(json[key], serde_json::json!(expected), "{key}");
    }
}

#[test]
fn malformed_prefixes_nonfinite_floats_and_invalid_status_are_rejected() {
    let (p, g, s) = fixture();
    for end in 0..PHYSICS_BYTES {
        assert!(decode(&p[..end], &g, &s).is_err());
    }
    for end in 0..GRAPHICS_BYTES {
        assert!(decode(&p, &g[..end], &s).is_err());
    }
    for end in 0..STATIC_BYTES {
        assert!(decode(&p, &g, &s[..end]).is_err());
    }
    for offset in (4..PHYSICS_BYTES)
        .step_by(4)
        .filter(|i| *i != 16 && *i != 20)
    {
        let mut bad = p;
        bad[offset..offset + 4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(decode(&bad, &g, &s).is_err(), "float offset {offset}");
    }
    for (offset, value) in [(4, -1_i32), (4, 4), (8, 7), (8, -2)] {
        let mut bad = g;
        bad[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        assert!(decode(&p, &bad, &s).is_err());
    }
}

#[test]
fn identity_does_not_read_player_names_and_rejects_invalid_utf16() {
    let (p, g, s) = fixture();
    let mut with_player = s.to_vec();
    with_player.extend_from_slice(b"PRIVATE PLAYER NAME NOT PART OF THE PREFIX");
    assert!(decode(&p, &g, &with_player).is_err());
    let value = decode(&p, &g, &with_player[..STATIC_BYTES]).unwrap();
    assert!(!serde_json::to_string(&value).unwrap().contains("PRIVATE"));
    for offset in [0, 30, 68, 134] {
        let mut bad = s;
        bad[offset..offset + 2].copy_from_slice(&0xd800_u16.to_le_bytes());
        bad[offset + 2..offset + 4].fill(0);
        assert!(decode(&p, &g, &bad).is_err());
        bad[offset..offset + 2].fill(0);
        assert!(decode(&p, &g, &bad).is_err());
    }
    let mut bad = s;
    for word in bad[134..200].chunks_exact_mut(2) {
        word.copy_from_slice(&65_u16.to_le_bytes());
    }
    assert!(decode(&p, &g, &bad).is_err());
}

#[test]
fn changing_prefix_retries_are_bounded_and_io_errors_propagate() {
    let mut calls = 0;
    assert!(
        repeated_copy::<1>(|| {
            calls += 1;
            Ok([calls])
        })
        .is_err()
    );
    assert_eq!(calls, 16);
    let mut calls = 0;
    let data = repeated_copy::<1>(|| {
        calls += 1;
        Ok([if calls == 1 { 8 } else { 9 }])
    })
    .unwrap();
    assert_eq!((data, calls), ([9], 4));
    assert!(repeated_copy::<1>(|| Err(io::Error::other("fixture read failure"))).is_err());
}

#[test]
fn journal_preserves_packet_wrap_duplicates_and_actual_receive_times() {
    let (p, g, s) = fixture();
    let mut value = decode(&p, &g, &s).unwrap();
    let mut output = vec![];
    let mut journal = Journal::new(&mut output, &value, 3, 10).unwrap();
    journal.sample(&value, 0).unwrap();
    journal.sample(&value, 15_000_000).unwrap();
    value.physics.packet_id = i32::MIN;
    journal.sample(&value, 35_000_000).unwrap();
    let summary = journal.finish().unwrap();
    assert_eq!(
        (
            summary.samples,
            summary.packet_changes,
            summary.duplicate_packets
        ),
        (3, 1, 1)
    );
    assert_eq!(summary.elapsed_ns, 35_000_000);
    let lines: Vec<serde_json::Value> = output
        .split(|b| *b == b'\n')
        .filter(|s| !s.is_empty())
        .map(|s| serde_json::from_slice(s).unwrap())
        .collect();
    assert_eq!(lines.len(), 5);
    assert_eq!(lines[0]["cross_page_synchronized"], false);
    assert_eq!(lines[2]["duplicate_physics_packet"], true);
    assert_eq!(lines[3]["physics"]["packet_id"], i32::MIN);
    assert_eq!(lines[4]["type"], "complete");
}

#[test]
fn journal_never_completes_frozen_or_missing_samples_or_failed_writes() {
    let (p, g, s) = fixture();
    let value = decode(&p, &g, &s).unwrap();
    for samples in 0..=2 {
        let mut output = vec![];
        let mut journal = Journal::new(&mut output, &value, 2, 10).unwrap();
        for i in 0..samples {
            journal.sample(&value, i).unwrap();
        }
        assert!(journal.finish().is_err());
        assert!(
            !String::from_utf8(output)
                .unwrap()
                .contains("\"type\":\"complete\"")
        );
    }
    struct FailsAfterHeader {
        writes: usize,
    }
    impl io::Write for FailsAfterHeader {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if self.writes > 0 {
                return Err(io::Error::other("fixture disk failure"));
            }
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            self.writes += 1;
            Ok(())
        }
    }
    let mut journal = Journal::new(FailsAfterHeader { writes: 0 }, &value, 2, 10).unwrap();
    assert!(journal.sample(&value, 0).is_err());
    assert!(journal.sample(&value, 1).is_err());
    assert!(journal.finish().is_err());
}

#[test]
fn journal_rejects_identity_changes_pause_replay_backwards_time_and_excess_samples() {
    let (p, g, s) = fixture();
    let initial = decode(&p, &g, &s).unwrap();
    for status in [0, 1, 3] {
        let mut paused = initial.clone();
        paused.graphics.status = status;
        assert!(Journal::new(vec![], &paused, 2, 10).is_err());
    }
    let mut journal = Journal::new(vec![], &initial, 2, 10).unwrap();
    journal.sample(&initial, 10).unwrap();
    let mut other = initial.clone();
    other.identity.car = "different".into();
    assert!(journal.sample(&other, 11).is_err());
    other = initial.clone();
    other.graphics.status = 3;
    assert!(journal.sample(&other, 11).is_err());
    assert!(journal.sample(&initial, 9).is_err());
    assert!(journal.sample(&initial, 60_000_000_001).is_err());
    other = initial.clone();
    other.physics.packet_id = -7;
    journal.sample(&other, 20).unwrap();
    assert!(journal.sample(&other, 30).is_err());
    journal.finish().unwrap();
}
