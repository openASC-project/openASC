//! Read-only research acquisition, separate from the engine and game control.
//! Layout evidence and synchronization limitations: docs/TELEMETRY.md.
use serde::Serialize;
use std::io;
pub mod journal;

#[cfg(windows)]
pub mod windows;

pub const PHYSICS_BYTES: usize = 200;
pub const STATIC_BYTES: usize = 200;
pub const GRAPHICS_BYTES: usize = 12;

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Identity {
    pub shared_memory_version: String,
    pub game_version: String,
    pub car: String,
    pub track: String,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Physics {
    pub packet_id: i32,
    pub gas: f32,
    pub brake: f32,
    pub fuel: f32,
    /// Raw source value: no transmission conversion is inferred.
    pub gear: i32,
    pub rpms: i32,
    pub steer_angle: f32,
    pub speed_kmh: f32,
    pub velocity: [f32; 3],
    pub acc_g: [f32; 3],
    /// Source wheel order and units are retained, not converted to engine inputs.
    pub wheel_slip: [f32; 4],
    pub wheel_load: [f32; 4],
    pub wheels_pressure: [f32; 4],
    pub wheel_angular_speed: [f32; 4],
    pub tyre_wear: [f32; 4],
    pub tyre_dirty_level: [f32; 4],
    pub tyre_core_temperature: [f32; 4],
    pub camber_rad: [f32; 4],
    pub suspension_travel: [f32; 4],
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Graphics {
    pub packet_id: i32,
    pub status: i32,
    pub session: i32,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Snapshot {
    pub identity: Identity,
    pub graphics: Graphics,
    pub physics: Physics,
}

pub fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message.into())
}

fn integer(bytes: &[u8], offset: usize) -> i32 {
    i32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}
fn scalar(bytes: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}
fn array<const N: usize>(bytes: &[u8], offset: usize) -> [f32; N] {
    std::array::from_fn(|i| scalar(bytes, offset + i * 4))
}
fn string(bytes: &[u8], start: usize, units: usize) -> io::Result<String> {
    let words: Vec<_> = bytes[start..start + units * 2]
        .chunks_exact(2)
        .map(|v| u16::from_le_bytes([v[0], v[1]]))
        .collect();
    let end = words
        .iter()
        .position(|v| *v == 0)
        .ok_or_else(|| invalid("Unterminated shared-memory string"))?;
    let value = String::from_utf16(&words[..end])
        .map_err(|_| invalid("Invalid UTF-16 in shared-memory string"))?;
    if value.is_empty() || value.chars().any(char::is_control) {
        return Err(invalid("Empty/control character in shared-memory identity"));
    }
    Ok(value)
}

/// Decode owned little-endian bytes, never casting shared memory to Rust structs.
pub fn decode(physics: &[u8], graphics: &[u8], identity: &[u8]) -> io::Result<Snapshot> {
    if physics.len() != PHYSICS_BYTES
        || graphics.len() != GRAPHICS_BYTES
        || identity.len() != STATIC_BYTES
    {
        return Err(invalid("Incorrect shared-memory prefix size"));
    }
    // Integer fields at 0, 16 and 20 are not interpreted as floats.
    if (4..PHYSICS_BYTES)
        .step_by(4)
        .filter(|i| *i != 16 && *i != 20)
        .any(|i| !scalar(physics, i).is_finite())
    {
        return Err(invalid("Non-finite shared-memory physics field"));
    }
    let status = integer(graphics, 4);
    let session = integer(graphics, 8);
    if !(0..=3).contains(&status) || !(-1..=6).contains(&session) {
        return Err(invalid("Unknown original-AC status/session value"));
    }
    Ok(Snapshot {
        identity: Identity {
            shared_memory_version: string(identity, 0, 15)?,
            game_version: string(identity, 30, 15)?,
            car: string(identity, 68, 33)?,
            track: string(identity, 134, 33)?,
        },
        graphics: Graphics {
            packet_id: integer(graphics, 0),
            status,
            session,
        },
        physics: Physics {
            packet_id: integer(physics, 0),
            gas: scalar(physics, 4),
            brake: scalar(physics, 8),
            fuel: scalar(physics, 12),
            gear: integer(physics, 16),
            rpms: integer(physics, 20),
            steer_angle: scalar(physics, 24),
            speed_kmh: scalar(physics, 28),
            velocity: array(physics, 32),
            acc_g: array(physics, 44),
            wheel_slip: array(physics, 56),
            wheel_load: array(physics, 72),
            wheels_pressure: array(physics, 88),
            wheel_angular_speed: array(physics, 104),
            tyre_wear: array(physics, 120),
            tyre_dirty_level: array(physics, 136),
            tyre_core_temperature: array(physics, 152),
            camber_rad: array(physics, 168),
            suspension_travel: array(physics, 184),
        },
    })
}

/// Bounded double-copy filter. Equal copies reduce observed tearing; they do not
/// prove atomicity, even for a single page. No producer seqlock is assumed.
pub fn repeated_copy<const N: usize>(
    mut read: impl FnMut() -> io::Result<[u8; N]>,
) -> io::Result<[u8; N]> {
    for _ in 0..8 {
        let first = read()?;
        let second = read()?;
        if first == second {
            return Ok(second);
        }
    }
    Err(invalid(
        "Shared-memory prefix changed during all eight read attempts",
    ))
}

pub fn validate_live(snapshot: &Snapshot, expected: &Identity) -> io::Result<()> {
    if snapshot.identity != *expected {
        return Err(invalid(
            "Shared-memory version, car or track changed during capture",
        ));
    }
    if snapshot.graphics.status != 2 {
        return Err(invalid(
            "Capture requires AC_LIVE; off, pause and replay are rejected",
        ));
    }
    Ok(())
}
