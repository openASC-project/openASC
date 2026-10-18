//! Only existing mappings are opened. ReadProcessMemory copies our own mapped
//! address range, avoiding Rust references into concurrently modified memory.
use crate::{GRAPHICS_BYTES, PHYSICS_BYTES, STATIC_BYTES, Snapshot, decode, repeated_copy};
use std::io;
use windows_sys::Win32::{
    Foundation::{CloseHandle, HANDLE},
    System::{
        Diagnostics::Debug::ReadProcessMemory,
        Memory::{
            FILE_MAP_READ, MEMORY_MAPPED_VIEW_ADDRESS, MapViewOfFile, OpenFileMappingW,
            UnmapViewOfFile,
        },
        Threading::GetCurrentProcess,
    },
};

struct Mapping<const N: usize> {
    handle: HANDLE,
    view: MEMORY_MAPPED_VIEW_ADDRESS,
}
impl<const N: usize> Mapping<N> {
    fn open(name: &str) -> io::Result<Self> {
        let wide: Vec<_> = name.encode_utf16().chain(Some(0)).collect();
        // SAFETY: NUL-terminated name lives through call. Handle is owned below.
        let handle = unsafe { OpenFileMappingW(FILE_MAP_READ, 0, wide.as_ptr()) };
        if handle.is_null() {
            let error = io::Error::last_os_error();
            return Err(io::Error::new(
                error.kind(),
                format!(
                    "Cannot open {name}: {}. Start an original Assetto Corsa session first.",
                    error
                ),
            ));
        }
        // SAFETY: valid mapping handle, bounded prefix, read-only access.
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, N) };
        if view.Value.is_null() {
            let error = io::Error::last_os_error();
            // SAFETY: owned handle; no view was created.
            unsafe {
                CloseHandle(handle);
            }
            return Err(error);
        }
        Ok(Self { handle, view })
    }
    fn copy(&self) -> io::Result<[u8; N]> {
        let mut bytes = [0; N];
        let mut read = 0;
        // SAFETY: our mapped view remains alive, target is an owned N-byte array.
        // No external process handle is opened and no foreign Rust reference is made.
        let ok = unsafe {
            ReadProcessMemory(
                GetCurrentProcess(),
                self.view.Value,
                bytes.as_mut_ptr().cast(),
                N,
                &mut read,
            )
        };
        if ok == 0 {
            return Err(io::Error::last_os_error());
        }
        if read != N {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Short mapped-prefix read",
            ));
        }
        Ok(bytes)
    }
    fn snapshot(&self) -> io::Result<[u8; N]> {
        repeated_copy(|| self.copy())
    }
}
impl<const N: usize> Drop for Mapping<N> {
    fn drop(&mut self) {
        // SAFETY: each resource is owned and released once, after all reads.
        unsafe {
            UnmapViewOfFile(self.view);
            CloseHandle(self.handle);
        }
    }
}

pub struct Connection {
    physics: Mapping<PHYSICS_BYTES>,
    graphics: Mapping<GRAPHICS_BYTES>,
    identity: Mapping<STATIC_BYTES>,
}
impl Connection {
    pub fn open() -> io::Result<Self> {
        Ok(Self {
            physics: Mapping::open("Local\\acpmf_physics")?,
            graphics: Mapping::open("Local\\acpmf_graphics")?,
            identity: Mapping::open("Local\\acpmf_static")?,
        })
    }
    /// Pages are copied separately and have unrelated packet IDs.
    pub fn snapshot(&self) -> io::Result<Snapshot> {
        let identity = self.identity.snapshot()?;
        let graphics = self.graphics.snapshot()?;
        let physics = self.physics.snapshot()?;
        let identity_after = self.identity.snapshot()?;
        let graphics_after = self.graphics.snapshot()?;
        if identity != identity_after || graphics[4..12] != graphics_after[4..12] {
            return Err(crate::invalid(
                "Session identity/status changed across prefix reads",
            ));
        }
        decode(&physics, &graphics, &identity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};
    use windows_sys::Win32::{
        Foundation::INVALID_HANDLE_VALUE,
        System::Memory::{CreateFileMappingW, FILE_MAP_WRITE, PAGE_READWRITE},
    };

    fn fixture<const N: usize>(data: [u8; N]) -> (Mapping<N>, Mapping<N>) {
        static SERIAL: AtomicU64 = AtomicU64::new(0);
        let name = format!(
            "Local\\openasc-telemetry-test-pages-{}-{}",
            std::process::id(),
            SERIAL.fetch_add(1, Ordering::Relaxed)
        );
        let wide: Vec<_> = name.encode_utf16().chain(Some(0)).collect();
        // SAFETY: uniquely named paging-file fixture; no game mappings used.
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                std::ptr::null(),
                PAGE_READWRITE,
                0,
                N as u32,
                wide.as_ptr(),
            )
        };
        assert!(!handle.is_null());
        // SAFETY: owned fixture handle, valid length and write access.
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, N) };
        assert!(!view.Value.is_null());
        let producer = Mapping { handle, view };
        // SAFETY: private writable fixture initialized before reader exists.
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), producer.view.Value.cast(), N);
        }
        (producer, Mapping::open(&name).unwrap())
    }

    #[test]
    fn three_read_only_pages_decode_and_feed_a_complete_fixture_journal() {
        let mut physics = [0; PHYSICS_BYTES];
        physics[0..4].copy_from_slice(&40_i32.to_le_bytes());
        physics[28..32].copy_from_slice(&72.0_f32.to_le_bytes());
        physics[196..200].copy_from_slice(&0.125_f32.to_le_bytes());
        let mut graphics = [0; GRAPHICS_BYTES];
        graphics[0..4].copy_from_slice(&990_i32.to_le_bytes());
        graphics[4..8].copy_from_slice(&2_i32.to_le_bytes());
        let mut identity = [0; STATIC_BYTES];
        for (offset, value) in [
            (0, "1.7"),
            (30, "1.16.4"),
            (68, "fixture_car"),
            (134, "fixture_track"),
        ] {
            for (i, unit) in value.encode_utf16().enumerate() {
                identity[offset + i * 2..offset + i * 2 + 2].copy_from_slice(&unit.to_le_bytes());
            }
        }
        let (producer_physics, physics) = fixture(physics);
        let (_producer_graphics, graphics) = fixture(graphics);
        let (_producer_identity, identity) = fixture(identity);
        let reader = Connection {
            physics,
            graphics,
            identity,
        };
        let initial = reader.snapshot().unwrap();
        assert_eq!(initial.identity.car, "fixture_car");
        assert_eq!(initial.physics.speed_kmh, 72.0);
        assert_eq!(initial.physics.suspension_travel[3], 0.125);
        assert_eq!(initial.graphics.packet_id, 990);
        let mut output = vec![];
        let mut journal = crate::journal::Journal::new(&mut output, &initial, 2, 10).unwrap();
        journal.sample(&initial, 0).unwrap();
        // SAFETY: only producer-owned fixture memory is written, between reads.
        unsafe {
            std::ptr::copy_nonoverlapping(
                41_i32.to_le_bytes().as_ptr(),
                producer_physics.view.Value.cast(),
                4,
            );
        }
        drop(producer_physics);
        let next = reader.snapshot().unwrap();
        assert_eq!(next.physics.packet_id, 41);
        journal.sample(&next, 11_000_000).unwrap();
        assert_eq!(journal.finish().unwrap().packet_changes, 1);
        assert!(
            String::from_utf8(output)
                .unwrap()
                .contains("\"type\":\"complete\"")
        );
    }

    #[test]
    fn reads_only_an_owned_named_fixture_and_releases_the_mapping() {
        static SERIAL: AtomicU64 = AtomicU64::new(0);
        let name = format!(
            "Local\\openasc-telemetry-test-{}-{}",
            std::process::id(),
            SERIAL.fetch_add(1, Ordering::Relaxed)
        );
        assert!(Mapping::<16>::open(&name).is_err());
        let wide: Vec<_> = name.encode_utf16().chain(Some(0)).collect();
        // SAFETY: create only our unique paging-file fixture, never an AC name.
        let handle = unsafe {
            CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                std::ptr::null(),
                PAGE_READWRITE,
                0,
                16,
                wide.as_ptr(),
            )
        };
        assert!(!handle.is_null());
        // SAFETY: owned fixture handle with write access and sufficient size.
        let view = unsafe { MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, 16) };
        assert!(!view.Value.is_null());
        let data: [u8; 16] = std::array::from_fn(|i| i as u8 * 7);
        // SAFETY: unique writable 16-byte fixture view; no other writer exists.
        unsafe {
            std::ptr::copy_nonoverlapping(data.as_ptr(), view.Value.cast(), 16);
        }
        let reader = Mapping::<16>::open(&name).unwrap();
        assert_eq!(reader.snapshot().unwrap(), data);
        // Keep the read mapping alive after releasing producer resources.
        unsafe {
            UnmapViewOfFile(view);
            CloseHandle(handle);
        }
        assert_eq!(reader.copy().unwrap(), data);
        drop(reader);
        assert!(Mapping::<16>::open(&name).is_err());
    }
}
