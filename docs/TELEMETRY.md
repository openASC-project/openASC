# Original-game telemetry research

`asc-telemetry` is a separate Rust research tool under `tools/asc-telemetry/`. It reads existing Windows named shared memory and writes a bounded JSONL log. This provides an acquisition path for future original-game comparisons; no live Assetto Corsa capture or physics parity has been verified yet.

## Capture an existing session

Build with `cargo build --locked -p asc-telemetry`. Start an original Assetto Corsa (2014) session yourself, enter the car and leave the session live rather than paused or in replay. The collector does not launch the game, send controls or change setup files.

```sh
cargo run --locked -p asc-telemetry -- capture reports/ac-baseline-001.jsonl --expect-ac-version 1.16.4 --expect-sm-version 1.7 --samples 1000 --interval-ms 10
```

Create `reports/` first. Use a new filename for each capture. The example versions are explicit expectations, not a claim of live validation on that version pair. A mismatch fails before creating the output; update expectations only after checking the actual installation and protocol. Both version options are mandatory and accept numeric 1.x identifiers. ACC and Assetto Corsa EVO are outside this tool's scope.

The collector opens `Local\acpmf_physics`, `Local\acpmf_graphics` and `Local\acpmf_static` using read-only mapping access. Missing mappings, unsupported operating systems, invalid arguments, initial version mismatch and non-live initial status produce failure before output creation. Existing files are never overwritten. Access errors retain their operating-system diagnostic. There is no fallback that creates game mappings.

The bounds are 2–6000 samples, 1–1000 milliseconds between polls, and at most 60000 milliseconds for the sample-count/interval product. A separate elapsed-time limit stops a capture after 60 seconds. Sleep and I/O delays can lengthen the requested interval; there is no catch-up burst or hard real-time promise. The log retains actual monotonic receive timestamps after the page reads, not simulation time. A blocked filesystem or OS call cannot be forcibly interrupted by this time check.

## Recorded subset

All records are decoded from owned little-endian bytes using explicit offsets. There is no native-struct cast, SDK binary or extracted game file in the implementation.

| Page | Prefix bytes | Fields |
| --- | --- | --- |
| Physics | 200 | Packet ID, gas, brake, fuel, raw gear, RPM, steer angle, speed in km/h, velocity, acceleration G, four-element wheel slip/load/pressure/angular-speed/wear/dirt/core-temperature/camber/travel arrays |
| Graphics | 12 | Its own packet ID, status and session enum values |
| Static | 200 | UTF-16 shared-memory version, game version, car model and track identifiers; session/car count slots are skipped |

The static prefix ends immediately before player names. Player identity fields and raw memory dumps are not read or saved. Car/track identifiers and source physics values are retained without converting coordinate frames, wheel order, transmission indexing or undocumented units into engine inputs. Non-finite physics values, malformed UTF-16, unterminated or empty identity strings and unknown status/session enum values fail decoding. A finite value is not proof that a sample is physically plausible or untorn.

Track configuration, car setup, complete tyre state, aids and weather are not recorded. The header declares these gaps. Record those conditions separately before attempting a controlled comparison; car and track identifiers alone do not identify an equivalent experiment.

## Consistency and completion

For each prefix the reader requires two equal consecutive copies, retrying at most eight pairs. It reads identity and graphics status/session again around the physics read to detect observed session changes. Equal copies do not establish an atomic snapshot: a producer could pause halfway through an update. There is no verified producer seqlock. The pages are read separately and their packet IDs are unrelated; cross-page synchronization is explicitly false in the header.

The JSONL stream contains one `header`, numbered `sample` records, and a final `complete` footer only after the requested sample count and at least one change of physics packet ID. Consecutive equal IDs are logged as duplicates; any unequal IDs count as a transition, including signed wrap or reset. Transitions do not measure the producer's frequency or identify skipped simulation steps. The summary reports samples, transitions, consecutive duplicates and final elapsed time.

Pause, replay, observed car/track/version changes, read errors, non-finite data or output failures terminate acquisition. Any file without a complete footer is incomplete; an I/O error can leave a partial last line. Keep that distinction when analyzing traces. A complete footer establishes collector completion, not correct ABI interpretation, consistent samples or matching original physics.

## Evidence and validation

The field order, four-byte packing, UTF-16 widths and mapping names were checked against the independently maintained [C# shared-memory implementation](https://github.com/mdjarv/assettocorsasharedmemory), specifically its [physics declarations](https://github.com/mdjarv/assettocorsasharedmemory/blob/master/Physics.cs), [static declarations](https://github.com/mdjarv/assettocorsasharedmemory/blob/master/StaticInfo.cs), [graphics declarations](https://github.com/mdjarv/assettocorsasharedmemory/blob/master/Graphics.cs) and [connection names](https://github.com/mdjarv/assettocorsasharedmemory/blob/master/AssettoCorsa.cs). That project links the original [official shared-memory reference](https://www.assettocorsa.net/forum/index.php?threads/shared-memory-reference.3352/); the official page was unavailable during this implementation. This is declaration evidence from an existing consumer, not live verification of the installed game's ABI. No implementation code or game/SDK bytes were copied.

Windows access follows Microsoft's [OpenFileMappingW](https://learn.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-openfilemappingw) and [MapViewOfFile](https://learn.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-mapviewoffile) interfaces. [ReadProcessMemory](https://learn.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-readprocessmemory) copies the collector's own mapped address into an owned buffer, without opening another process or creating Rust references into concurrently modified bytes. Handles and views are released on drop.

```sh
cargo test --locked -p asc-telemetry
```

Tests use original bytes with distinct field values, invalid lengths/UTF-16/non-finite values, changing-copy retries, packet wrap/duplicates, interrupted journals and output failures. Windows tests create uniquely named `openasc-telemetry-test-*` mappings, verify reads after the producer handle closes, check final resource release, and feed all three fixture pages through the connection and journal. They never create or write an `acpmf_*` mapping. CLI tests cover argument rejection and overwrite refusal. Pure decoding/journal tests also run on Linux. A local availability check returned missing game mappings and left no output file; real-game acquisition remains unverified.
