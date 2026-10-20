# Explicit suspension workbench inputs

`suspension-travel.json` contains original synthetic mass, inertia, gravity and spring parameters for the fixed-frame native dynamics workbench. They are not copied from or calibrated to Assetto Corsa. The zero gravity vector is intentional: the example isolates the mounted spring response.

```sh
cargo run --locked -p asc-app -- suspension-run ./assettocorsa/content/cars/bmw_m3_e30 front examples/suspension-travel.json
```

The selected local car supplies raw DWB/STRUT geometry only. No game content is included in this directory. See the [configuration and limitations](../docs/USER_GUIDE.md#native-fixed-frame-suspension-dynamics) and [validation](../docs/FORMATS.md#fixed-frame-suspension-dynamics-validation).

`chassis-suspension.json` supplies an explicitly mounted floating chassis and four uprights. Its masses, tensors, COM offsets, mounting positions/rotations and spring laws are synthetic. In particular, the right-side rotations are example proper rotations, not a verified Assetto Corsa mirroring/setup conversion. Gravity is zero and no tyres/ground are present, so this isolates internal recoil and momentum behavior.

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-suspension.json
```

Car paths resolve relative to the JSON file. See the [coupled configuration guide](../docs/USER_GUIDE.md#native-coupled-chassis-dynamics) and [validation](../docs/FORMATS.md#coupled-chassis-dynamics-validation).

`chassis-wheels.json` adds four separate axisymmetric wheel rotors and internal wheel/upright motors. The synthetic per-corner mass is explicitly split into a 15 kg upright and 20 kg wheel; their tensors are separate too. Its axle conventions and +/-8 N m motor values are diagnostic choices, not a vehicle calibration, driveline or brake law. Zero gravity and absent road contact isolate internal recoil and motor work.

```sh
cargo run --locked -p asc-app -- chassis-run examples/chassis-wheels.json
```

See [wheel inputs and reactions](../docs/USER_GUIDE.md#native-wheel-rotation-and-motor-reactions) and [validation](../docs/FORMATS.md#axisymmetric-wheel-dynamics-validation).
