# Explicit suspension workbench inputs

`suspension-travel.json` contains original synthetic mass, inertia, gravity and spring parameters for the fixed-frame native dynamics workbench. They are not copied from or calibrated to Assetto Corsa. The zero gravity vector is intentional: the example isolates the mounted spring response.

```sh
cargo run --locked -p asc-app -- suspension-run ./assettocorsa/content/cars/bmw_m3_e30 front examples/suspension-travel.json
```

The selected local car supplies raw DWB/STRUT geometry only. No game content is included in this directory. See the [configuration and limitations](../docs/USER_GUIDE.md#native-fixed-frame-suspension-dynamics) and [validation](../docs/FORMATS.md#fixed-frame-suspension-dynamics-validation).
