# HIL (Hardware-In-the-Loop) Test Harness

Plays an OpenRocket simulation through the **actual flight firmware** on
the ESP32-S3, then verifies the firmware's KF/FSM output against the
ground truth.

Everything is one script: **`hil.py`**.

## Requirements

```
pip install pyserial numpy
```

## Quickest path

Put OpenRocket CSV in this folder (e.g. `HIL.csv`), connect the board, run:

```
python hil.py HIL.csv
```

This auto-runs the full pipeline:
1. `convert` — `HIL.csv` → `flight.hil` (binary trace, with sensor noise)
2. `run`     — uploads to board, drives flight, captures flash dump → `flight.bin`
3. `verify`  — parses dump → `flight_nav.csv`, `flight_baro.csv`,
               `flight_imu.csv`, `flight_events.csv`, `flight_compare.csv`,
               and prints RMSE summary + FSM event table.

## Individual subcommands

```
python hil.py convert HIL.csv -o flight.hil [--summary]
python hil.py run     flight.hil -o dump.bin [-p COM7] [--no-erase]
python hil.py verify  dump.bin --truth HIL.csv -p flight
```

## Common options

| Option | Default | Meaning |
|--------|---------|---------|
| `--prefix flight` | `flight` | output file prefix for `all` mode |
| `-p COM7` | auto-detect | serial port |
| `-b 921600` | 921600 | baud |
| `--accel-noise 0.025` | 0.025 m/s² | 1-σ accel noise |
| `--gyro-noise 0.005`  | 0.005 rad/s | 1-σ gyro noise |
| `--baro-noise 0.30`   | 0.30 m | 1-σ baro noise |
| `--no-erase` | off | skip flash erase (ERASE takes 1-3 min for 32MB) |
| `--manual-stop` + `--wait-after-start 90` | off | stop after fixed delay |
| `--flight-timeout 120` | 120s | max wait for autonomous STOPPED. |

## Outputs (verify)

| File | Columns |
|------|---------|
| `<prefix>_nav.csv` | `t, pN, pE, pD, vN, vE, vD, qw, qx, qy, qz` |
| `<prefix>_baro.csv` | `t, alt` |
| `<prefix>_imu.csv` | `t, gx, gy, gz, ax, ay, az` (int16 LSB) |
| `<prefix>_events.csv` | `t, phase, event_id, phase_name, event_name` |
| `<prefix>_compare.csv` | `t, alt_kf, alt_truth, err_alt, vvel_kf, vvel_truth, err_vvel, cos_tilt_kf, cos_tilt_truth, err_cos_tilt` |

Console prints RMSE for altitude / vertical velocity / cos(tilt) and the
FSM event timeline.

## Firmware HIL command

The firmware adds **one** command (`HIL`):

```
HIL → enters HIL mode, awaits binary trace on Serial
```

After receiving the trace, IMU_Task / BMP_Task pull samples from PSRAM
via a single `if (hil.isActive())` branch — the entire algorithm path
(NAV, KF, flight FSM, logger, BLE) runs identically to a real flight.

`CALIBRATE` is a no-op in HIL mode. Other commands (`ERASE`, `START`,
`STOP`, `PARSE`, `REBOOT`) behave normally.

## Notes

- Trace size ~500 KB for 78 s flight. Upload at ~450 KB/s over USB-CDC.
- ESP32-S3 USB-CDC RX buffer is sized to 16 KB in firmware setup; do
  not reduce.
- Trace is held in PSRAM (8 MB on N16R8V).
- LAND auto-detect requires `|vvel| < 1 m/s`. OpenRocket descent speeds
  are usually higher, so the runner times out and sends STOP manually.
