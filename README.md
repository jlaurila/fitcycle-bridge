# fitcycle-bridge

An ESP32-C6 bridge that lets a Garmin watch use a **Tunturi FitCycle 50i** exercise bike as a smart trainer, with a small status screen that tracks your total energy output.

```
FitCycle 50i  <── BLE (unencrypted) ──  ESP32-C6  ── BLE (bonded) ──>  Garmin Fenix 8
                                           │
                                      1.47" LCD: watts, rpm, level, kJ
```

## Why a bridge?

The FitCycle 50i speaks standard Bluetooth FTMS (Fitness Machine Service), so in theory a Garmin should pair with it directly. In practice:

1. **The bike refuses BLE pairing.** It rejects every pairing request (legacy and LE Secure Connections). Garmin watches insist on bonding with a trainer, so pairing hangs forever at "connecting".
2. **The bike's ERG and simulation modes are unusable.** Both behave as a jittery power target that constantly hunts between resistance levels (audible motor noise). Once either mode has been used, the bike ignores *Set Target Resistance*, *Reset* and *Stop* until the BLE link drops.

The bridge connects to the bike unencrypted, offers the watch a normal bondable FTMS trainer, and implements ERG and simulation itself on top of plain resistance levels, the one mode the bike does well.

## Features

- **FTMS trainer for the watch:** speed, cadence, distance, resistance, power, elapsed time. Data is re-encoded to fit 20-byte notifications, because the Fenix doesn't raise the MTU.
- **Cycling Power service** as a fallback, if you'd rather pair it as a power meter. Crank revolutions are synthesised from cadence.
- **Simulation (free ride, courses):** grade maps to a fixed resistance level, so it feels like a road and the motor stays quiet on a steady grade.
- **ERG (target power):** the bridge computes the level from a fitted power model and cadence. Hysteresis and persistence keep level changes to a minimum.
- **Set resistance:** passed straight through.
- **Self-healing:** if the bike drops or gets stuck in its own ERG mode, the bridge reconnects and restores the level. Distance and time keep counting on the watch.
- **Status screen:** connection lights, mode, watts as seven-segment digits, rpm, level, plus session and lifetime energy (kJ). The lifetime total persists in flash.

## Hardware

- [Waveshare ESP32-C6-LCD-1.47](https://www.waveshare.com/wiki/ESP32-C6-LCD-1.47): ESP32-C6, 1.47" ST7789 172×320 LCD. No wiring needed; just power it over USB near the bike.
- Tunturi FitCycle 50i (reports as `FitCycle 50i-xxx`, made by ChangYow, model `SM5081E BLE`, firmware `21.00.21.00`). Other FTMS bikes from the same OEM may work too.
- Tested with a Garmin Fenix 8.

## Build and flash

Requires [PlatformIO](https://platformio.org/) (`pip install platformio`). The ESP32-C6 needs Arduino core 3.x, so the project uses the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform.

```sh
pio run -e bridge -t upload      # the bridge (default)
pio device monitor               # serial log, 115200 baud
```

## Pairing with the watch

1. Wake the bike console (pedal a few strokes). The bridge finds the bike within a few seconds and the **BIKE** light turns green.
2. On the watch: **Sensors & Accessories → Add New → Search All**, and pick the trainer ("Tunturi Bridge", shown as *Indoor Trainer*).
3. Start an **Indoor Bike** activity. The **WATCH** light turns green and the mode shows `SIM`, `ERG` or `RES`.

The bike only accepts one connection, so once the bridge is running, the Tunturi app and other apps can't connect to the bike.

## Tuning

At the top of the resistance-control section in `src/bridge/main.cpp`:

| Constant | Default | Meaning |
|---|---|---|
| `SIM_BASE_LEVEL` | 5 | resistance level on a flat road |
| `SIM_LEVELS_PER_PERCENT` | 2 | extra levels per 1 % of grade |
| `ERG_HYSTERESIS_LEVELS` | 1.0 | how far off the ideal level must be before changing |
| `ERG_PERSIST_MS` | 2000 | ...and for how long |
| `ERG_REVERSE_MS` | 3000 | minimum wait before changing back the other way |

## Protocol notes

What the FitCycle 50i exposes, from the sniffer:

| Service | Characteristics |
|---|---|
| `0x1826` FTMS | Feature `0x2ACC`, Indoor Bike Data `0x2AD2` (notify, 2 Hz, 30 bytes), Training Status `0x2AD3`, Resistance Range `0x2AD6` (1.0–32.0, step 1), Power Range `0x2AD8` (0–999 W), Control Point `0x2AD9`, Machine Status `0x2ADA` |
| `0xFFF0` vendor | `0xFFF1` notify, `0xFFF2` write (silent during rides) |
| `02f00000-…-ff00` vendor | `ff00`–`ff03` (`ff02` reads `ntf_enable`) |
| `0x180A` Device Info | `ChangYow` / `SM5081E BLE` / `21.00.21.00` |

Quirks:

- **Pairing** fails within ~60 ms with SMP reason `0x08` (unspecified).
- **MTU:** the bike starts its own MTU exchange right after connecting. If the central also requests one, the two collide and the connection fails, so the bridge skips its own request.
- **ERG lock-in:** after `0x05` (target power) or `0x11` (simulation), the bike stays in ERG until disconnected. `0x01` reset, `0x07` start, `0x08` stop and `0x05` with 0 W don't release it. `0x08 01` (stop) also freezes the reported data.
- **Set Target Resistance** (`0x04`) accepts both the spec's 1-byte and Garmin's 2-byte parameter. The bike reports the resulting level (1–32) in Indoor Bike Data.
- **Power model:** the bike's reported power fits `P = (19.5 + 10 × level) × (rpm / 70)^1.72` within about 5 W across all 32 levels.
- **Hand-grip HR** reads about 80 bpm even when nobody holds the grips, so the bridge doesn't forward it.

## Tools

- `src/sniffer/`: BLE explorer firmware (`pio run -e sniffer -t upload`). It scans, connects, dumps the GATT table, subscribes to everything and decodes FTMS data. Serial commands include `p` (try pairing), `q` (request control) and `w <hex>` (write the FTMS control point).
- `tools/capture.py`: logs serial output to `logs/` and can send timed commands, e.g. `python tools/capture.py --seconds 60 --send "@15 q" --send "@17 w 04 64 00"`.

## License

[MIT](LICENSE). An independent hobby project, not affiliated with or endorsed by Tunturi or Garmin. Use at your own risk.
